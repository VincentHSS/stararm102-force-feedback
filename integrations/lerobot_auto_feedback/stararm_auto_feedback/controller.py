from __future__ import annotations

# 官方遥操作接入顺序：连接设备后初始化，send_action 后 update，退出时 close。
# 本模块复用已连接设备；从臂运动仍由 LeRobot 官方遥操作循环负责。

import json
import logging
import math
import os
import time
from pathlib import Path

logger = logging.getLogger(__name__)
NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)


def settings(path=None):
    # Nm 参数用于归一化从臂负载；mW 参数是 Leader SDK 的功率指令，两者不能直接等同。
    cfg = {
        "enabled": True,
        # 参考值是可调的反馈标尺，并非电机额定扭矩；floor 过滤轻微负载。
        "torque_reference_nm": dict(zip(NAMES, (5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0))),
        "torque_floor_nm": dict(zip(NAMES, (0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05))),
        # 协议中 0 表示使用保护功率上限，因此最小功率从 1 开始。
        "power_min_mw": 1,
        "power_max_mw": 1200,
        "power_deadband_mw": 20,
        "sample_hz": 30.0,
        # 锁定与释放采用不同角差阈值，配合持续时间确认，减少临界点反复切换。
        "lock_deg": 20.0,
        "release_deg": 8.0,
        "window_s": 0.4,
        "tracking_ratio": 0.35,
        "tracking_min_deg": 1.5,
        "convergence_deg": 1.0,
        "minimum_load_ratio": 0.3,
        "confirm_windows": 1,
        "release_stable_s": 0.3,
        "hold_mw": 6000,
        "relock_s": 1.0,
    }
    if path:
        values = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        unknown = set(values) - set(cfg)
        if unknown:
            raise ValueError(f"Unknown feedback settings: {sorted(unknown)}")
        cfg.update(values)
    if not isinstance(cfg["enabled"], bool):
        raise ValueError("enabled must be boolean")
    for key in ("torque_reference_nm", "torque_floor_nm"):
        if not isinstance(cfg[key], dict) or set(cfg[key]) != set(NAMES):
            raise ValueError(f"{key} must define all seven joint names")
        for value in cfg[key].values():
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError(f"{key} must contain finite numbers")
    for key, value in cfg.items():
        if key == "enabled" or key.startswith("torque_"):
            continue
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0
        ):
            raise ValueError(f"{key} must be a finite nonnegative number")
    for name in NAMES:
        if not 0 <= cfg["torque_floor_nm"][name] < cfg["torque_reference_nm"][name]:
            raise ValueError(f"{name}: torque floor must be below reference")
    if not 1 <= cfg["power_min_mw"] <= cfg["power_max_mw"] <= 65535:
        raise ValueError("power range must be within 1..65535mW")
    if not 1 <= cfg["hold_mw"] <= 65535 or not 1 <= cfg["sample_hz"] <= 100:
        raise ValueError("invalid holding power or sample rate")
    if not cfg["release_deg"] < cfg["lock_deg"] or cfg["window_s"] < 0.05:
        raise ValueError("invalid angle thresholds or observation window")
    if cfg["relock_s"] < 0.05 or cfg["release_stable_s"] < 0.05:
        raise ValueError("relock and release confirmation intervals must be >= 0.05s")
    if not 0 <= cfg["tracking_ratio"] <= 1 or not 0 <= cfg["minimum_load_ratio"] <= 1:
        raise ValueError("ratios must be within 0..1")
    if cfg["confirm_windows"] < 1 or cfg["confirm_windows"] != int(cfg["confirm_windows"]):
        raise ValueError("confirm_windows must be a positive integer")
    return cfg


class AutomaticFeedback:
    """Only adds Leader damping/holding; never commands the follower or opens a port."""

    def __init__(self, leader, robot, *, config=None, clock=time.monotonic):
        self.leader, self.robot, self.clock = leader, robot, clock
        self.cfg = settings(os.environ.get("STARARM_FEEDBACK_CONFIG")) if config is None else config
        self.active = False
        self.closed = False
        self.locked = False
        self.managed = set()
        self.sent = {}
        # triggers 记录触发堵转的关节；任一关节未恢复时，整个 Leader 继续保持。
        self.triggers = set()
        self.windows = {}
        self.over = dict.fromkeys(NAMES, 0)
        self.release_since = {}
        self.fault = False
        self.fault_clear_since = None
        self.next_sample = 0.0
        self.next_relock = 0.0
        self._monitor_ok = False
        self._original_monitor = None
        if not self.cfg["enabled"] or getattr(leader, "name", None) not in (
            "rebot_arm_102_leader",
            "rebot_102_leader",
        ):
            return
        robot_name = getattr(robot, "name", None)
        if robot_name == "seeed_b601_dm_follower":
            self.builtin = False
        elif robot_name == "rebot_b601_follower" and robot.config.motor_family == "dm":
            self.builtin = True
        else:
            return
        self.ids = dict(leader.config.joint_ids)
        if set(self.ids) != set(NAMES) or len(set(self.ids.values())) != 7:
            raise ValueError("Automatic feedback requires the standard seven-joint Leader mapping")
        if set(robot.motors) != set(NAMES):
            raise ValueError("B601 motor names do not match the Leader")
        self.bus = leader.bus
        if not callable(getattr(self.bus, "set_stop_mode", None)) or not callable(
            getattr(self.bus, "sync_monitor", None)
        ):
            raise TypeError("Official FashionStarServo set_stop_mode/sync_monitor API required")
        self.leader_scale = getattr(leader.config, "joint_directions", dict.fromkeys(NAMES, 1.0))
        self.follower_scale = (
            robot.profile.joint_directions if self.builtin else robot.config.joint_directions
        )
        for mapping in (self.leader_scale, self.follower_scale):
            for name in NAMES:
                value = mapping[name]
                if not isinstance(value, (int, float)) or not math.isfinite(value) or value == 0:
                    raise ValueError(f"{name}: invalid official direction/scale")
        # 只包装当前 Leader 的监测调用，不创建第二条串口连接或独立控制线程。
        self._original_monitor = self.bus.sync_monitor

        def monitor(*args, **kwargs):
            self._monitor_ok = False
            result = self._original_monitor(*args, **kwargs)
            # reliable=False 的缓存回退数据不作为本轮有效反馈。
            self._monitor_ok = all(
                result.get(i) is not None and bool(getattr(result[i], "reliable", False))
                for i in self.ids.values()
            )
            return result

        self._wrapped_monitor = monitor
        self.bus.sync_monitor = monitor
        self.active = True
        logger.info("StarArm automatic Leader feedback enabled.")

    def _mode(self, name, mode, power):
        # 先登记再发送：即使写入中途异常，退出清理仍会尝试释放这个关节。
        self.managed.add(name)
        self.bus.set_stop_mode(self.ids[name], mode=mode, power=int(power))

    def _lock(self, now, reason):
        # 0x11 请求舵机保持；定期重发以维持该模式，不额外下发角度目标。
        for name in NAMES:
            self._mode(name, 0x11, self.cfg["hold_mw"])
        if not self.locked:
            logger.warning("Leader held: %s", reason)
        self.locked = True
        self.next_relock = now + self.cfg["relock_s"]

    def _sample(self, sent_action):
        if not self._monitor_ok:
            raise ValueError("Leader feedback unavailable")
        if not isinstance(sent_action, dict):
            raise ValueError("Official send_action must return the actual action dictionary")
        # 使用官方 send_action 返回的实际指令，避免把限幅前的输入当作目标。
        target, actual, loads = {}, {}, {}
        for name in NAMES:
            # 从臂 SDK 返回缓存状态；此接口没有逐帧时间戳，无法据此保证数据新鲜度。
            state = self.robot.motors[name].get_state()
            if state is None or state.status_code != 1:
                raise ValueError("Follower feedback unavailable or motor not enabled")
            raw_target = float(sent_action[f"{name}.pos"])
            raw_actual = math.degrees(state.pos)
            torque = abs(state.torq)
            if not all(math.isfinite(v) for v in (raw_target, raw_actual, torque)):
                raise ValueError("Non-finite follower feedback")
            # 内置 LeRobot 返回公共坐标目标；Seeed 外置插件返回电机坐标目标。
            # state.pos 始终是电机弧度，先转角度，再还原方向与比例。
            if self.builtin:
                raw_actual /= self.follower_scale[name]
            else:
                raw_target /= self.follower_scale[name]
                raw_actual /= self.follower_scale[name]
            # 统一到 Leader 等效角度域，确保角差与跟随量使用同一单位和方向。
            target[name] = raw_target / self.leader_scale[name]
            actual[name] = raw_actual / self.leader_scale[name]
            floor, reference = (
                self.cfg["torque_floor_nm"][name],
                self.cfg["torque_reference_nm"][name],
            )
            # 扭矩绝对值映射为 0~1 负载；含重力、摩擦影响，不代表纯接触力。
            loads[name] = min(1.0, max(0.0, (torque - floor) / (reference - floor)))
        return target, actual, loads

    def _observe(self, target, actual, loads, now):
        cfg = self.cfg
        for name in NAMES:
            error = abs(target[name] - actual[name])
            if name in self.triggers:
                # 已触发关节需连续保持小角差才释放；中途超限就重新计时。
                if error <= cfg["release_deg"]:
                    since = self.release_since.setdefault(name, now)
                    if now - since >= cfg["release_stable_s"]:
                        self.triggers.remove(name)
                        self.release_since.pop(name, None)
                else:
                    self.release_since.pop(name, None)
                continue
            if error < cfg["lock_deg"]:
                self.windows.pop(name, None)
                self.over[name] = 0
                continue
            # 大角差只启动观察窗口；还需验证跟随不足和负载，避免正常追赶时误锁。
            window = self.windows.setdefault(name, (now, target[name], actual[name], error))
            if now - window[0] < cfg["window_s"]:
                continue
            self.windows[name] = (now, target[name], actual[name], error)
            ld = target[name] - window[1]
            fl = actual[name] - window[2]
            if abs(ld) >= cfg["tracking_min_deg"] and abs(ld) > 0:
                # 保留位移符号：反向运动得到负比值，不能误判为正常跟随。
                following = fl / ld >= cfg["tracking_ratio"]
            else:
                # 目标基本不动时，改看角差是否缩小，允许从臂继续追赶静止目标。
                following = window[3] - error >= cfg["convergence_deg"]
            if following or loads[name] < cfg["minimum_load_ratio"]:
                self.over[name] = 0
                continue
            # 连续达到设定数量的异常窗口，才把此关节加入堵转集合。
            self.over[name] += 1
            if self.over[name] >= cfg["confirm_windows"]:
                self.triggers.add(name)
                self.windows.pop(name, None)
                self.over[name] = 0

    def update(self, sent_action):
        if not self.active or self.closed:
            return
        now = self.clock()
        # 保持重发先于采样限频；两者都由官方循环驱动，循环阻塞时无法独立执行。
        if self.locked and now >= self.next_relock:
            self._lock(now, "holding")
        if now + 1e-9 < self.next_sample:
            return
        self.next_sample = now + 1 / self.cfg["sample_hz"]
        try:
            target, actual, loads = self._sample(sent_action)
        except Exception as exc:
            # 反馈无效则进入保持并清空确认计时，防止跨越数据中断累计判断。
            self.windows.clear()
            self.release_since.clear()
            self.over = dict.fromkeys(NAMES, 0)
            self.fault = True
            self.fault_clear_since = None
            if not self.locked:
                self._lock(now, str(exc))
            return
        finally:
            # 每次采样消耗本轮监测标记，下轮必须再次获得有效 Leader 监测结果。
            self._monitor_ok = False
        if self.fault:
            # 故障恢复要求所有关节角差连续达标，单次读取成功不会立即释放。
            if any(abs(target[name] - actual[name]) > self.cfg["release_deg"] for name in NAMES):
                self.fault_clear_since = None
                return
            if self.fault_clear_since is None:
                self.fault_clear_since = now
            if now - self.fault_clear_since < self.cfg["release_stable_s"]:
                return
            self.fault = False
            self.fault_clear_since = None
        self._observe(target, actual, loads, now)
        if self.triggers:
            if not self.locked:
                self._lock(now, "follower stalled")
            return
        if self.locked:
            self.locked = False
            # 清掉旧功率缓存，确保本轮真正从保持模式切回阻尼模式。
            self.sent.clear()
            logger.info("Leader feedback restored.")
        for name in NAMES:
            # 负载线性映射为阻尼功率；0x12 为阻尼模式，死区减少串口重复写入。
            power = round(
                self.cfg["power_min_mw"]
                + loads[name] * (self.cfg["power_max_mw"] - self.cfg["power_min_mw"])
            )
            previous = self.sent.get(name)
            if previous is None or abs(power - previous) >= self.cfg["power_deadband_mw"]:
                self._mode(name, 0x12, power)
                self.sent[name] = power

    def close(self):
        # 可重复调用；恢复监测方法并逐个释放本模块接管的关节。
        # 从臂断开和设备生命周期仍由官方遥操作负责。
        if self.closed:
            return
        self.closed = True
        if not self.active:
            return
        if self.bus.sync_monitor is self._wrapped_monitor:
            self.bus.sync_monitor = self._original_monitor
        for name in sorted(self.managed):
            try:
                self.bus.unlock(self.ids[name])
            except Exception:
                logger.exception("Could not release Leader joint %s during cleanup.", name)
