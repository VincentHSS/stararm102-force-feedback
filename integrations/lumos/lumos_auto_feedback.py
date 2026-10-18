"""Lumos 负载反馈核心：自动阻尼、堵转保持、恢复释放及 HD 按钮仲裁。"""

from __future__ import annotations

import logging
import math
import time
from dataclasses import dataclass

logger = logging.getLogger(__name__)


@dataclass(frozen=True)
class FeedbackConfig:
    # 按 Lumos J1..J6、夹爪排列；用于反馈归一化，不是额定扭矩或保护阈值。
    torque_reference_nm: tuple = (5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0)
    torque_floor_nm: tuple = (0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05)
    # Leader 协议使用 mW；0 有特殊含义，因此阻尼最小值从 1 开始。
    power_min_mw: int = 1
    power_max_mw: int = 1200
    power_deadband_mw: int = 20
    hold_mw: int = 6000
    sample_hz: float = 30.0
    lock_deg: float = 20.0
    release_deg: float = 8.0
    window_s: float = 0.4
    tracking_ratio: float = 0.35
    tracking_min_deg: float = 1.5
    convergence_deg: float = 1.0
    minimum_load_ratio: float = 0.3
    confirm_windows: int = 1
    release_stable_s: float = 0.3
    relock_s: float = 1.0
    gripper_max_age_ms: float = 250.0

    def __post_init__(self):
        for name, value in vars(self).items():
            values = value if name.startswith("torque_") else (value,)
            if name.startswith("torque_") and len(values) != 7:
                raise ValueError(f"{name}: expected seven values")
            if any(
                isinstance(v, bool)
                or not isinstance(v, (int, float))
                or not math.isfinite(v)
                or v < 0
                for v in values
            ):
                raise ValueError(f"{name}: expected finite nonnegative numbers")
        if any(floor >= ref for floor, ref in zip(self.torque_floor_nm, self.torque_reference_nm)):
            raise ValueError("Torque floor must be below reference")
        if not 1 <= self.power_min_mw <= self.power_max_mw <= 65535:
            raise ValueError("Damping power must be within 1..65535 mW")
        if not 1 <= self.hold_mw <= 65535 or not 0 < self.sample_hz <= 100:
            raise ValueError("Invalid holding power or sample rate")
        if not self.release_deg < self.lock_deg or self.window_s < 0.05:
            raise ValueError("Invalid angle thresholds or observation window")
        if min(self.release_stable_s, self.relock_s) < 0.05 or self.gripper_max_age_ms <= 0:
            raise ValueError("Invalid confirmation or freshness interval")
        if not 0 <= self.tracking_ratio <= 1 or not 0 <= self.minimum_load_ratio <= 1:
            raise ValueError("Ratios must be within 0..1")
        if self.confirm_windows < 1 or self.confirm_windows != int(self.confirm_windows):
            raise ValueError("confirm_windows must be a positive integer")


class LumosAutoFeedback:
    """复用已连接的 UART/CAN 设备；本模块只控制 Leader，不下发 Lumos 运动。"""

    def __init__(
        self,
        leader,
        lumos,
        joint_map,
        *,
        gain=1.0,
        gripper_span_deg=None,
        config=None,
        clock=time.monotonic,
    ):
        self.cfg = FeedbackConfig() if config is None else config
        self.leader, self.lumos, self.clock = leader, lumos, clock
        self.ids = tuple(joint_map)
        if len(self.ids) != 6 or set(self.ids) != set(range(6)):
            raise ValueError("Expected a permutation of Leader IDs 0..5")
        if not math.isfinite(gain) or not 0 < gain <= 1:
            raise ValueError("gain must be within (0, 1]")
        self.gain = gain
        self.gripper_span = None if gripper_span_deg is None else abs(gripper_span_deg)
        if self.gripper_span is not None:
            if not math.isfinite(self.gripper_span) or self.gripper_span < 2:
                raise ValueError("Invalid calibrated gripper range")
            self.ids += (6,)
        required = ("get_joint_positions", "get_joint_torques")
        if self.gripper_span is not None:
            required += ("get_gripper_state",)
        if any(not callable(getattr(lumos, name, None)) for name in required):
            raise RuntimeError(
                "Lumos SDK 缺少反馈接口，请更新 startouch_sdk；排错可加 --feedback-disable"
            )
        self.closed = False
        self.fault = False
        self.fault_since = None
        self.triggers = set()
        self.windows, self.over, self.release_since = {}, {}, {}
        self.sent, self.managed = {}, set()
        self.loads = [0.0] * len(self.ids)
        self.next_sample = 0.0
        self.state = None

    @staticmethod
    def _vector(values, length):
        values = [float(v) for v in values]
        if len(values) != length or not all(math.isfinite(v) for v in values):
            raise ValueError("Lumos 返回无效反馈维度或数值")
        return values

    def _sample(self, sent_target, gripper_target):
        target = self._vector(sent_target, 6)
        actual = self._vector(self.lumos.get_joint_positions(), 6)
        torque = self._vector(self.lumos.get_joint_torques(), 6)
        # 关节位置从 rad 转为 Leader 等效角度。原点和方向对角差绝对值、跟随比均抵消。
        # 比较上一帧已发送的限速目标，避免把滤波或限速延迟误认为堵转。
        target = [math.degrees(q) / self.gain for q in target]
        actual = [math.degrees(q) / self.gain for q in actual]
        if self.gripper_span is not None:
            state = self.lumos.get_gripper_state()
            age = float(state.feedback_age_ms)
            if (
                not state.feedback_valid
                or not state.enabled
                or state.error_code != 0
                or not math.isfinite(age)
                or not 0 <= age <= self.cfg.gripper_max_age_ms
            ):
                raise ValueError("Lumos 夹爪反馈失效、过期或电机异常")
            position = float(state.position)
            command = position if gripper_target is None else float(gripper_target)
            if not all(math.isfinite(v) and 0 <= v <= 1 for v in (position, command)):
                raise ValueError("Lumos 夹爪开度无效")
            # 夹爪开度是 0..1，乘校准行程后才能与角度阈值比较；反转映射已在主循环完成。
            target.append(command * self.gripper_span)
            actual.append(position * self.gripper_span)
            torque.append(float(state.effort_nm))
        torque = self._vector(torque, len(self.ids))
        # 力矩含重力与摩擦；这里生成阻尼强弱，不将它解释为纯接触力。
        loads = [
            min(1.0, max(0.0, (abs(t) - floor) / (ref - floor)))
            for t, floor, ref in zip(torque, self.cfg.torque_floor_nm, self.cfg.torque_reference_nm)
        ]
        return target, actual, loads

    def _observe(self, target, actual, loads, now):
        cfg = self.cfg
        for i, (goal, position, load) in enumerate(zip(target, actual, loads)):
            error = abs(goal - position)
            if i in self.triggers:
                # 小于释放阈值且持续稳定才解除此关节；任何触发关节未恢复都继续整体保持。
                if error <= cfg.release_deg:
                    since = self.release_since.setdefault(i, now)
                    if now - since >= cfg.release_stable_s:
                        self.triggers.remove(i)
                        self.release_since.pop(i, None)
                else:
                    self.release_since.pop(i, None)
                continue
            if error < cfg.lock_deg:
                self.windows.pop(i, None)
                self.over[i] = 0
                continue
            window = self.windows.setdefault(i, (now, goal, position, error))
            if now - window[0] < cfg.window_s:
                continue
            self.windows[i] = (now, goal, position, error)
            movement = goal - window[1]
            # 有符号位移比识别反向滑动；目标静止时，改用角差收敛量识别正常追赶。
            following = (
                (position - window[2]) / movement >= cfg.tracking_ratio
                if abs(movement) >= cfg.tracking_min_deg and movement != 0
                else window[3] - error >= cfg.convergence_deg
            )
            if following or load < cfg.minimum_load_ratio:
                self.over[i] = 0
                continue
            self.over[i] = self.over.get(i, 0) + 1
            if self.over[i] >= cfg.confirm_windows:
                self.triggers.add(i)
                self.windows.pop(i, None)
                self.over[i] = 0

    def _apply(self, now, manual_hold):
        automatic_hold = self.fault or bool(self.triggers)
        state = (
            "反馈故障保持"
            if self.fault
            else ("堵转保持" if self.triggers else ("HD悬停" if manual_hold else "自动阻尼"))
        )
        if state != self.state:
            logger.info("Leader：%s", state)
            self.state = state
        for i, servo_id in enumerate(self.ids):
            # HD 只保持六个关节；自动堵转或反馈故障保持所有参与反馈的关节及夹爪。
            # 所有模式从这里统一下发，HD 释放不能覆盖仍有效的自动保持。
            hold = automatic_hold or (manual_hold and i < 6)
            mode = 0x11 if hold else 0x12
            power = (
                self.cfg.hold_mw
                if hold
                else round(
                    self.cfg.power_min_mw
                    + self.loads[i] * (self.cfg.power_max_mw - self.cfg.power_min_mw)
                )
            )
            previous = self.sent.get(servo_id)
            if (
                previous is None
                or previous[0] != mode
                or (hold and now - previous[2] >= self.cfg.relock_s)
                or (not hold and abs(power - previous[1]) >= self.cfg.power_deadband_mw)
            ):
                self.managed.add(servo_id)
                self.leader.stop_on_control_mode(servo_id, mode, int(power))
                self.sent[servo_id] = (mode, power, now)

    def update(self, sent_target, *, gripper_target=None, manual_hold=False):
        """主循环发送下一帧前调用；False 表示反馈异常，暂停发送新的 Lumos 目标。"""
        if self.closed:
            return False
        now = self.clock()
        if now >= self.next_sample:
            self.next_sample = now + 1.0 / self.cfg.sample_hz
            try:
                target, actual, loads = self._sample(sent_target, gripper_target)
            except Exception as exc:
                if not self.fault:
                    logger.warning("Lumos 反馈异常，Leader 保持：%s", exc)
                self.fault = True
                self.fault_since = None
                self.windows.clear()
                self.over.clear()
                self.release_since.clear()
            else:
                self.loads = loads
                if self.fault:
                    # 恢复要求所有角差连续达标；避免一次正常读取立即释放。
                    if any(abs(t - a) > self.cfg.release_deg for t, a in zip(target, actual)):
                        self.fault_since = None
                    else:
                        if self.fault_since is None:
                            self.fault_since = now
                        if now - self.fault_since >= self.cfg.release_stable_s:
                            self.fault = False
                            self.fault_since = None
                if not self.fault:
                    self._observe(target, actual, loads, now)
        # 保持重发和 HD 仲裁不受采样限频影响，但依然依赖主循环持续运行。
        self._apply(now, manual_hold)
        return not self.fault

    def close(self):
        # 幂等清理；单个关节释放失败不影响后续关节，串口由主程序最后关闭。
        if self.closed:
            return
        self.closed = True
        for servo_id in sorted(self.managed):
            try:
                self.leader.stop_on_control_mode(servo_id, 0x10, 0)
            except Exception:
                logger.exception("Leader 关节 %s 释放失败", servo_id)
