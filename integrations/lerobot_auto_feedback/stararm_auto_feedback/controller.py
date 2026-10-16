from __future__ import annotations

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
    cfg = {
        "enabled": True,
        "torque_reference_nm": dict(zip(NAMES, (5.0, 5.0, 5.0, 1.0, 1.0, 1.0, 1.0))),
        "torque_floor_nm": dict(zip(NAMES, (0.1, 0.1, 0.1, 0.05, 0.05, 0.05, 0.05))),
        "power_min_mw": 1,
        "power_max_mw": 1200,
        "power_deadband_mw": 20,
        "sample_hz": 30.0,
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
        self._original_monitor = self.bus.sync_monitor

        def monitor(*args, **kwargs):
            self._monitor_ok = False
            result = self._original_monitor(*args, **kwargs)
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
        self.managed.add(name)
        self.bus.set_stop_mode(self.ids[name], mode=mode, power=int(power))

    def _lock(self, now, reason):
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
        target, actual, loads = {}, {}, {}
        for name in NAMES:
            state = self.robot.motors[name].get_state()
            if state is None or state.status_code != 1:
                raise ValueError("Follower feedback unavailable or motor not enabled")
            raw_target = float(sent_action[f"{name}.pos"])
            raw_actual = math.degrees(state.pos)
            torque = abs(state.torq)
            if not all(math.isfinite(v) for v in (raw_target, raw_actual, torque)):
                raise ValueError("Non-finite follower feedback")
            # Built-in LeRobot returns public coordinates; Seeed's external plugin returns motor coordinates.
            if self.builtin:
                raw_actual /= self.follower_scale[name]
            else:
                raw_target /= self.follower_scale[name]
                raw_actual /= self.follower_scale[name]
            target[name] = raw_target / self.leader_scale[name]
            actual[name] = raw_actual / self.leader_scale[name]
            floor, reference = (
                self.cfg["torque_floor_nm"][name],
                self.cfg["torque_reference_nm"][name],
            )
            loads[name] = min(1.0, max(0.0, (torque - floor) / (reference - floor)))
        return target, actual, loads

    def _observe(self, target, actual, loads, now):
        cfg = self.cfg
        for name in NAMES:
            error = abs(target[name] - actual[name])
            if name in self.triggers:
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
            window = self.windows.setdefault(name, (now, target[name], actual[name], error))
            if now - window[0] < cfg["window_s"]:
                continue
            self.windows[name] = (now, target[name], actual[name], error)
            ld = target[name] - window[1]
            fl = actual[name] - window[2]
            if abs(ld) >= cfg["tracking_min_deg"] and abs(ld) > 0:
                following = fl / ld >= cfg["tracking_ratio"]
            else:
                following = window[3] - error >= cfg["convergence_deg"]
            if following or loads[name] < cfg["minimum_load_ratio"]:
                self.over[name] = 0
                continue
            self.over[name] += 1
            if self.over[name] >= cfg["confirm_windows"]:
                self.triggers.add(name)
                self.windows.pop(name, None)
                self.over[name] = 0

    def update(self, sent_action):
        if not self.active or self.closed:
            return
        now = self.clock()
        if self.locked and now >= self.next_relock:
            self._lock(now, "holding")
        if now + 1e-9 < self.next_sample:
            return
        self.next_sample = now + 1 / self.cfg["sample_hz"]
        try:
            target, actual, loads = self._sample(sent_action)
        except Exception as exc:
            self.windows.clear()
            self.release_since.clear()
            self.over = dict.fromkeys(NAMES, 0)
            self.fault = True
            self.fault_clear_since = None
            if not self.locked:
                self._lock(now, str(exc))
            return
        finally:
            self._monitor_ok = False
        if self.fault:
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
            self.sent.clear()
            logger.info("Leader feedback restored.")
        for name in NAMES:
            power = round(
                self.cfg["power_min_mw"]
                + loads[name] * (self.cfg["power_max_mw"] - self.cfg["power_min_mw"])
            )
            previous = self.sent.get(name)
            if previous is None or abs(power - previous) >= self.cfg["power_deadband_mw"]:
                self._mode(name, 0x12, power)
                self.sent[name] = power

    def close(self):
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
