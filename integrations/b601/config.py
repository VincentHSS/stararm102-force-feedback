"""B601 独立 Python 版配置；映射与配对姿态不依赖 LeRobot。"""

from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass, field, fields
from pathlib import Path

NAMES = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_yaw",
    "wrist_roll",
    "gripper",
)


@dataclass
class Joint:
    leader_id: int
    motor_id: int
    feedback_id: int
    direction: float
    lower_deg: float
    upper_deg: float
    speed_deg_s: float = 50.0
    leader_home_deg: float = 0.0
    follower_home_rad: float = 0.0
    torque_reference_nm: float = 5.0
    torque_deadband_nm: float = 0.2


def default_joints():
    directions = (-1, -1, 1, 1, 1, -1, -6)
    limits = ((-145, 145), (-170, 0), (-200, 0), (-80, 90), (-90, 90), (-130, 130), (-270, 0))
    return [
        Joint(
            i,
            i + 1,
            i + 17,
            directions[i],
            *limits[i],
            speed_deg_s=100 if i == 6 else 50,
            torque_reference_nm=5 if i < 3 else 1,
            torque_deadband_nm=0.2 if i < 3 else 0.1,
        )
        for i in range(7)
    ]


@dataclass
class Config:
    schema_version: int = 2
    leader_port: str = "/dev/ttyUSB0"
    follower_port: str = "/dev/ttyACM0"
    adapter: str = "damiao"
    follower_baud: int = 921600
    button_id: int | None = None
    calibration_verified: bool = False
    loop_hz: float = 100.0
    read_timeout_s: float = 0.015
    link_timeout_s: float = 0.25
    startup_error_deg: float = 5.0
    filter_cutoff_hz: float = 5.0
    filter_beta: float = 0.15
    input_deadband_deg: float = 0.1
    input_max_speed_deg_s: float = 800.0
    input_jump_margin_deg: float = 10.0
    max_step_deg: float = 2.0
    acceleration_deg_s2: float = 300.0
    baseline_s: float = 1.0
    baseline_adapt_s: float = 5.0
    torque_filter_s: float = 0.1
    power_min_mw: int = 1
    power_max_mw: int = 1000
    power_deadband_mw: int = 25
    hold_mw: int = 6000
    relock_s: float = 1.0
    lock_deg: float = 20.0
    release_deg: float = 8.0
    window_s: float = 0.5
    confirm_windows: int = 2
    minimum_load_ratio: float = 0.35
    stalled_motion_deg: float = 0.5
    convergence_deg: float = 1.0
    release_stable_s: float = 0.4
    gripper_force_ratio: float = 0.1
    motor_temperature_c: float = 80.0
    joints: list[Joint] = field(default_factory=default_joints)

    def validate(self):
        if self.schema_version != 2 or self.adapter not in ("damiao", "socketcan"):
            raise ValueError("Use schema_version=2 and adapter damiao/socketcan")
        if not isinstance(self.calibration_verified, bool):
            raise ValueError("calibration_verified must be boolean")
        if not all(
            isinstance(p, str) and p.strip() for p in (self.leader_port, self.follower_port)
        ):
            raise ValueError("Both ports are required")
        if self.leader_port.casefold() == self.follower_port.casefold():
            raise ValueError("Leader and follower must use different ports")
        for key, value in vars(self).items():
            if key in ("joints", "calibration_verified", "button_id") or isinstance(value, str):
                continue
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
                or value < 0
            ):
                raise ValueError(f"Invalid setting: {key}")
        for key in (
            "follower_baud",
            "power_min_mw",
            "power_max_mw",
            "power_deadband_mw",
            "hold_mw",
            "confirm_windows",
        ):
            if type(getattr(self, key)) is not int:
                raise ValueError(f"{key} must be an integer")
        if self.button_id is not None and (
            type(self.button_id) is not int or not 7 <= self.button_id <= 253
        ):
            raise ValueError("button_id must be null or an integer within 7..253")
        if (
            not 1 <= self.loop_hz <= 200
            or not 0.001 <= self.read_timeout_s < self.link_timeout_s <= 2
        ):
            raise ValueError("Invalid loop frequency or link timeout")
        if not 9600 <= self.follower_baud <= 3000000 or not 1 <= self.motor_temperature_c <= 120:
            raise ValueError("Invalid serial baud or temperature limit")
        if (
            not 1 <= self.power_min_mw <= self.power_max_mw <= 65535
            or not 1 <= self.hold_mw <= 65535
        ):
            raise ValueError("Power must be within 1..65535 mW")
        if not 0 <= self.release_deg < self.lock_deg or not 0 <= self.minimum_load_ratio <= 1:
            raise ValueError("Invalid stall thresholds")
        if (
            min(
                self.filter_cutoff_hz,
                self.max_step_deg,
                self.acceleration_deg_s2,
                self.input_max_speed_deg_s,
                self.torque_filter_s,
                self.baseline_adapt_s,
                self.release_stable_s,
                self.relock_s,
            )
            <= 0
        ):
            raise ValueError("Filter, motion and confirmation settings must be positive")
        if self.window_s < 0.1 or self.confirm_windows < 1 or self.baseline_s < 0.2:
            raise ValueError("Invalid observation/baseline interval")
        if not 0 < self.gripper_force_ratio <= 1 or not 1 <= self.startup_error_deg <= 20:
            raise ValueError("Invalid gripper force ratio or startup error")
        if len(self.joints) != 7 or [j.leader_id for j in self.joints] != list(range(7)):
            raise ValueError("Joint rows must be ordered by Leader ID 0..6")
        for attr in ("motor_id", "feedback_id"):
            values = [getattr(j, attr) for j in self.joints]
            if len(set(values)) != 7 or any(
                type(v) is not int or not 1 <= v <= (15 if attr == "motor_id" else 0x7FF)
                for v in values
            ):
                raise ValueError(f"Invalid or duplicate {attr}")
        if {j.motor_id for j in self.joints} & {j.feedback_id for j in self.joints}:
            raise ValueError("Command and feedback CAN IDs overlap")
        for j in self.joints:
            if any(
                isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                for v in vars(j).values()
            ):
                raise ValueError("Joint values must be finite numbers")
            if (
                not 0.01 <= abs(j.direction) <= 20
                or not j.lower_deg < j.upper_deg
                or not j.lower_deg - 0.1 <= math.degrees(j.follower_home_rad) <= j.upper_deg + 0.1
            ):
                raise ValueError("Invalid direction, limits or home pose")
            if (
                not 0 < j.speed_deg_s <= 360
                or j.torque_deadband_nm < 0
                or j.torque_reference_nm <= j.torque_deadband_nm
            ):
                raise ValueError("Invalid joint speed or torque scale")
        return self

    @classmethod
    def load(cls, path):
        values = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        if set(values) - {f.name for f in fields(cls)}:
            raise ValueError("Unknown configuration keys")
        if "joints" in values:
            values["joints"] = [Joint(**j) for j in values["joints"]]
        return cls(**values).validate()

    def save(self, path):
        self.validate()
        Path(path).write_text(
            json.dumps(asdict(self), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
