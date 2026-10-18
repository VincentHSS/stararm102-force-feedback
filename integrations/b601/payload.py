"""整臂额外负载估计：固定空载模型、竖直载荷拟合与有界阻尼输出。"""

from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path

from gravity import GravityModel, dot

ALGORITHM = "b601-quasistatic-payload-v1"


def signature(cfg):
    # 配对或零位改变后要求重新验证，不把旧标定悄悄用于新的角度基准。
    rows = [
        [j.motor_id, j.feedback_id, j.direction, j.follower_home_rad, j.lower_deg, j.upper_deg]
        for j in cfg.joints
    ]
    return hashlib.sha256(json.dumps(rows, separators=(",", ":")).encode()).hexdigest()


def finite_vector(values, size):
    if (
        not isinstance(values, list)
        or len(values) != size
        or any(
            isinstance(x, bool) or not isinstance(x, (int, float)) or not math.isfinite(x)
            for x in values
        )
    ):
        raise ValueError(f"Expected {size} finite calibration values")
    return values


def friction_features(gravity, velocity):
    return [1.0, gravity, velocity, math.tanh(velocity / 0.03)]


class MotionSignals:
    def __init__(self):
        self.last = self.since = None
        self.velocity = [0.0] * 6
        self.acceleration = [0.0] * 6

    def update(self, now, raw_velocity):
        if self.last is None or not 0 < now - self.last <= 0.1:
            self.last = self.since = now
            self.velocity = list(raw_velocity)
            self.acceleration = [0.0] * 6
            return False
        dt, self.last = now - self.last, now
        previous = list(self.velocity)
        va, aa = dt / (0.08 + dt), dt / (0.12 + dt)
        for i, raw in enumerate(raw_velocity):
            self.velocity[i] += va * (raw - self.velocity[i])
            self.acceleration[i] += aa * (
                (self.velocity[i] - previous[i]) / dt - self.acceleration[i]
            )
        return now - self.since >= 0.3


def vertical_fit(model_values, residual, sigma, active_axes):
    """联合拟合竖直负载，投影掉未知物体重心带来的水平力矩。"""
    h = [model_values["unit_mass"][i] / sigma[i] for i in active_axes]
    r = [residual[i] / sigma[i] for i in active_axes]
    basis = []
    for name in ("moment_x", "moment_y"):
        original = [model_values[name][i] / sigma[i] for i in active_axes]
        candidate = list(original)
        for b in basis:
            projection = dot(candidate, b)
            candidate = [x - projection * y for x, y in zip(candidate, b)]
        length = math.sqrt(dot(candidate, candidate))
        if length > max(1e-8, 1e-6 * math.sqrt(dot(original, original))):
            basis.append([x / length for x in candidate])
    for b in basis:
        hp, rp = dot(h, b), dot(r, b)
        h = [x - hp * y for x, y in zip(h, b)]
        r = [x - rp * y for x, y in zip(r, b)]
    information = dot(h, h)
    degrees = len(active_axes) - len(basis) - 1
    if information < 1e-10 or degrees < 1:
        return None
    mass = dot(h, r) / information
    score = math.sqrt(sum((x - mass * y) ** 2 for x, y in zip(r, h)) / degrees)
    return mass, score, 1 / math.sqrt(information)


class Calibration:
    def __init__(self, data, model):
        self.data, self.model = data, model
        self.coefficients = data["coefficients"]
        self.sigma = data["sigma_nm"]
        self.axes = data["active_axes"]
        self.deadband = data["mass_deadband_kg"]

    @classmethod
    def load(cls, path, cfg):
        path = Path(path).resolve()
        data = json.loads(path.read_text(encoding="utf-8-sig"))
        if data.get("algorithm") != ALGORITHM or data.get("validated") is not True:
            raise ValueError("Use a validated unloaded calibration from payload_tools.py")
        if data.get("config_signature") != signature(cfg):
            raise ValueError("Pairing/mapping changed; repeat unloaded calibration and validation")
        directions = finite_vector(data["model_directions"], 6)
        offsets = finite_vector(data["model_offsets_rad"], 6)
        gravity = finite_vector(data["gravity_m_s2"], 3)
        model = GravityModel(path.parent / data["model_path"], directions, offsets, gravity)
        if model.sha256 != data["model_sha256"]:
            raise ValueError("URDF changed; repeat unloaded calibration")
        if not isinstance(data["coefficients"], list) or len(data["coefficients"]) != 6:
            raise ValueError("Expected six torque models")
        for row in data["coefficients"]:
            finite_vector(row, 4)
            if not 0.2 <= abs(row[1]) <= 3:
                raise ValueError("Invalid measured torque polarity/scale")
        finite_vector(data["sigma_nm"], 6)
        axes = data["active_axes"]
        if (
            not isinstance(axes, list)
            or len(set(axes)) != len(axes)
            or len(axes) < 3
            or any(type(i) is not int or not 0 <= i < 6 for i in axes)
            or any(x < 0.01 for x in data["sigma_nm"])
        ):
            raise ValueError("Invalid torque noise/active axes")
        if not isinstance(data["bounds_rad"], list) or len(data["bounds_rad"]) != 6:
            raise ValueError("Expected six calibrated position ranges")
        for pair in data["bounds_rad"]:
            finite_vector(pair, 2)
            if pair[0] >= pair[1]:
                raise ValueError("Invalid calibrated position range")
        report = data["validation"]
        for key in ("mass_rmse_kg", "mass_abs_p99_kg", "valid_fraction"):
            value = report[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError("Invalid unloaded validation report")
        deadband = data["mass_deadband_kg"]
        if (
            not isinstance(deadband, (int, float))
            or not math.isfinite(deadband)
            or not 0.05 <= deadband <= 0.2
        ):
            raise ValueError("Calibration uncertainty too large for payload feedback")
        if (
            not 0 <= report["mass_rmse_kg"] <= 0.1
            or not 0 <= report["mass_abs_p99_kg"] <= deadband
            or not 0.7 <= report["valid_fraction"] <= 1
        ):
            raise ValueError("Unloaded validation did not pass")
        if report.get("samples", 0) < 80 or data.get("training_samples", 0) < 120:
            raise ValueError("Insufficient independent unloaded samples")
        return cls(data, model)

    def residual(self, positions, velocity, measured):
        values = self.model.evaluate(positions)
        result = []
        for i, row in enumerate(self.coefficients):
            prediction = dot(row, friction_features(values["gravity"][i], velocity[i]))
            result.append((measured[i] - prediction) / row[1])
        return values, result


class PayloadObserver:
    def __init__(self, cfg, calibration):
        self.cfg, self.calibration = cfg, calibration
        self.signals = MotionSignals()
        self.mass_kg = 0.0
        self.raw_mass_kg = math.nan
        self.score = self.sigma_kg = math.nan
        self.residual_nm = [0.0] * 6
        self.loads = [0.0] * 7
        self.status = "warming up"
        self.last = self.last_evidence = self.positive_since = None

    def invalidate(self, now, reason):
        self.signals = MotionSignals()
        self.mass_kg = 0.0
        self.loads = [0.0] * 7
        self.raw_mass_kg = self.score = self.sigma_kg = math.nan
        self.last_evidence = self.positive_since = None
        self.status, self.last = reason, now

    def gap(self, now):
        if self.last_evidence is None or now - self.last_evidence >= min(
            self.cfg.link_timeout_s, self.cfg.payload_hold_s
        ):
            self.invalidate(now, "feedback gap")
        else:
            self.status = "short feedback gap: estimate held"
            if self.mass_kg <= self.calibration.deadband:
                self.positive_since = None

    def update(self, positions, velocities, torques, now):
        if any(
            len(v) < 6 or not all(math.isfinite(x) for x in v[:6])
            for v in (positions, velocities, torques)
        ):
            self.invalidate(now, "invalid input")
            return
        cfg, calibration = self.cfg, self.calibration
        dt = 1 / cfg.loop_hz if self.last is None else min(max(now - self.last, 0), 0.1)
        self.last = now
        ready = self.signals.update(now, velocities[:6])
        if any(
            not lo - 0.0873 <= q <= hi + 0.0873
            for q, (lo, hi) in zip(positions, calibration.data["bounds_rad"])
        ):
            self.invalidate(now, "outside calibration")
            return
        values, self.residual_nm = calibration.residual(
            positions[:6], self.signals.velocity, torques[:6]
        )
        steady = (
            ready
            and max(abs(v) for v in velocities[:6]) <= cfg.payload_max_speed_rad_s
            and max(abs(a) for a in self.signals.acceleration) <= cfg.payload_max_accel_rad_s2
        )
        target = None
        if steady:
            fitted = vertical_fit(values, self.residual_nm, calibration.sigma, calibration.axes)
            if fitted is not None:
                self.raw_mass_kg, self.score, self.sigma_kg = fitted
            else:
                self.raw_mass_kg = self.score = self.sigma_kg = math.nan
            if fitted is not None and self.score <= 3 and self.sigma_kg <= 0.2:
                target = max(0.0, self.raw_mass_kg)
                self.last_evidence, self.status = now, "payload estimate"
            else:
                target, self.status = 0.0, "unexplained torque"
                self.positive_since = None
        else:
            self.status = "motion: estimate held"
        if target is None:
            if self.last_evidence is None or now - self.last_evidence > cfg.payload_hold_s:
                target, self.status = 0.0, "motion: feedback faded"
            else:
                target = self.mass_kg
        # 不在线学习载荷；持续拿着物体时，固定空载模型不会将物体重量吸收掉。
        if target > calibration.deadband:
            if self.positive_since is None:
                self.positive_since = now
            if now - self.positive_since < 0.2:
                target = 0.0
        else:
            self.positive_since = None
        tau = cfg.payload_smoothing_s if target >= self.mass_kg else 0.12
        self.mass_kg += dt / (tau + dt) * (target - self.mass_kg)
        ratio = min(
            1.0,
            max(
                0.0,
                (self.mass_kg - calibration.deadband)
                / (cfg.payload_reference_kg - calibration.deadband),
            ),
        )
        self.loads = [ratio * weight for weight in cfg.payload_haptic_weights]
