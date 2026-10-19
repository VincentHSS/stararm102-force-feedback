"""扣除自重与摩擦后，按多轴额外负载证据输出阻尼；不输出公斤数。"""

from __future__ import annotations

import math
import hashlib
import json

from gravity import dot
from payload import (
    Calibration,
    MotionSignals,
    SettledPose,
    SETTLED_GATE,
    finite_vector,
    interior_pose,
)

ALGORITHM = "b601-extra-load-v1"
THRESHOLD = 3.0
MIN_COHERENCE = 0.5
MIN_INFORMATION = 1.0
CONFIRMATION_GAP_S = 0.05
WORKSPACE_RADIUS_RAD = math.radians(5)


def model_parameters_hash(data):
    keys = (
        "algorithm",
        "config_signature",
        "model_sha256",
        "model_directions",
        "model_offsets_rad",
        "gravity_m_s2",
        "coefficients",
        "sigma_nm",
        "active_axes",
        "evidence_threshold",
        "minimum_coherence",
    )
    raw = json.dumps(
        {k: data[k] for k in keys}, sort_keys=True, separators=(",", ":"), allow_nan=False
    )
    return hashlib.sha256(raw.encode()).hexdigest()


def workspace_blocked_joints(data, positions):
    # 新区域仅覆盖已验证姿态附近，不能将各轴极值拼成未验证的整个大方框。
    regions = [data["bounds_rad"]]
    for entry in data.get("workspace_validations", []):
        regions.extend([[q, q] for q in pose] for pose in entry["poses_rad"])
    blocked = []
    for bounds in regions:
        outside = [
            i + 1
            for i, (q, (lo, hi)) in enumerate(zip(positions[:6], bounds))
            if not lo - WORKSPACE_RADIUS_RAD <= q <= hi + WORKSPACE_RADIUS_RAD
        ]
        if not outside:
            return []
        blocked.append(outside)
    return min(blocked, key=len)


def load_evidence(values, residual, noise, axes):
    # 各轴先除以训练空载噪声，避免噪声大的肩肘轴支配判断。
    h = [values["unit_mass"][i] / noise[i] for i in axes]
    y = [residual[i] / noise[i] for i in axes]
    information = dot(h, h)
    if information < MIN_INFORMATION:
        return None
    projection = dot(h, y)
    score = projection / math.sqrt(information)
    coherence = projection / max(1e-12, math.sqrt(information * dot(y, y)))
    return score, coherence


def is_extra_load(score, coherence):
    return score > THRESHOLD and coherence > MIN_COHERENCE


class ExtraLoadCalibration(Calibration):
    algorithm = ALGORITHM

    @staticmethod
    def validate_report(data):
        report = data["validation"]
        if (
            data.get("validation_scope") != "independent_unloaded_only"
            or data.get("noise_source") != "training_only_p95_absolute_residual"
            or data.get("evidence_threshold") != THRESHOLD
            or data.get("minimum_coherence") != MIN_COHERENCE
            or any(value < 0.03 for value in data["sigma_nm"])
        ):
            raise ValueError("Invalid extra-load calibration method")
        for key in ("valid_fraction", "false_activation_fraction", "score_p99", "score_max"):
            value = report[key]
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not math.isfinite(value)
            ):
                raise ValueError("Invalid extra-load validation report")
        if not 0.7 <= report["valid_fraction"] <= 1 or report["false_activation_fraction"] != 0:
            raise ValueError("Independent unloaded extra-load validation did not pass")
        entries = data.get("workspace_validations", [])
        if not isinstance(entries, list) or len(entries) > 32:
            raise ValueError("Invalid workspace validations")
        for entry in entries:
            poses, check = entry["poses_rad"], entry["validation"]
            finite_vector(
                [
                    check[k]
                    for k in (
                        "valid_fraction",
                        "false_activation_fraction",
                        "score_p99",
                        "score_max",
                    )
                ],
                4,
            )
            if (
                entry.get("model_parameters_sha256") != model_parameters_hash(data)
                or entry.get("confirmed_unloaded") is not True
                or entry.get("validated") is not True
                or entry.get("evidence_gate") != SETTLED_GATE
                or entry.get("radius_rad") != WORKSPACE_RADIUS_RAD
                or not isinstance(entry.get("source_sha256"), str)
                or len(entry["source_sha256"]) != 64
                or any(c not in "0123456789abcdef" for c in entry["source_sha256"])
                or not isinstance(poses, list)
                or not 1 <= len(poses) <= 10000
                or type(check["observable_samples"]) is not int
                or check["observable_samples"] < 80
                or type(check["samples"]) is not int
                or check["samples"] < check["observable_samples"]
                or not math.isclose(
                    check["valid_fraction"], check["observable_samples"] / check["samples"]
                )
                or not 0.7 <= check["valid_fraction"] <= 1
                or check["false_activation_fraction"] != 0
            ):
                raise ValueError("Unloaded workspace validation did not pass for this frozen model")
            for pose in poses:
                finite_vector(pose, 6)


class ExtraLoadObserver:
    def __init__(self, cfg, calibration):
        self.cfg, self.calibration = cfg, calibration
        self.signals = MotionSignals()
        self.settled = SettledPose()
        self.loads = [0.0] * 7
        self.residual_nm = [0.0] * 6
        self.score = self.coherence = math.nan
        self.strength = 0.0
        self.status = "warming up"
        self.last = self.last_evidence = self.last_positive = None
        self.confirmation_s, self.previous_positive = 0.0, False
        self.blocked_joints = []
        # 保持旧诊断列兼容；额外负载模式从不产生质量读数。
        self.mass_kg = self.raw_mass_kg = self.sigma_kg = math.nan

    def invalidate(self, now, reason):
        self.signals = MotionSignals()
        self.settled = SettledPose()
        self.loads, self.strength = [0.0] * 7, 0.0
        self.score = self.coherence = math.nan
        self.last_evidence = self.last_positive = None
        self.confirmation_s, self.previous_positive = 0.0, False
        self.blocked_joints = []
        self.last, self.status = now, reason

    def pause_confirmation(self, now):
        # 仅保留短暂中断前已观察到的证据时间，缺失或动态帧不计入 0.2 秒。
        self.previous_positive = False
        if self.last_positive is None or now - self.last_positive > CONFIRMATION_GAP_S:
            self.confirmation_s = 0.0

    def gap(self, now):
        self.pause_confirmation(now)
        # 通信新鲜度与准静态证据是两个时钟；运动中缺一帧不能按旧静态证据判超时。
        timed_out = self.last is None or now - self.last >= self.cfg.link_timeout_s
        expired = (
            self.last_evidence is not None and now - self.last_evidence > self.cfg.payload_hold_s
        )
        if timed_out or expired:
            self.invalidate(now, "feedback gap")
        else:
            self.status = "short feedback gap: feedback held"

    def update(self, positions, velocities, torques, now):
        if not math.isfinite(now) or any(
            len(v) < 6 or not all(math.isfinite(x) for x in v[:6])
            for v in (positions, velocities, torques)
        ):
            self.invalidate(now, "invalid input")
            return
        if self.last is not None and (
            now <= self.last or now - self.last > self.cfg.link_timeout_s
        ):
            self.invalidate(now, "feedback gap")
        if not interior_pose(self.cfg, positions):
            self.invalidate(now, "near joint limit")
            self.blocked_joints = [
                i + 1
                for i, (joint, q) in enumerate(zip(self.cfg.joints[:6], positions[:6]))
                if not math.radians(joint.lower_deg + 2) <= q <= math.radians(joint.upper_deg - 2)
            ]
            return
        outside = workspace_blocked_joints(self.calibration.data, positions)
        if outside:
            self.invalidate(now, "outside calibration")
            self.blocked_joints = outside
            return
        self.blocked_joints = []
        dt = 1 / self.cfg.loop_hz if self.last is None else min(max(now - self.last, 0), 0.1)
        self.last = now
        ready = self.signals.update(now, velocities[:6])
        values, self.residual_nm = self.calibration.residual(
            positions[:6], self.signals.velocity, torques[:6]
        )
        steady = ready and self.settled.update(
            now,
            positions[:6],
            velocities[:6],
            self.signals.acceleration,
            self.cfg.payload_max_speed_rad_s,
            self.cfg.payload_max_accel_rad_s2,
        )
        target = 0.0
        if steady:
            evidence = load_evidence(
                values, self.residual_nm, self.calibration.sigma, self.calibration.axes
            )
            if evidence is None:
                self.invalidate(now, "unobservable pose")
                return
            self.score, self.coherence = evidence
            self.last_evidence = now
            if is_extra_load(*evidence):
                if self.last_positive is None or now - self.last_positive > CONFIRMATION_GAP_S:
                    self.confirmation_s = 0.0
                elif self.previous_positive:
                    self.confirmation_s += min(dt, 0.03)
                self.last_positive, self.previous_positive = now, True
                self.status = "confirming extra load"
                if self.confirmation_s >= 0.2:
                    # 阻尼表示超出空载波动的信号强度，不等同于物体质量。
                    target = min(1.0, (self.score - THRESHOLD) / THRESHOLD)
                    self.status = "extra load"
            else:
                self.last_positive = None
                self.confirmation_s, self.previous_positive = 0.0, False
                self.status = "no confirmed extra load"
        else:
            self.pause_confirmation(now)
            if (
                self.last_evidence is not None
                and now - self.last_evidence <= self.cfg.payload_hold_s
            ):
                target, self.status = self.strength, "motion: feedback held"
            else:
                self.status = "motion: feedback faded"
        # 固定空载模型；持物期间不更新零偏，也不将物体学习成自重。
        tau = self.cfg.payload_smoothing_s if target >= self.strength else 0.12
        self.strength += dt / (tau + dt) * (target - self.strength)
        if self.strength < 1e-4:
            self.strength = 0.0
        self.loads = [self.strength * weight for weight in self.cfg.payload_haptic_weights]


def load_observer(cfg):
    if cfg.feedback_mode == "extra_load":
        return ExtraLoadObserver(cfg, ExtraLoadCalibration.load(cfg.payload_calibration, cfg))
    from payload import PayloadObserver

    return PayloadObserver(cfg, Calibration.load(cfg.payload_calibration, cfg))
