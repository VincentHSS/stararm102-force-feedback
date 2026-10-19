"""扣除自重与摩擦后，按多轴额外负载证据输出阻尼；不输出公斤数。"""

from __future__ import annotations

import math

from gravity import dot
from payload import Calibration, MotionSignals, interior_pose

ALGORITHM = "b601-extra-load-v1"
THRESHOLD = 3.0
MIN_COHERENCE = 0.5
MIN_INFORMATION = 1.0


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


class ExtraLoadObserver:
    def __init__(self, cfg, calibration):
        self.cfg, self.calibration = cfg, calibration
        self.signals = MotionSignals()
        self.loads = [0.0] * 7
        self.residual_nm = [0.0] * 6
        self.score = self.coherence = math.nan
        self.strength = 0.0
        self.status = "warming up"
        self.last = self.last_evidence = self.positive_since = None
        # 保持旧诊断列兼容；额外负载模式从不产生质量读数。
        self.mass_kg = self.raw_mass_kg = self.sigma_kg = math.nan

    def invalidate(self, now, reason):
        self.signals = MotionSignals()
        self.loads, self.strength = [0.0] * 7, 0.0
        self.score = self.coherence = math.nan
        self.last_evidence = self.positive_since = None
        self.last, self.status = now, reason

    def gap(self, now):
        self.positive_since = None
        if self.last_evidence is None or now - self.last_evidence >= min(
            self.cfg.link_timeout_s, self.cfg.payload_hold_s
        ):
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
            return
        if any(
            not lo - math.radians(5) <= q <= hi + math.radians(5)
            for q, (lo, hi) in zip(positions, self.calibration.data["bounds_rad"])
        ):
            self.invalidate(now, "outside calibration")
            return
        dt = 1 / self.cfg.loop_hz if self.last is None else min(max(now - self.last, 0), 0.1)
        self.last = now
        ready = self.signals.update(now, velocities[:6])
        values, self.residual_nm = self.calibration.residual(
            positions[:6], self.signals.velocity, torques[:6]
        )
        steady = (
            ready
            and max(map(abs, velocities[:6])) <= self.cfg.payload_max_speed_rad_s
            and max(map(abs, self.signals.acceleration)) <= self.cfg.payload_max_accel_rad_s2
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
                if self.positive_since is None:
                    self.positive_since = now
                self.status = "confirming extra load"
                if now - self.positive_since >= 0.2:
                    # 阻尼表示超出空载波动的信号强度，不等同于物体质量。
                    target = min(1.0, (self.score - THRESHOLD) / THRESHOLD)
                    self.status = "extra load"
            else:
                self.positive_since = None
                self.status = "no confirmed extra load"
        else:
            self.positive_since = None
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
