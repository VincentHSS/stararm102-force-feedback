"""用明确空载的新姿态验证固定模型；不改系数、噪声或判定阈值，不打开串口。"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
from pathlib import Path

from config import Config
from extra_load import (
    ExtraLoadCalibration,
    WORKSPACE_RADIUS_RAD,
    is_extra_load,
    load_evidence,
    model_parameters_hash,
)
from payload import SETTLED_GATE, interior_pose
from payload_tools import load_samples, new_json, percentile


def validate_workspace(calibration, samples, cfg, source_hash):
    if any(row.get("settled_pose") is not True for row in samples):
        raise ValueError("Workspace samples must pass the continuous settled-pose gate")
    samples = [row for row in samples if interior_pose(cfg, row["q"])]
    if len(samples) < 80:
        raise ValueError("Need at least 80 stable unloaded workspace samples")
    evidence, observable = [], []
    for row in samples:
        values, residual = calibration.residual(row["q"], row["v"], row["tau"])
        result = load_evidence(values, residual, calibration.sigma, calibration.axes)
        if result is not None:
            evidence.append(result)
            observable.append(row["q"])
    if len(evidence) < 80 or len(evidence) / len(samples) < 0.7:
        raise ValueError("Need at least 80 observable samples and 70% observable coverage")
    report = {
        "samples": len(samples),
        "observable_samples": len(evidence),
        "valid_fraction": len(evidence) / len(samples),
        "false_activation_fraction": sum(is_extra_load(*row) for row in evidence) / len(evidence),
        "score_p99": percentile([row[0] for row in evidence], 0.99),
        "score_max": max(row[0] for row in evidence),
    }
    if report["false_activation_fraction"]:
        raise ValueError("Unloaded workspace produces false load evidence: " + json.dumps(report))
    # 合并非常接近的实测姿态，仅允许每个实测姿态周围 5° 的邻域。
    poses = []
    for pose in observable:
        if not any(max(abs(a - b) for a, b in zip(pose, old)) < math.radians(1) for old in poses):
            poses.append(list(pose))
    result = copy.deepcopy(calibration.data)
    entry = {
        "validated": True,
        "confirmed_unloaded": True,
        "evidence_gate": SETTLED_GATE,
        "source_sha256": source_hash,
        "model_parameters_sha256": model_parameters_hash(result),
        "radius_rad": WORKSPACE_RADIUS_RAD,
        "poses_rad": poses,
        "validation": report,
    }
    result.setdefault("workspace_validations", []).append(entry)
    ExtraLoadCalibration.validate_report(result)
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Validate extra-load workspace without opening devices"
    )
    for name in ("config", "profile", "validation", "output"):
        parser.add_argument("--" + name, type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        cfg = Config.load(args.config)
        calibration = ExtraLoadCalibration.load(args.profile, cfg)
        profile = validate_workspace(
            calibration,
            load_samples(args.validation, cfg, require_settled=True),
            cfg,
            hashlib.sha256(args.validation.read_bytes()).hexdigest(),
        )
        profile["model_path"] = os.path.relpath(
            calibration.model.path, args.output.resolve().parent
        )
        new_json(args.output, profile)
        print(json.dumps(profile["workspace_validations"][-1]["validation"], indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Workspace not enabled: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
