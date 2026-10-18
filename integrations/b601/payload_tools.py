"""空载数据拟合、独立验证与配置生成；本工具不打开串口。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import statistics
import urllib.request
import xml.etree.ElementTree as ET
from pathlib import Path

from config import Config
from gravity import GravityModel, dot
from payload import (
    ALGORITHM,
    Calibration,
    MotionSignals,
    friction_features,
    signature,
    vertical_fit,
)

MODEL_COMMIT = "0b0b29ef92b4e03f0553dbe7502a8afe164dbb29"
MODEL_SHA256 = "bce0d4740ec97a27e30d521e890416231c6d0117093fa997d1ae7ac95204ffb6"
MODEL_URL = f"https://raw.githubusercontent.com/Seeed-Projects/reBot-DevArm/{MODEL_COMMIT}/Rebot_Arm_description/DM/urdf/ReBot_Arm_DM.urdf"


def percentile(values, fraction):
    return sorted(values)[min(len(values) - 1, math.ceil((len(values) - 1) * fraction))]


def solve(matrix, rhs):
    a = [list(row) + [value] for row, value in zip(matrix, rhs)]
    for i in range(len(a)):
        pivot = max(range(i, len(a)), key=lambda j: abs(a[j][i]))
        a[i], a[pivot] = a[pivot], a[i]
        if abs(a[i][i]) < 1e-12:
            raise ValueError("Calibration data do not excite independent model features")
        divisor = a[i][i]
        a[i] = [x / divisor for x in a[i]]
        for j in range(len(a)):
            if i != j:
                factor = a[j][i]
                a[j] = [x - factor * y for x, y in zip(a[j], a[i])]
    return [row[-1] for row in a]


def regress(features, values, prior):
    count, width = len(features), len(features[0])
    scales = [
        max(0.01, math.sqrt(sum(row[i] ** 2 for row in features) / count)) for i in range(width)
    ]
    x = [[v / scale for v, scale in zip(row, scales)] for row in features]
    weights = [1.0] * count
    for _ in range(4):
        matrix = [
            [sum(w * row[i] * row[j] for w, row in zip(weights, x)) for j in range(width)]
            for i in range(width)
        ]
        rhs = [sum(w * row[i] * y for w, row, y in zip(weights, x, values)) for i in range(width)]
        for i in range(width):
            ridge = count * (1e-8 if i == 0 else 1e-4)
            matrix[i][i] += ridge
            rhs[i] += ridge * prior[i] * scales[i]
        beta = solve(matrix, rhs)
        errors = [y - dot(beta, row) for row, y in zip(x, values)]
        scale = max(0.005, 1.4826 * statistics.median(abs(v) for v in errors))
        weights = [min(1.0, 1.5 * scale / max(abs(v), 1e-9)) for v in errors]
    return [b / scale for b, scale in zip(beta, scales)]


def load_samples(path, cfg):
    path = Path(path)
    metadata = json.loads(path.with_suffix(path.suffix + ".meta.json").read_text(encoding="utf-8"))
    if metadata.get("confirmed_unloaded") is not True or metadata.get(
        "config_signature"
    ) != signature(cfg):
        raise ValueError("Use separately collected, confirmed unloaded data from the same pairing")
    signals, result, last_sample = MotionSignals(), [], -math.inf
    with path.open(encoding="utf-8-sig", newline="") as stream:
        for row in csv.DictReader(stream):
            try:
                if (
                    row["valid"] != "1"
                    or row["unloaded_collection"] != "1"
                    or row["state"] != "feedback"
                ):
                    signals = MotionSignals()
                    continue
                now = float(row["elapsed_s"])
                q = [float(row[f"actual_j{i}_rad"]) for i in range(1, 7)]
                raw_v = [float(row[f"velocity_j{i}_rad_s"]) for i in range(1, 7)]
                tau = [float(row[f"torque_j{i}_nm"]) for i in range(1, 7)]
                if not all(math.isfinite(v) for v in q + raw_v + tau + [now]):
                    signals = MotionSignals()
                    continue
                ready = signals.update(now, raw_v)
                if (
                    not ready
                    or max(map(abs, raw_v)) > cfg.payload_max_speed_rad_s
                    or max(map(abs, signals.acceleration)) > cfg.payload_max_accel_rad_s2
                ):
                    continue
                if now - last_sample < 0.05:
                    continue
                if any(
                    not math.radians(j.lower_deg) - 0.002
                    <= value
                    <= math.radians(j.upper_deg) + 0.002
                    for value, j in zip(q, cfg.joints)
                ):
                    continue
                result.append({"q": q, "v": list(signals.velocity), "tau": tau})
                last_sample = now
            except (KeyError, ValueError):
                raise ValueError(
                    "CSV lacks signed torque/velocity/unloaded fields; collect new unloaded data"
                ) from None
    return result


def fit_profile(training, validation, model, cfg):
    if len(training) < 120 or len(validation) < 80:
        raise ValueError(
            "Need at least 120 training and 80 separate validation samples after motion filtering"
        )
    for name, samples in (("training", training), ("validation", validation)):
        for i in (1, 2):
            span = max(row["q"][i] for row in samples) - min(row["q"][i] for row in samples)
            if span < math.radians(15):
                raise ValueError(
                    f"{name}: move J{i + 1} through at least 15 degrees across unloaded poses"
                )
    gravity_rows = [model.evaluate(row["q"])["gravity"] for row in training]
    coefficients, axes = [], []
    for i in range(6):
        g = [row[i] for row in gravity_rows]
        active = statistics.pstdev(g) >= 0.08
        features = [
            friction_features(gravity[i], row["v"][i])
            for row, gravity in zip(training, gravity_rows)
        ]
        y = [row["tau"][i] for row in training]
        if active:
            beta = regress(features, y, [0, 1, 0, 0])
            axes.append(i)
        else:
            # 重力变化不足时不从噪声拟合比例；该轴不参与物体质量估计。
            beta3 = regress(
                [[x[0], x[2], x[3]] for x in features],
                [value - x[1] for value, x in zip(y, features)],
                [0, 0, 0],
            )
            beta = [beta3[0], 1.0, beta3[1], beta3[2]]
        if not 0.2 <= abs(beta[1]) <= 3:
            raise ValueError(
                f"J{i + 1}: torque sign/scale does not fit model; check model zero, signs and gravity direction"
            )
        coefficients.append(beta)
    if len(axes) < 3:
        raise ValueError(
            "Need gravity excitation in at least three arm joints; add unloaded wrist/arm poses"
        )
    bounds = [
        [min(row["q"][i] for row in training), max(row["q"][i] for row in training)]
        for i in range(6)
    ]
    # 静止轴也保存窄区间，运行时仅允许小范围外插。
    bounds = [[lo - 1e-5, hi + 1e-5] for lo, hi in bounds]
    errors = [[] for _ in range(6)]
    valid_rows = []
    for row in validation:
        if any(
            not lo - math.radians(5) <= q <= hi + math.radians(5)
            for q, (lo, hi) in zip(row["q"], bounds)
        ):
            continue
        values = model.evaluate(row["q"])
        residual = [
            (
                row["tau"][i]
                - dot(coefficients[i], friction_features(values["gravity"][i], row["v"][i]))
            )
            / coefficients[i][1]
            for i in range(6)
        ]
        for i, value in enumerate(residual):
            errors[i].append(value)
        valid_rows.append((values, residual))
    if len(valid_rows) < 80:
        raise ValueError(
            "Validation poses exceed training coverage; collect a wider unloaded training path"
        )
    sigma = [max(0.03, percentile([abs(x) for x in row], 0.95)) for row in errors]
    masses = []
    for values, residual in valid_rows:
        result = vertical_fit(values, residual, sigma, axes)
        if result is not None and result[1] <= 3 and result[2] <= 0.2:
            masses.append(result[0])
    if not masses:
        raise ValueError(
            "Unloaded validation has no reliable mass fits; check empty gripper, model mapping and pose excitation"
        )
    report = {
        "samples": len(validation),
        "valid_fraction": len(masses) / len(validation),
        "mass_rmse_kg": math.sqrt(sum(x * x for x in masses) / len(masses)),
        "mass_abs_p99_kg": percentile([abs(x) for x in masses], 0.99),
        "joint_residual_rmse_nm": [math.sqrt(sum(x * x for x in row) / len(row)) for row in errors],
    }
    deadband = max(0.05, report["mass_abs_p99_kg"])
    passed = report["valid_fraction"] >= 0.7 and report["mass_rmse_kg"] <= 0.1 and deadband <= 0.2
    return {
        "algorithm": ALGORITHM,
        "validated": passed,
        "config_signature": signature(cfg),
        "model_path": str(model.path),
        "model_sha256": model.sha256,
        "model_directions": model.directions,
        "model_offsets_rad": model.offsets,
        "gravity_m_s2": list(model.gravity),
        "coefficients": coefficients,
        "sigma_nm": sigma,
        "active_axes": axes,
        "bounds_rad": bounds,
        "mass_deadband_kg": deadband,
        "training_samples": len(training),
        "validation": report,
    }


def new_json(path, data):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("x", encoding="utf-8") as stream:
        json.dump(data, stream, indent=2, ensure_ascii=False, allow_nan=False)
        stream.write("\n")


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="B601 unloaded calibration; no serial device is opened"
    )
    commands = parser.add_subparsers(dest="command", required=True)
    fetch = commands.add_parser("fetch-model")
    fetch.add_argument("--output", type=Path, required=True)
    fit = commands.add_parser("fit")
    for name in ("config", "model", "train", "validation", "output"):
        fit.add_argument("--" + name, type=Path, required=True)
    fit.add_argument("--directions", nargs=6, type=float, default=[1.0] * 6)
    fit.add_argument("--offsets-deg", nargs=6, type=float, default=[0.0] * 6)
    fit.add_argument("--gravity", nargs=3, type=float, default=[0, 0, -9.80665])
    for name in ("enable", "inspect"):
        sub = commands.add_parser(name)
        sub.add_argument("--config", type=Path, required=True)
        sub.add_argument("--profile", type=Path, required=True)
        if name == "enable":
            sub.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "fetch-model":
            if args.output.exists():
                raise FileExistsError(args.output)
            with urllib.request.urlopen(MODEL_URL, timeout=30) as response:
                data = response.read(1_000_001)
            if len(data) > 1_000_000 or hashlib.sha256(data).hexdigest() != MODEL_SHA256:
                raise ValueError("Official model checksum mismatch; no model installed")
            args.output.parent.mkdir(parents=True, exist_ok=True)
            with args.output.open("xb") as stream:
                stream.write(data)
            print(f"Model saved: {args.output}; source commit {MODEL_COMMIT}")
            return 0
        cfg = Config.load(args.config)
        if args.command == "fit":
            train_hash, val_hash = [
                hashlib.sha256(p.read_bytes()).hexdigest() for p in (args.train, args.validation)
            ]
            if train_hash == val_hash:
                raise ValueError("Training and validation must be two separate unloaded runs")
            model = GravityModel(
                args.model,
                args.directions,
                [math.radians(x) for x in args.offsets_deg],
                args.gravity,
            )
            profile = fit_profile(
                load_samples(args.train, cfg), load_samples(args.validation, cfg), model, cfg
            )
            profile["data_sha256"] = {"training": train_hash, "validation": val_hash}
            profile["model_path"] = os.path.relpath(model.path, args.output.resolve().parent)
            new_json(args.output, profile)
            print(
                json.dumps(
                    {
                        "validated": profile["validated"],
                        "deadband_kg": profile["mass_deadband_kg"],
                        "validation": profile["validation"],
                    },
                    ensure_ascii=False,
                    indent=2,
                )
            )
            return 0 if profile["validated"] else 2
        calibration = Calibration.load(args.profile, cfg)
        if args.command == "enable":
            if args.output.exists():
                raise FileExistsError(args.output)
            cfg.feedback_mode, cfg.payload_calibration = "payload", str(args.profile.resolve())
            cfg.save(args.output)
            print(f"Payload configuration saved: {args.output}; no device opened")
        else:
            print(json.dumps(calibration.data["validation"], indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError, ET.ParseError) as exc:
        print(f"Stopped: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
