import copy
import csv
import hashlib
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path

from extra_load import ExtraLoadCalibration, workspace_blocked_joints
from payload import signature
from payload_tools import fit_profile, load_samples
from test_payload import samples, toy_model
from config import Config
from gravity import GravityModel, dot
from payload import friction_features
from workspace_validation import validate_workspace

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "windows"))
from workspace_capture import begin, finish


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        model_path = self.root / "model.urdf"
        toy_model(model_path)
        self.model, self.cfg = GravityModel(model_path), Config()
        profile = fit_profile(
            samples(self.model, 7), samples(self.model, 91), self.model, self.cfg, "extra_load"
        )
        self.calibration = ExtraLoadCalibration(profile, self.model)

    def records(self, mass=0):
        q = [0, -0.15, -0.3, 0.2, -0.1, 0.1]
        values = self.model.evaluate(q)
        tau = [
            dot(row, friction_features(values["gravity"][i], 0))
            + row[1] * mass * values["unit_mass"][i]
            for i, row in enumerate(self.calibration.coefficients)
        ]
        return [
            {"q": list(q), "v": [0] * 6, "tau": list(tau), "settled_pose": True} for _ in range(100)
        ]

    def test_extension_keeps_frozen_model_and_only_adds_nearby_validated_poses(self):
        original = copy.deepcopy(self.calibration.data)
        q = self.records()[0]["q"]
        self.assertTrue(workspace_blocked_joints(original, q))
        profile = validate_workspace(self.calibration, self.records(), self.cfg, "a" * 64)
        self.assertFalse(workspace_blocked_joints(profile, q))
        far = list(q)
        far[0] += math.radians(20)
        self.assertTrue(workspace_blocked_joints(profile, far))
        del profile["workspace_validations"]
        self.assertEqual(profile, original)
        self.assertEqual(self.calibration.data, original)

    def test_loaded_empty_validation_rejected_and_model_tampering_invalidates_extension(self):
        with self.assertRaisesRegex(ValueError, "false load evidence"):
            validate_workspace(self.calibration, self.records(2), self.cfg, "b" * 64)
        profile = validate_workspace(self.calibration, self.records(), self.cfg, "b" * 64)
        profile["sigma_nm"][1] *= 2
        with self.assertRaisesRegex(ValueError, "frozen model"):
            ExtraLoadCalibration.validate_report(profile)

    def test_new_workspace_does_not_bypass_joint_limit_guard(self):
        from extra_load import ExtraLoadObserver

        profile = validate_workspace(self.calibration, self.records(), self.cfg, "a" * 64)
        near = [0, -0.01, -0.3, 0.2, -0.1, 0.1]
        profile["workspace_validations"][0]["poses_rad"].append(near)
        observer = ExtraLoadObserver(self.cfg, ExtraLoadCalibration(profile, self.model))
        observer.update(near, [0] * 6, [0] * 6, 1)
        self.assertEqual(observer.status, "near joint limit")
        self.assertEqual(observer.loads, [0] * 7)

    def test_unsettled_rows_cannot_expand_workspace(self):
        rows = self.records()
        rows[0]["settled_pose"] = False
        with self.assertRaisesRegex(ValueError, "settled-pose"):
            validate_workspace(self.calibration, rows, self.cfg, "a" * 64)

    def write_csv(self, path, count):
        fields = ["elapsed_s", "valid", "state", "unloaded_collection"]
        fields += [
            f"{prefix}_j{i}_{unit}"
            for prefix, unit in (("actual", "rad"), ("velocity", "rad_s"), ("torque", "nm"))
            for i in range(1, 7)
        ]
        with path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            record = self.records()[0]
            for tick in range(count):
                row = dict(elapsed_s=tick * 0.05, valid=1, state="feedback", unloaded_collection=0)
                for prefix, unit, values in (
                    ("actual", "rad", record["q"]),
                    ("velocity", "rad_s", record["v"]),
                    ("torque", "nm", record["tau"]),
                ):
                    for i, value in enumerate(values, 1):
                        # Earlier mixed frames must never enter the explicitly empty interval.
                        row[f"{prefix}_j{i}_{unit}"] = (
                            100 if prefix == "torque" and tick < 100 else value
                        )
                writer.writerow(row)

    def test_regular_feedback_requires_complete_hashed_empty_interval(self):
        path = self.root / "regular.csv"
        self.write_csv(path, 300)
        sidecar = path.with_suffix(".csv.meta.json")
        meta = {"confirmed_unloaded": True, "config_signature": signature(self.cfg)}
        sidecar.write_text(json.dumps(meta))
        self.assertFalse(load_samples(path, self.cfg))
        meta.update(
            sample_selection="operator_confirmed_empty_interval",
            capture_complete=False,
            sample_intervals_s=[[5, 14.95]],
            source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
        )
        sidecar.write_text(json.dumps(meta))
        with self.assertRaisesRegex(ValueError, "incomplete"):
            load_samples(path, self.cfg)
        meta["capture_complete"] = True
        sidecar.write_text(json.dumps(meta))
        rows = load_samples(path, self.cfg)
        self.assertGreater(len(rows), 80)
        self.assertTrue(all(row["tau"] == self.records()[0]["tau"] for row in rows))
        path.write_text(path.read_text() + "\n")
        with self.assertRaisesRegex(ValueError, "checksum"):
            load_samples(path, self.cfg)

    def test_capture_rejects_stale_unchanged_and_rotated_logs(self):
        path, pending = self.root / "live.csv", self.root / "pending.json"
        self.write_csv(path, 120)
        clock = lambda: path.stat().st_mtime
        with self.assertRaisesRegex(ValueError, "stale"):
            begin(path, pending, self.cfg, lambda: clock() + 10)
        begin(path, pending, self.cfg, clock)
        with self.assertRaisesRegex(ValueError, "No new empty"):
            finish(pending, self.root / "empty.csv", self.cfg, clock)
        self.write_csv(path, 400)
        result = finish(pending, self.root / "empty.csv", self.cfg, clock)
        self.assertTrue(result["capture_complete"])
        self.assertGreater(len(load_samples(self.root / "empty.csv", self.cfg)), 80)
        with self.assertRaisesRegex(ValueError, "completed"):
            finish(pending, self.root / "again.csv", self.cfg, clock)
        pending = self.root / "next_pending.json"
        begin(path, pending, self.cfg, clock)
        path.write_text(path.read_text().replace("feedback", "replaced", 1))
        with self.assertRaisesRegex(ValueError, "rotated"):
            finish(pending, self.root / "other.csv", self.cfg, clock)


if __name__ == "__main__":
    unittest.main()
