import copy
import csv
import json
import math
import random
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from bus import State
from config import Config
from controller import Session
from gravity import GravityModel, dot
from payload import Calibration, PayloadObserver, friction_features, signature, vertical_fit
from payload_tools import fit_profile, load_samples
from teleop import SimLeader, hardware, main
from test_native_b601 import CANWire, Clock, LeaderWire, SerialCANWire


def toy_model(path):
    parts = ['<robot name="test"><link name="base_link"/>']
    for i in range(1, 7):
        parts.append(f"""<link name="link{i}"><inertial><origin xyz="0.08 0 0"/>
            <mass value="{0.2 + i * 0.04}"/></inertial></link>
            <joint name="joint{i}" type="revolute"><parent link="{'base_link' if i == 1 else f'link{i-1}'}"/>
            <child link="link{i}"/><origin xyz="{'0 0 0.1' if i == 1 else '0.2 0 0'}"/>
            <axis xyz="{'0 0 1' if i == 1 else '0 1 0'}"/></joint>""")
    parts.append(
        """<link name="end_link"><inertial><origin xyz="0.04 0 0"/><mass value="0.3"/></inertial></link>
        <joint name="end_joint" type="fixed"><parent link="link6"/><child link="end_link"/><origin xyz="0.2 0 0"/></joint></robot>"""
    )
    path.write_text("".join(parts), encoding="utf-8")


def samples(model, seed, count=220, payload=0):
    rng = random.Random(seed)
    rows = []
    for _ in range(count):
        q = [rng.uniform(-0.4, 0.4), rng.uniform(-1.9, -0.3), rng.uniform(-1.8, -0.2)] + [
            rng.uniform(-0.7, 0.7) for _ in range(3)
        ]
        v = [rng.uniform(-0.18, 0.18) for _ in range(6)]
        values = model.evaluate(q)
        tau = []
        for i in range(6):
            gain = -0.9 if i == 1 else 1.1
            coefficient = [0.03 * (i + 1), gain, 0.12, 0.04]
            torque = dot(coefficient, friction_features(values["gravity"][i], v[i]))
            tau.append(
                torque + gain * payload * values["unit_mass"][i] + rng.uniform(-0.003, 0.003)
            )
        rows.append({"q": q, "v": v, "tau": tau})
    return rows


class PayloadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.root = Path(cls.directory.name)
        cls.model_path = cls.root / "model.urdf"
        toy_model(cls.model_path)
        cls.model = GravityModel(cls.model_path)
        cls.cfg = Config()
        cls.training, cls.validation = samples(cls.model, 7), samples(cls.model, 91)
        cls.profile = fit_profile(cls.training, cls.validation, cls.model, cls.cfg)

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def observer(self):
        return PayloadObserver(
            copy.deepcopy(self.cfg), Calibration(copy.deepcopy(self.profile), self.model)
        )

    def torque(self, observer, q, mass=0.0, moments=(0, 0), v=None):
        v = v or [0.0] * 6
        values = self.model.evaluate(q)
        result = []
        for i, row in enumerate(observer.calibration.coefficients):
            extra = (
                mass * values["unit_mass"][i]
                + moments[0] * values["moment_x"][i]
                + moments[1] * values["moment_y"][i]
            )
            result.append(dot(row, friction_features(values["gravity"][i], v[i])) + row[1] * extra)
        return result

    def settle(self, observer, mass=0.0, duration=3, start=0, q=None):
        q = q or [0, -1.1, -1.0, 0.2, -0.1, 0.1]
        for tick in range(1, int(duration * 100) + 1):
            observer.update(q, [0.0] * 6, self.torque(observer, q, mass), start + tick / 100)
        return start + duration

    def test_gravity_matches_potential_gradient_all_joint_signs(self):
        model = GravityModel(self.model_path, [-1, 1, -1, 1, 1, -1], [0.1] * 6)
        q = [0.2, -0.8, -1, 0.3, 0.1, -0.1]
        values = model.evaluate(q)
        for i in range(6):
            plus, minus = list(q), list(q)
            plus[i] += 1e-6
            minus[i] -= 1e-6
            gradient = (
                model.evaluate(plus)["potential"] - model.evaluate(minus)["potential"]
            ) / 2e-6
            self.assertAlmostEqual(gradient, values["gravity"][i], places=6)

    def test_payload_column_matches_point_mass_potential(self):
        q = [0.1, -0.5, -1.5, 0.2, 0.3, 0.1]
        values = self.model.evaluate(q)
        for i in range(6):
            plus, minus = list(q), list(q)
            plus[i] += 1e-6
            minus[i] -= 1e-6
            potential_plus = -dot(self.model.gravity, self.model.evaluate(plus)["tip"])
            potential_minus = -dot(self.model.gravity, self.model.evaluate(minus)["tip"])
            self.assertAlmostEqual(
                (potential_plus - potential_minus) / 2e-6, values["unit_mass"][i], places=6
            )

    def test_vertical_fit_recovers_mass_with_unknown_center_of_mass_moments(self):
        values = self.model.evaluate([0.2, -0.7, -1.2, 0.4, 0.2, -0.1])
        residual = [
            0.65 * h + 0.4 * x - 0.3 * y
            for h, x, y in zip(values["unit_mass"], values["moment_x"], values["moment_y"])
        ]
        result = vertical_fit(values, residual, [0.03] * 6, [1, 2, 3, 4, 5])
        self.assertAlmostEqual(result[0], 0.65)
        self.assertLess(result[1], 1e-8)

    def test_unobservable_pose_is_not_assigned_a_weight(self):
        values = {k: [0.0] * 6 for k in ("unit_mass", "moment_x", "moment_y")}
        self.assertIsNone(vertical_fit(values, [3.0] * 6, [0.03] * 6, list(range(6))))

    def test_independent_empty_validation_and_signed_gain_fit(self):
        self.assertTrue(self.profile["validated"], self.profile["validation"])
        self.assertAlmostEqual(self.profile["coefficients"][1][1], -0.9, delta=0.02)
        self.assertLess(self.profile["validation"]["mass_rmse_kg"], 0.02)

    def test_loaded_validation_does_not_pass_as_empty(self):
        with self.assertRaisesRegex(ValueError, "Unloaded validation"):
            fit_profile(self.training, samples(self.model, 98, payload=0.5), self.model, self.cfg)

    def test_one_pose_tare_cannot_become_a_calibration(self):
        repeated = [self.training[0]] * 220
        with self.assertRaisesRegex(ValueError, "15 degrees"):
            fit_profile(repeated, self.validation, self.model, self.cfg)

    def test_empty_arm_varied_poses_has_no_haptic_load(self):
        observer = self.observer()
        now = 0
        for q in ([0, -0.4, -0.3, 0.5, -0.5, 0.1], [0.2, -1.8, -1.6, -0.5, 0.4, -0.4]):
            now = self.settle(observer, start=now, q=q)
            self.assertLess(observer.mass_kg, 0.02)
            self.assertEqual(observer.loads, [0.0] * 7)

    def test_payload_does_not_get_learned_away_when_held(self):
        observer = self.observer()
        self.settle(observer, 0.5, duration=15)
        self.assertAlmostEqual(observer.mass_kg, 0.5, delta=0.02)
        self.assertGreater(observer.loads[1], 0.5)
        self.assertEqual(observer.loads[0], 0)
        self.assertEqual(observer.loads[6], 0)

    def test_putting_object_down_releases_feedback(self):
        observer = self.observer()
        now = self.settle(observer, 0.5)
        self.settle(observer, 0, duration=2, start=now)
        self.assertLess(observer.mass_kg, 0.01)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_mass_estimate_is_not_clipped_to_haptic_saturation_weight(self):
        observer = self.observer()
        self.settle(observer, 1.0)
        self.assertAlmostEqual(observer.mass_kg, 1.0, delta=0.02)
        self.assertEqual(observer.loads[1], 1)
        self.assertTrue(all(0 <= value <= 1 for value in observer.loads))

    def test_fast_motion_cannot_create_a_new_payload_estimate(self):
        observer = self.observer()
        q = [0, -1.1, -1, 0, 0, 0]
        for tick in range(1, 300):
            observer.update(q, [1.0] * 6, [20.0] * 6, tick / 100)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_fast_motion_holds_estimate_only_for_bounded_time(self):
        observer = self.observer()
        now = self.settle(observer, 0.5)
        q = [0, -1.1, -1, 0, 0, 0]
        for tick in range(1, 350):
            observer.update(q, [1.0] * 6, [20.0] * 6, now + tick / 100)
        self.assertLess(observer.mass_kg, 0.01)

    def test_outside_calibrated_workspace_suppresses_feedback(self):
        observer = self.observer()
        self.settle(observer, 0.5)
        observer.update([2.0, -1, -1, 0, 0, 0], [0.0] * 6, [10.0] * 6, 3.01)
        self.assertEqual(observer.status, "outside calibration")
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_short_gap_does_not_repeatedly_drop_and_ramp_feedback(self):
        observer = self.observer()
        now = self.settle(observer, 0.5)
        before = observer.mass_kg
        observer.gap(now + 0.01)
        self.settle(observer, 0.5, duration=0.1, start=now + 0.01)
        self.assertAlmostEqual(observer.mass_kg, before, delta=0.01)
        observer.gap(now + 1)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_unexplained_multi_joint_torque_is_not_reported_as_payload(self):
        observer = self.observer()
        q = [0, -1.1, -1, 0, 0, 0]
        torque = self.torque(observer, q, 0.4)
        torque[3] += 4
        for tick in range(1, 250):
            observer.update(q, [0.0] * 6, torque, tick / 100)
        self.assertEqual(observer.status, "unexplained torque")
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_nonfinite_torque_cannot_reach_haptic_output(self):
        observer = self.observer()
        self.settle(observer, 0.5)
        observer.update([0.0] * 6, [0.0] * 6, [math.nan] * 6, 3.01)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_brief_torque_transient_does_not_create_weight_feedback(self):
        observer = self.observer()
        now = self.settle(observer)
        q = [0, -1.1, -1, 0.2, -0.1, 0.1]
        for tick in range(1, 6):
            observer.update(q, [0.0] * 6, self.torque(observer, q, 0.6), now + tick / 100)
        self.settle(observer, duration=0.2, start=now + 0.05)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_gripper_torque_alone_is_not_a_carried_payload(self):
        observer = self.observer()
        q = [0, -1.1, -1, 0.2, -0.1, 0.1]
        for tick in range(1, 300):
            observer.update(q + [0], [0.0] * 7, self.torque(observer, q) + [20.0], tick / 100)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_slow_empty_lifting_with_friction_does_not_create_payload(self):
        observer = self.observer()
        for tick in range(1, 1200):
            now = tick / 100
            q = [
                0,
                -1.1 + 0.3 * math.sin(now * 0.4),
                -1 + 0.3 * math.sin(now * 0.4 + 0.5),
                0.2,
                -0.1,
                0.1,
            ]
            v = [0, 0.12 * math.cos(now * 0.4), 0.12 * math.cos(now * 0.4 + 0.5), 0, 0, 0]
            observer.update(q, v, self.torque(observer, q, v=v), now)
        self.assertLess(observer.mass_kg, 0.03)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_tilted_base_gravity_uses_consistent_payload_moment_plane(self):
        model = GravityModel(self.model_path, gravity=(0, -6.9343487, -6.9343487))
        values = model.evaluate([0.2, -0.7, -1.2, 0.4, 0.2, -0.1])
        residual = [
            0.4 * h + 0.2 * x + 0.3 * y
            for h, x, y in zip(values["unit_mass"], values["moment_x"], values["moment_y"])
        ]
        result = vertical_fit(values, residual, [0.03] * 6, list(range(6)))
        self.assertAlmostEqual(result[0], 0.4)

    def test_unmodeled_mass_on_prismatic_branch_is_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.urdf"
            text = self.model_path.read_text().replace(
                "</robot>",
                """<link name="slider"/><link name="mass"><inertial><mass value="1"/></inertial></link>
                <joint name="slide" type="prismatic"><parent link="end_link"/><child link="slider"/></joint>
                <joint name="mass_joint" type="fixed"><parent link="slider"/><child link="mass"/></joint></robot>""",
            )
            path.write_text(text)
            with self.assertRaisesRegex(ValueError, "moving branch"):
                GravityModel(path)

    def write_profile(self, directory):
        path = Path(directory) / "profile.json"
        data = copy.deepcopy(self.profile)
        path.write_text(json.dumps(data), encoding="utf-8")
        return path

    def test_changed_pairing_refuses_old_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_profile(directory)
            cfg = copy.deepcopy(self.cfg)
            cfg.joints[1].follower_home_rad = -0.1
            with self.assertRaisesRegex(ValueError, "Pairing/mapping changed"):
                Calibration.load(path, cfg)

    def test_changed_urdf_refuses_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_profile(directory)
            data = json.loads(path.read_text())
            data["model_sha256"] = "0" * 64
            path.write_text(json.dumps(data))
            with self.assertRaisesRegex(ValueError, "URDF changed"):
                Calibration.load(path, self.cfg)

    def test_unvalidated_calibration_fails_before_opening_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_profile(directory)
            data = json.loads(path.read_text())
            data["validated"] = False
            path.write_text(json.dumps(data))
            cfg = copy.deepcopy(self.cfg)
            cfg.feedback_mode, cfg.payload_calibration, cfg.calibration_verified = (
                "payload",
                str(path),
                True,
            )
            factory = Mock()
            with self.assertRaises(ValueError):
                hardware(cfg, execute=True, serial_factory=factory)
            factory.assert_not_called()

    def test_profile_load_and_config_relative_path_roundtrip(self):
        with tempfile.TemporaryDirectory() as directory:
            path = self.write_profile(directory)
            self.assertIsNotNone(Calibration.load(path, self.cfg))
            cfg = copy.deepcopy(self.cfg)
            cfg.feedback_mode, cfg.payload_calibration = "payload", str(path)
            config_path = Path(directory) / "config.json"
            cfg.save(config_path)
            self.assertEqual(
                json.loads(config_path.read_text())["payload_calibration"], "profile.json"
            )
            self.assertEqual(Config.load(config_path).payload_calibration, str(path.resolve()))

    def test_unloaded_collection_suppresses_damping_but_keeps_fault_hold(self):
        cfg, leader, follower = copy.deepcopy(self.cfg), SimLeader(), Mock()
        initial = [State(0, 0, 0, 1, 25, 25, 0, i + 1) for i in range(7)]
        session = Session(cfg, leader, follower, initial, 0, calibrating=True)
        for tick in range(1, 120):
            now = tick / 100
            states = [State(0, 0, 5, 1, 25, 25, now, tick * 7 + i + 1) for i in range(7)]
            session.tick([0.0] * 7, states, now)
        self.assertTrue(all(power == 1 for _, mode, power in leader.calls))
        session.tick(None, None, 2)
        self.assertTrue(all(mode == 0x11 for _, mode, _ in leader.calls[-7:]))

    def test_unloaded_collection_requires_explicit_confirmation(self):
        with self.assertRaises(SystemExit):
            main(["--execute", "--collect-unloaded", "empty.csv"])

    def test_old_unknown_load_csv_is_not_accepted_for_calibration(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "unknown.csv"
            path.write_text("valid,torque_j1_nm\n1,0\n")
            path.with_suffix(".csv.meta.json").write_text(
                json.dumps({"confirmed_unloaded": False, "config_signature": signature(self.cfg)})
            )
            with self.assertRaisesRegex(ValueError, "confirmed unloaded"):
                load_samples(path, self.cfg)

    def test_unloaded_capture_writes_bound_metadata_velocities_and_zero_damping(self):
        cfg = copy.deepcopy(self.cfg)
        cfg.calibration_verified, cfg.baseline_s = True, 0.2
        clock, can, leader = Clock(), CANWire(), LeaderWire()
        wire = SerialCANWire(can)
        ports = iter((leader, wire))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "empty.csv"
            session = hardware(
                cfg,
                execute=True,
                collect_unloaded=True,
                diagnostics=path,
                duration=0.06,
                serial_factory=lambda *a, **kw: next(ports),
                clock=clock,
                sleep=clock.sleep,
            )
            metadata = json.loads(path.with_suffix(".csv.meta.json").read_text())
            self.assertEqual(metadata["config_signature"], signature(cfg))
            self.assertIs(metadata["confirmed_unloaded"], True)
            with path.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertTrue(rows)
            self.assertTrue(all(row["unloaded_collection"] == "1" for row in rows))
            self.assertTrue(all(f"velocity_j{i}_rad_s" in rows[0] for i in range(1, 8)))
            self.assertTrue(
                all(float(row[f"haptic_j{i}"]) == 0 for row in rows for i in range(1, 8))
            )
            self.assertGreater(session.cycles, 0)
        self.assertEqual(can.enabled, set())
        self.assertTrue(leader.closed and wire.closed)


if __name__ == "__main__":
    unittest.main()
