import copy
import json
import math
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock

from bus import State
from config import Config
from controller import Session
from extra_load import ExtraLoadCalibration, ExtraLoadObserver, load_evidence
from gravity import GravityModel, dot
from payload import friction_features
from payload_tools import fit_profile
from teleop import SimLeader, hardware
from test_payload import samples, toy_model


class ExtraLoadTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.directory = tempfile.TemporaryDirectory()
        cls.root = Path(cls.directory.name)
        path = cls.root / "model.urdf"
        toy_model(path)
        cls.model, cls.cfg = GravityModel(path), Config()
        cls.train, cls.check = samples(cls.model, 7), samples(cls.model, 91)
        cls.profile = fit_profile(cls.train, cls.check, cls.model, cls.cfg, "extra_load")

    @classmethod
    def tearDownClass(cls):
        cls.directory.cleanup()

    def observer(self):
        return ExtraLoadObserver(
            copy.deepcopy(self.cfg), ExtraLoadCalibration(copy.deepcopy(self.profile), self.model)
        )

    def step(self, observer, now, mass=0, v=None, q=None):
        q = q or [0, -1.1, -1.0, 0.2, -0.1, 0.1]
        v = v or [0.0] * 6
        values = self.model.evaluate(q)
        torques = [
            dot(row, friction_features(values["gravity"][i], v[i]))
            + row[1] * mass * values["unit_mass"][i]
            for i, row in enumerate(observer.calibration.coefficients)
        ]
        observer.update(q, v, torques, now)

    def settle(self, observer, mass=0, start=0, duration=3):
        for tick in range(1, int(duration * 100) + 1):
            self.step(observer, start + tick / 100, mass)
        return start + duration

    def test_profile_uses_training_noise_and_empty_validation_only(self):
        self.assertTrue(self.profile["validated"], self.profile["validation"])
        changed = copy.deepcopy(self.check)
        for row in changed:
            row["tau"][1] += 0.02
        profile = fit_profile(self.train, changed, self.model, self.cfg, "extra_load")
        self.assertEqual(self.profile["sigma_nm"], profile["sigma_nm"])
        self.assertEqual(self.profile["coefficients"], profile["coefficients"])
        self.assertEqual(self.profile["validation_scope"], "independent_unloaded_only")

    def test_loaded_validation_cannot_be_accepted_as_empty(self):
        profile = fit_profile(
            self.train, samples(self.model, 91, payload=0.5), self.model, self.cfg, "extra_load"
        )
        self.assertFalse(profile["validated"])
        self.assertGreater(profile["validation"]["false_activation_fraction"], 0)

    def test_empty_slow_lifting_and_gripper_load_have_no_drag(self):
        observer = self.observer()
        for tick in range(1, 1000):
            t = tick / 100
            q = [
                0,
                -1.1 + 0.2 * math.sin(t * 0.4),
                -1 + 0.2 * math.sin(t * 0.4 + 0.5),
                0.2,
                -0.1,
                0.1,
            ]
            v = [0, 0.08 * math.cos(t * 0.4), 0.08 * math.cos(t * 0.4 + 0.5), 0, 0, 0]
            self.step(observer, t, q=q, v=v)
        self.assertEqual(observer.loads, [0.0] * 7)
        self.assertTrue(math.isnan(observer.mass_kg))

    def test_pick_up_hold_and_put_down_without_online_tare(self):
        observer = self.observer()
        now = self.settle(observer)
        self.assertEqual(observer.strength, 0)
        now = self.settle(observer, 0.5, start=now, duration=15)
        self.assertGreater(observer.loads[1], 0.95)
        self.assertEqual(observer.loads[0], 0)
        self.assertEqual(observer.loads[6], 0)
        self.settle(observer, start=now, duration=2)
        self.assertEqual(observer.loads, [0.0] * 7)

    def test_short_pulse_and_negative_load_are_rejected(self):
        observer = self.observer()
        now = self.settle(observer)
        for tick in range(1, 11):
            self.step(observer, now + tick / 100, 1)
        self.assertEqual(observer.strength, 0)
        self.settle(observer, -0.5, start=now + 0.1)
        self.assertEqual(observer.strength, 0)

    def test_fast_motion_cannot_confirm_load_and_hold_is_bounded(self):
        observer = self.observer()
        for tick in range(1, 300):
            self.step(observer, tick / 100, 1, v=[1.0] * 6)
        self.assertEqual(observer.strength, 0)
        now = self.settle(observer, 0.5, start=3)
        for tick in range(1, 350):
            self.step(observer, now + tick / 100, 1, v=[1.0] * 6)
        self.assertLess(observer.strength, 0.001)

    def test_interrupted_confirmation_does_not_count_unobserved_time(self):
        observer = self.observer()
        now = self.settle(observer)
        for tick in range(1, 11):
            self.step(observer, now + tick / 100, 1)
        observer.gap(now + 0.11)
        for tick in range(12, 25):
            self.step(observer, now + tick / 100, 1)
        self.assertEqual(observer.strength, 0)

    def test_gap_hold_then_timeout_clears_and_direct_long_gap_rewarms(self):
        observer = self.observer()
        now = self.settle(observer, 0.5)
        before = observer.strength
        observer.gap(now + 0.01)
        self.assertEqual(observer.strength, before)
        observer.gap(now + 0.3)
        self.assertEqual(observer.strength, 0)
        self.settle(observer, 0.5, start=now + 0.3)
        self.step(observer, now + 4, 0.5)
        self.assertEqual(observer.strength, 0)

    def test_limit_outside_range_and_invalid_input_clear_feedback(self):
        for q, expected in (
            ([0, -0.01, -1, 0, 0, 0], "near joint limit"),
            ([2, -1, -1, 0, 0, 0], "outside calibration"),
        ):
            observer = self.observer()
            self.settle(observer, 0.5)
            self.step(observer, 3.01, 0.5, q=q)
            self.assertEqual(observer.loads, [0.0] * 7)
            self.assertEqual(observer.status, expected)
        observer = self.observer()
        self.settle(observer, 0.5)
        observer.update([0] * 6, [0] * 6, [math.nan] * 6, 3.01)
        self.assertEqual(observer.strength, 0)

    def test_unobservable_and_incoherent_torques_are_not_payload_evidence(self):
        self.assertIsNone(load_evidence({"unit_mass": [0] * 6}, [10] * 6, [1] * 6, [1, 2, 3]))
        score, coherence = load_evidence(
            {"unit_mass": [0, 1, 0, 0, 0, 0]}, [0, 4, 100, 0, 0, 0], [1] * 6, [1, 2, 3]
        )
        self.assertGreater(score, 3)
        self.assertLess(coherence, 0.5)

    def test_gripper_torque_and_fault_hold_keep_separate_meanings(self):
        cfg, observer, leader, follower = (
            copy.deepcopy(self.cfg),
            self.observer(),
            SimLeader(),
            Mock(),
        )
        cfg.feedback_mode = "extra_load"
        q = [0, -1.1, -1, 0.2, -0.1, 0.1, -0.5]
        for joint, angle in zip(cfg.joints, q):
            joint.follower_home_rad = angle
        initial = [State(p, 0, 0, 1, 25, 25, 0, i + 1) for i, p in enumerate(q)]
        session = Session(cfg, leader, follower, initial, 0, payload_observer=observer)
        values = self.model.evaluate(q[:6])
        torque = [
            dot(row, friction_features(values["gravity"][i], 0))
            for i, row in enumerate(observer.calibration.coefficients)
        ] + [20]
        for tick in range(1, 120):
            now = tick / 100
            states = [
                State(p, 0, t, 1, 25, 25, now, tick * 7 + i + 1)
                for i, (p, t) in enumerate(zip(q, torque))
            ]
            session.tick([0.0] * 7, states, now)
        self.assertEqual(observer.strength, 0)
        self.assertTrue(all(power == 1 for _, mode, power in leader.calls))
        session.tick(None, None, 2)
        self.assertTrue(session.fault)
        self.assertTrue(all(mode == 0x11 for _, mode, _ in leader.calls[-7:]))

    def test_reject_failed_tampered_and_wrong_type_profiles_before_ports(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "profile.json"
            cfg = copy.deepcopy(self.cfg)
            cfg.feedback_mode, cfg.payload_calibration, cfg.calibration_verified = (
                "extra_load",
                str(path),
                True,
            )
            for changes in (
                {"validated": False},
                {"evidence_threshold": 1},
                {"algorithm": "b601-quasistatic-payload-v2"},
            ):
                path.write_text(json.dumps({**self.profile, **changes}), encoding="utf-8")
                factory = Mock()
                with self.assertRaises(ValueError):
                    hardware(cfg, execute=True, serial_factory=factory)
                factory.assert_not_called()
            path.write_text(json.dumps(self.profile), encoding="utf-8")
            self.assertIsNotNone(ExtraLoadCalibration.load(path, cfg))
            cfg.joints[1].follower_home_rad += 0.1
            with self.assertRaisesRegex(ValueError, "Pairing/mapping changed"):
                ExtraLoadCalibration.load(path, cfg)


if __name__ == "__main__":
    unittest.main()
