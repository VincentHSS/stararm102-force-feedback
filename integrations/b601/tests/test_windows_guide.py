import contextlib
import importlib.util
import io
import json
import sys
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

APP = Path(__file__).resolve().parents[1] / "windows"
sys.path.insert(0, str(APP))
import calibration_guide as guide
import launcher
import demo
from guided_plan import PoseCapture


class Capture(io.StringIO):
    def reconfigure(self, **kwargs):
        pass


class GuideTests(unittest.TestCase):
    def test_workspace_waits_for_supported_arm_confirmation_before_finishing(self):
        from config import Config

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            Config().save(root / "paired.json")
            row = {"elapsed_s": "25", "valid": "1", "state": "feedback", "settled_pose": "1"}
            for i, q in enumerate((0, -0.5, -0.7, 0, 0, 0), 1):
                row[f"actual_j{i}_rad"] = str(q)
                row[f"velocity_j{i}_rad_s"] = "0"
            events, child = [], Mock()
            child.poll.return_value = None
            plan = Mock(index=3, collecting=True, complete=True, seconds=10)
            plan.update.return_value = True
            keyboard = Mock()
            keyboard.kbhit.return_value = False
            with (
                patch.object(guide, "ROOT", root),
                patch.dict(sys.modules, {"msvcrt": keyboard, "winsound": Mock()}),
                patch("guided_plan.PoseCapture", return_value=plan),
                patch.object(guide, "latest_row", return_value=row),
                patch.object(
                    guide, "save_capture_metadata", side_effect=lambda *a: events.append("saved")
                ),
                patch("builtins.input", side_effect=lambda *a: events.append("supported")),
                contextlib.redirect_stdout(Capture()),
            ):
                self.assertTrue(
                    guide.guided_loop(child, root / "data.csv", guide.WORKSPACE_TASKS, "workspace")
                )
            self.assertEqual(events, ["saved", "supported"])
            plan.update.assert_called_once_with(25, True)
            child.wait.assert_not_called()

    def test_demo_explicit_recovery_is_disclosed_and_forwarded(self):
        cfg, child = Mock(feedback_mode="extra_load"), Mock()
        child.wait.return_value, child.poll.return_value = 0, 0
        with (
            patch("config.Config.load", return_value=cfg),
            patch("extra_load.load_observer"),
            patch("launcher.single_instance", return_value=contextlib.nullcontext()),
            patch.object(guide, "refresh_leader_reference") as refresh,
            patch.object(demo.subprocess, "Popen", return_value=child) as start,
            patch.object(demo.sys, "platform", "linux"),
            patch.object(demo.sys, "stdout", Capture()) as output,
            patch.object(demo.sys, "stderr", Capture()),
            patch("builtins.input"),
        ):
            self.assertEqual(demo.main(["--recover-shoulder"]), 0)
        refresh.assert_called_once_with(True)
        self.assertIn("--recover-shoulder", start.call_args.args[0])
        self.assertIn("-5", output.getvalue())
        self.assertIn("8", output.getvalue())

    def test_damping_check_routes_to_leader_only_without_pairing_or_load_model(self):
        cfg, child = Mock(feedback_mode="legacy"), Mock()
        child.wait.return_value, child.poll.return_value = 0, 0
        with (
            patch("config.Config.load", return_value=cfg),
            patch("extra_load.load_observer") as observer,
            patch.object(guide, "refresh_leader_reference") as refresh,
            patch.object(demo.subprocess, "Popen", return_value=child) as start,
            patch.object(demo.sys, "platform", "linux"),
            patch.object(demo.sys, "stdout", Capture()),
            patch.object(demo.sys, "stderr", Capture()),
            patch("builtins.input"),
        ):
            self.assertEqual(demo.main(["--check-damping"]), 0)
        observer.assert_not_called()
        refresh.assert_not_called()
        self.assertEqual(start.call_args.args[0][-1], "--check-damping")

    def test_demo_waits_for_readiness_before_pairing_or_starting(self):
        events = []
        cfg = Mock(feedback_mode="extra_load")
        child = Mock()
        child.wait.return_value, child.poll.return_value = 0, 0
        with (
            patch("config.Config.load", return_value=cfg),
            patch("extra_load.load_observer"),
            patch("launcher.single_instance", return_value=contextlib.nullcontext()),
            patch.object(
                guide, "refresh_leader_reference", side_effect=lambda: events.append("reference")
            ),
            patch.object(
                demo.subprocess,
                "Popen",
                side_effect=lambda *a, **kw: events.append("start") or child,
            ),
            patch.object(demo.sys, "platform", "linux"),
            patch.object(demo.sys, "stdout", Capture()),
            patch.object(demo.sys, "stderr", Capture()),
            patch("builtins.input", side_effect=lambda prompt: events.append("ready")),
        ):
            self.assertEqual(demo.main([]), 0)
        self.assertEqual(events, ["ready", "reference", "start"])

    def test_demo_cancellation_before_readiness_opens_no_devices(self):
        with (
            patch("config.Config.load", return_value=Mock(feedback_mode="extra_load")),
            patch("extra_load.load_observer"),
            patch.object(guide, "refresh_leader_reference") as refresh,
            patch.object(demo.subprocess, "Popen") as start,
            patch.object(demo.sys, "stdout", Capture()),
            patch.object(demo.sys, "stderr", Capture()),
            patch("builtins.input", side_effect=KeyboardInterrupt),
        ):
            with self.assertRaises(KeyboardInterrupt):
                demo.main([])
        refresh.assert_not_called()
        start.assert_not_called()

    def test_preparation_time_never_starts_or_finishes_capture(self):
        plan = PoseCapture()
        for now in range(300):
            plan.update(now, True)
        self.assertEqual(plan.seconds, 0)
        self.assertEqual(plan.index, 0)
        self.assertFalse(plan.complete)

    def test_capture_waits_for_operator_and_pauses_when_moving(self):
        plan = PoseCapture(hold_s=2, poses=1)
        plan.begin(100)
        for tick in range(1, 21):
            plan.update(100 + tick / 10, True)
        accumulated = plan.seconds
        for tick in range(21, 51):
            plan.update(100 + tick / 10, False)
        self.assertEqual(plan.seconds, accumulated)
        for tick in range(51, 85):
            plan.update(100 + tick / 10, True)
            if plan.complete:
                break
        self.assertTrue(plan.complete)
        self.assertEqual(len(plan.intervals), 1)

    def test_all_four_poses_require_separate_operator_confirmations(self):
        plan = PoseCapture(hold_s=0.2)
        now = 0.0
        for index in range(4):
            self.assertFalse(plan.collecting)
            self.assertEqual(plan.index, index)
            plan.begin(now)
            for tick in range(30):
                now += 0.1
                if plan.update(now, True):
                    break
            self.assertEqual(plan.index, index + 1)
            now += 30
            plan.update(now, True)
        self.assertTrue(plan.complete)
        self.assertEqual(len(plan.intervals), 4)

    def test_archive_preserves_raw_data_and_bound_metadata(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "train.csv"
            path.write_bytes(b"original data")
            path.with_suffix(".csv.meta.json").write_bytes(b"original metadata")
            with contextlib.redirect_stdout(Capture()):
                guide.archive(path)
            self.assertFalse(path.exists())
            output = next(Path(directory).glob("*.csv"))
            self.assertEqual(output.read_bytes(), b"original data")
            self.assertEqual(
                output.with_suffix(".csv.meta.json").read_bytes(), b"original metadata"
            )

    def test_tail_reads_complete_last_sample_without_loading_whole_csv(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "run.csv"
            path.write_bytes(b"elapsed_s,valid\n" + b"0,1\n" * 4000 + b"15.25,1\n")
            self.assertEqual(float(guide.latest_row(path)["elapsed_s"]), 15.25)

    def test_leader_reference_refresh_preserves_follower_signature_without_enabling(self):
        from test_native_b601 import CANWire, LeaderWire, SerialCANWire
        from config import Config
        from payload import signature

        @contextlib.contextmanager
        def port(wire):
            try:
                yield wire
            finally:
                wire.close()

        can, leader = CANWire(), LeaderWire()
        can.positions = [0.1, -0.1, -0.1, 0.05, 0.08, -0.08, -0.1]
        bridge = SerialCANWire(can)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            cfg = Config()
            cfg.calibration_verified = True
            cfg.joints[0].leader_home_deg = 200
            cfg.save(root / "paired.json")
            payload_cfg = Config.load(root / "paired.json")
            payload_cfg.feedback_mode = "payload"
            payload_cfg.payload_calibration = str(root / "placeholder_profile.json")
            payload_cfg.save(root / "paired_payload.json")
            before = signature(cfg)
            with (
                patch.object(guide, "ROOT", root),
                patch("serial.Serial", side_effect=[port(leader), port(bridge)]),
            ):
                guide.refresh_leader_reference()
            after = Config.load(root / "paired.json")
            self.assertEqual(signature(after), before)
            payload_after = Config.load(root / "paired_payload.json")
            self.assertEqual(signature(payload_after), before)
            self.assertEqual(payload_after.feedback_mode, "payload")
            self.assertEqual(
                [j.leader_home_deg for j in payload_after.joints],
                [j.leader_home_deg for j in after.joints],
            )
            self.assertTrue(
                any(
                    a.leader_home_deg != b.leader_home_deg for a, b in zip(cfg.joints, after.joints)
                )
            )
            self.assertLess(abs(after.joints[0].leader_home_deg), 15)
            self.assertTrue(list((root / "logs/references").glob("paired_*.json")))
        self.assertTrue(all(can_id == 0x7FF for can_id, data in can.sent))
        self.assertTrue(all(frame[2] == 25 for frame in leader.sent))
        self.assertEqual(can.enabled, set())

    def run_wrapper(self, interrupted):
        from config import Config

        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "data").mkdir()
            Config().save(root / "paired.json")
            child = Mock(returncode=0)
            child.poll.return_value = None

            def spawn(*args, **kwargs):
                (root / "data/train.csv").write_text(
                    "elapsed_s,valid\n" + ("1,1\n" if interrupted else "59.9,1\n")
                )
                return child

            with (
                patch.object(guide, "ROOT", root),
                patch.object(guide.ctypes, "windll", Mock(), create=True),
                patch.object(launcher, "single_instance", lambda cfg: contextlib.nullcontext()),
                patch.object(guide, "refresh_leader_reference"),
                patch.object(
                    guide,
                    "guided_loop",
                    side_effect=KeyboardInterrupt if interrupted else None,
                    return_value=True,
                ),
                patch.object(guide.subprocess, "Popen", side_effect=spawn),
                patch.object(guide.time, "sleep"),
                patch("builtins.input", return_value=""),
                contextlib.redirect_stdout(Capture()) as output,
            ):
                code = guide.main(["--ready"])
            if interrupted:
                self.assertTrue((root / "stop.request").exists())
                child.wait.assert_called_once()
                self.assertIn("提前停止", output.getvalue())
            else:
                self.assertIn("采样结束", output.getvalue())
            self.assertEqual(code, 0)

    def test_complete_collection_waits_for_child_cleanup(self):
        self.run_wrapper(False)

    def test_ctrl_c_requests_child_shutdown_and_does_not_claim_complete(self):
        self.run_wrapper(True)


if __name__ == "__main__":
    unittest.main(verbosity=2)
