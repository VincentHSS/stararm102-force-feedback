import contextlib
import copy
import importlib
import io
import json
import math
import sys
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "integrations/lerobot_auto_feedback"))
from stararm_auto_feedback.controller import AutomaticFeedback, NAMES, settings
from stararm_auto_feedback.install import install, remove, patch_source


class Bus:
    def __init__(self):
        self.calls = []
        self.reliable = True

    def sync_monitor(self, ids):
        return {i: SimpleNamespace(reliable=self.reliable, angle_deg=0.0) for i in ids}

    def set_stop_mode(self, servo_id, *, mode, power):
        self.calls.append((servo_id, mode, power))

    def unlock(self, servo_id):
        self.calls.append((servo_id, 0x10, 0))


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


SOURCE = '''def teleop_loop(teleop, robot, fps=30):
    """Official-style teleoperation flow."""
    for _tick in range(2):
        obs = robot.get_observation()
        action = teleop.get_action()
        _ = robot.send_action(action)
    return "done"
'''


class AutoFeedbackTests(unittest.TestCase):
    def pair(self, builtin=False):
        bus = Bus()
        leader = SimpleNamespace(
            name="rebot_102_leader" if builtin else "rebot_arm_102_leader",
            bus=bus,
            config=SimpleNamespace(joint_ids=dict(zip(NAMES, range(7)))),
        )
        directions = dict(zip(NAMES, (-1, -1, 1, 1, 1, -1, -6)))
        if builtin:
            leader.config.joint_directions = directions
        states = {n: SimpleNamespace(pos=0.0, torq=0.02, status_code=1) for n in NAMES}
        motors = {n: SimpleNamespace(get_state=lambda n=n: states[n]) for n in NAMES}
        robot = SimpleNamespace(
            name="rebot_b601_follower" if builtin else "seeed_b601_dm_follower",
            config=SimpleNamespace(joint_directions=directions, motor_family="dm"),
            profile=SimpleNamespace(joint_directions=dict.fromkeys(NAMES, 1.0)),
            motors=motors,
        )
        clock = Clock()
        controller = AutomaticFeedback(leader, robot, config=settings(), clock=clock)
        return controller, leader, robot, states, clock

    def update(self, c, clock, t, action=None):
        clock.now = t
        c.bus.sync_monitor(list(c.ids.values()))
        c.update(dict.fromkeys((n + ".pos" for n in NAMES), 0.0) if action is None else action)

    def test_initial_feedback_is_automatic_and_uses_existing_bus(self):
        c, l, _, _, clock = self.pair()
        self.update(c, clock, 0)
        self.assertTrue(c.active)
        self.assertIs(c.bus, l.bus)
        self.assertEqual(len(c.bus.calls), 7)
        self.assertTrue(all(mode == 0x12 and power == 1 for _, mode, power in c.bus.calls))

    def test_no_terminal_input_or_follower_commands(self):
        c, _, robot, _, clock = self.pair()
        robot.send_action = lambda *a: self.fail("addon must not command follower")
        with patch("builtins.input", side_effect=AssertionError("manual input")):
            self.update(c, clock, 0)

    def test_torque_load_changes_damping_with_deadband(self):
        c, _, _, states, clock = self.pair()
        self.update(c, clock, 0)
        states[NAMES[0]].torq = 5
        self.update(c, clock, 0.1)
        self.assertEqual(c.bus.calls[-1], (0, 0x12, 1200))
        count = len(c.bus.calls)
        self.update(c, clock, 0.2)
        self.assertEqual(len(c.bus.calls), count)

    def test_action_dictionary_is_never_changed(self):
        c, _, _, _, clock = self.pair()
        action = {n + ".pos": i * 5 for i, n in enumerate(NAMES)}
        original = copy.deepcopy(action)
        self.update(c, clock, 0, action)
        self.assertEqual(action, original)

    def test_external_plugin_direction_and_gripper_scale(self):
        c, _, _, states, _ = self.pair()
        states["gripper"].pos = math.radians(-60)
        c.bus.sync_monitor(list(c.ids.values()))
        sent = dict.fromkeys((n + ".pos" for n in NAMES), 0.0)
        sent["gripper.pos"] = -120
        target, actual, _ = c._sample(sent)
        self.assertAlmostEqual(target["gripper"], 20)
        self.assertAlmostEqual(actual["gripper"], 10)

    def test_builtin_follower_and_leader_coordinate_scales(self):
        c, _, robot, states, _ = self.pair(True)
        robot.profile.joint_directions["gripper"] = -2
        states["gripper"].pos = math.radians(120)
        c.bus.sync_monitor(list(c.ids.values()))
        sent = dict.fromkeys((n + ".pos" for n in NAMES), 0.0)
        sent["gripper.pos"] = -120
        target, actual, _ = c._sample(sent)
        self.assertAlmostEqual(target["gripper"], 20)
        self.assertAlmostEqual(actual["gripper"], 10)

    def test_stall_automatically_locks_after_window(self):
        c, _, _, states, clock = self.pair()
        states["shoulder_pan"].pos = math.radians(30)
        states["shoulder_pan"].torq = 5
        self.update(c, clock, 0)
        self.update(c, clock, 0.41)
        self.assertTrue(c.locked)
        self.assertEqual(c.triggers, {"shoulder_pan"})
        self.assertTrue(all(mode == 0x11 for _, mode, _ in c.bus.calls[-7:]))

    def test_reverse_motion_not_classified_as_following(self):
        c, _, _, _, _ = self.pair()
        target = dict.fromkeys(NAMES, 30.0)
        actual = dict.fromkeys(NAMES, 0.0)
        loads = dict.fromkeys(NAMES, 1.0)
        c._observe(target, actual, loads, 0)
        target = dict.fromkeys(NAMES, 40.0)
        actual = dict.fromkeys(NAMES, -5.0)
        c._observe(target, actual, loads, 0.41)
        self.assertEqual(c.triggers, set(NAMES))

    def test_same_direction_tracking_does_not_lock(self):
        c, _, _, _, _ = self.pair()
        c._observe(
            dict.fromkeys(NAMES, 30.0), dict.fromkeys(NAMES, 0.0), dict.fromkeys(NAMES, 1.0), 0
        )
        c._observe(
            dict.fromkeys(NAMES, 40.0), dict.fromkeys(NAMES, 5.0), dict.fromkeys(NAMES, 1.0), 0.41
        )
        self.assertFalse(c.triggers)

    def test_lock_maintenance_never_overwritten_by_damping(self):
        c, _, _, states, clock = self.pair()
        states["shoulder_pan"].pos = math.radians(30)
        states["shoulder_pan"].torq = 5
        self.update(c, clock, 0)
        self.update(c, clock, 0.41)
        c.bus.calls.clear()
        self.update(c, clock, 1.5)
        self.assertEqual(len(c.bus.calls), 7)
        self.assertTrue(all(mode == 0x11 for _, mode, _ in c.bus.calls))

    def test_auto_release_requires_sustained_error_recovery(self):
        c, _, _, states, clock = self.pair()
        states["shoulder_pan"].pos = math.radians(30)
        states["shoulder_pan"].torq = 5
        self.update(c, clock, 0)
        self.update(c, clock, 0.41)
        states["shoulder_pan"].pos = 0.0
        self.update(c, clock, 0.5)
        self.assertTrue(c.locked)
        self.update(c, clock, 0.81)
        self.assertFalse(c.locked)
        self.assertTrue(all(mode == 0x12 for _, mode, _ in c.bus.calls[-7:]))

    def test_unreliable_leader_read_is_held_without_changing_result(self):
        c, _, _, _, clock = self.pair()
        c.bus.reliable = False
        result = c.bus.sync_monitor(list(c.ids.values()))
        self.assertFalse(result[0].reliable)
        c.update(dict.fromkeys((n + ".pos" for n in NAMES), 0.0))
        self.assertTrue(c.locked and c.fault)

    def test_missing_follower_state_holds_leader(self):
        c, _, _, states, clock = self.pair()
        states["gripper"] = None
        self.update(c, clock, 0)
        self.assertTrue(c.locked and c.fault)

    def test_follower_sdk_exception_holds_leader(self):
        c, _, robot, _, clock = self.pair()

        def failed_read():
            raise RuntimeError("native feedback error")

        robot.motors["gripper"].get_state = failed_read
        self.update(c, clock, 0)
        self.assertTrue(c.locked and c.fault)

    def test_disabled_follower_is_not_normal_feedback(self):
        c, _, _, states, clock = self.pair()
        states["gripper"].status_code = 0
        self.update(c, clock, 0)
        self.assertTrue(c.fault)

    def test_fault_does_not_release_while_error_is_large(self):
        c, _, _, states, clock = self.pair()
        c.bus.reliable = False
        self.update(c, clock, 0)
        c.bus.reliable = True
        states["shoulder_pan"].pos = math.radians(30)
        self.update(c, clock, 0.5)
        self.update(c, clock, 1.0)
        self.assertTrue(c.locked and c.fault)

    def test_fault_recovers_after_good_feedback_and_small_error(self):
        c, _, _, _, clock = self.pair()
        c.bus.reliable = False
        self.update(c, clock, 0)
        c.bus.reliable = True
        self.update(c, clock, 0.1)
        self.update(c, clock, 0.41)
        self.assertFalse(c.locked or c.fault)

    def test_close_restores_monitor_and_releases_owned_joints_once(self):
        c, l, _, _, clock = self.pair()
        self.update(c, clock, 0)
        c.close()
        c.close()
        self.assertEqual(sum(mode == 0x10 for _, mode, _ in c.bus.calls), 7)
        self.assertEqual(l.bus.sync_monitor, c._original_monitor)

    def test_other_robot_types_are_not_modified(self):
        bus = Bus()
        original = bus.sync_monitor
        c = AutomaticFeedback(SimpleNamespace(name="other", bus=bus), object(), config=settings())
        c.update({})
        c.close()
        self.assertFalse(c.active)
        self.assertEqual(bus.sync_monitor, original)
        self.assertEqual(bus.calls, [])

    def test_rs_variant_is_not_modified(self):
        c, l, r, _, _ = self.pair(True)
        c.close()
        r.config.motor_family = "rs"
        other = AutomaticFeedback(l, r, config=settings())
        self.assertFalse(other.active)

    def test_invalid_config_rejected(self):
        for values in (
            {"unknown": 1},
            {"power_min_mw": 0},
            {"release_deg": 30},
            {"sample_hz": float("nan")},
            {"confirm_windows": 1.2},
        ):
            with self.subTest(values=values), tempfile.TemporaryDirectory() as folder:
                p = Path(folder) / "cfg.json"
                p.write_text(json.dumps(values))
                with self.assertRaises(ValueError):
                    settings(p)

    def test_sdk_without_stop_api_is_rejected(self):
        c, l, r, _, _ = self.pair()
        c.close()
        l.bus.set_stop_mode = None
        with self.assertRaises(TypeError):
            AutomaticFeedback(l, r, config=settings())

    def test_hook_runs_original_actions_and_cleanup(self):
        namespace = {}
        exec(patch_source(SOURCE), namespace)
        c, l, r, states, clock = self.pair()
        c.close()
        l.get_action = lambda: {n + ".pos": 0.0 for n in NAMES}
        r.get_observation = lambda: {}
        calls = []
        r.send_action = lambda action: calls.append(copy.deepcopy(action)) or action
        with patch("stararm_auto_feedback.AutomaticFeedback") as factory:
            factory.return_value = SimpleNamespace(
                update=lambda action: None, close=lambda: calls.append("closed")
            )
            result = namespace["teleop_loop"](l, r)
        self.assertEqual(result, "done")
        self.assertEqual(calls[:2], [dict.fromkeys((n + ".pos" for n in NAMES), 0.0)] * 2)
        self.assertEqual(calls[-1], "closed")

    def test_hook_cleanup_on_official_loop_exception(self):
        namespace = {}
        exec(patch_source(SOURCE), namespace)
        closed = []
        l = SimpleNamespace(get_action=lambda: {})

        def fail():
            raise RuntimeError("official camera error")

        r = SimpleNamespace(get_observation=fail)
        with patch("stararm_auto_feedback.AutomaticFeedback") as factory:
            factory.return_value = SimpleNamespace(
                update=lambda action: None, close=lambda: closed.append(True)
            )
            with self.assertRaises(RuntimeError):
                namespace["teleop_loop"](l, r)
        self.assertEqual(closed, [True])

    def test_hook_handles_standalone_send_statement(self):
        src = SOURCE.replace("_ = robot.send_action(action)", "robot.send_action(action)")
        self.assertIn("_stararm_sent_action = robot.send_action(action)", patch_source(src))

    def test_installer_is_idempotent_and_restores_exact_bytes(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / "lerobot_teleoperate.py"
            original = SOURCE.replace("\n", "\r\n").encode()
            p.write_bytes(original)
            self.assertTrue(install(p))
            self.assertFalse(install(p))
            remove(p)
            self.assertEqual(p.read_bytes(), original)

    def test_installer_refuses_to_overwrite_user_changes(self):
        with tempfile.TemporaryDirectory() as folder:
            p = Path(folder) / "lerobot_teleoperate.py"
            p.write_text(SOURCE)
            install(p)
            p.write_text(p.read_text() + "\n# user modification\n")
            with self.assertRaises(ValueError):
                remove(p)

    def test_unsupported_official_source_is_not_patched(self):
        for source in (
            "def other(): pass\n",
            SOURCE.replace("robot.send_action(action)", "robot.other(action)"),
        ):
            with self.assertRaises(ValueError):
                patch_source(source)


if __name__ == "__main__":
    unittest.main()
