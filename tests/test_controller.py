import contextlib
import importlib.util
import io
import os
import subprocess
import sys
import struct
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

spec = importlib.util.spec_from_file_location(
    "controller", Path(__file__).resolve().parents[1] / "stararm102_ro.py"
)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


class Manager:
    def __init__(self, angles=None, currents=None):
        self.angles = angles if angles is not None else dict.fromkeys(m.servo_ids, 10.0)
        self.currents = currents if currents is not None else dict.fromkeys(m.servo_ids, 500.0)
        self.servos = {
            i: SimpleNamespace(angle_monitor=a, current=self.currents.get(i))
            for i, a in self.angles.items()
        }
        self.calls = []
        self.fail = False

    def send_sync_servo_monitor(self, ids):
        if self.fail:
            raise OSError("disconnected")
        for i in ids:
            if i in self.angles:
                self.servos.setdefault(i, SimpleNamespace()).angle_monitor = self.angles[i]
                self.servos[i].current = self.currents.get(i)
        return self.servos

    def stop_on_control_mode(self, *args):
        self.calls.append(("stop", *args))

    def set_damping(self, *args):
        self.calls.append(("damping", *args))

    def send_sync_multiturnanglebyinterval(self, size, count, data):
        self.calls.append(("hold", [struct.unpack("<BlLHHH", row) for row in data]))


class ControlTests(unittest.TestCase):
    def console(self, feedback=True, guard=True):
        c = m.DampingConsole(
            m.servo_ids,
            feedback=m.LoadFeedback(m.servo_ids, enabled=feedback),
            stall=m.StallGuard(m.servo_ids, enabled=guard),
        )
        c.emit = lambda message: None
        return c

    def step(self, g, t, diff=30, ld=30, fl=0, ids=(0,)):
        return g.update(
            {i: diff for i in ids},
            {i: ld for i in ids},
            {i: fl for i in ids},
            {i: 1 for i in ids},
            now=t,
        )

    def test_stalled_locks_after_window(self):
        g = m.StallGuard([0])
        self.assertEqual(self.step(g, 0), ([], []))
        self.assertEqual(self.step(g, 0.41), ([0], []))

    def test_same_direction_lag_does_not_lock(self):
        g = m.StallGuard([0])
        self.step(g, 0)
        self.assertEqual(self.step(g, 0.41, ld=40, fl=5, diff=35), ([], []))
        self.assertFalse(g.locked)

    def test_reverse_slip_is_not_following(self):
        g = m.StallGuard([0])
        self.step(g, 0)
        self.assertEqual(self.step(g, 0.41, ld=40, fl=-5, diff=45), ([0], []))

    def test_release_requires_stable_recovery(self):
        g = m.StallGuard([0])
        self.step(g, 0)
        self.step(g, 0.41)
        self.assertEqual(self.step(g, 0.5, diff=5), ([], []))
        self.assertEqual(self.step(g, 0.7, diff=9), ([], []))
        self.assertEqual(self.step(g, 0.8, diff=5), ([], []))
        self.assertEqual(self.step(g, 1.11, diff=5), ([], [0]))

    def test_missing_sample_resets_recovery_timer(self):
        g = m.StallGuard([0])
        self.step(g, 0)
        self.step(g, 0.41)
        self.step(g, 0.5, diff=5)
        g.update({}, now=1)
        self.assertEqual(self.step(g, 2, diff=5), ([], []))
        self.assertEqual(self.step(g, 2.31, diff=5), ([], [0]))

    def test_stalled_scope_adds_new_joints(self):
        g = m.StallGuard([0, 1], lock_scope="stalled")
        self.step(g, 0)
        self.step(g, 0.41)
        self.step(g, 0.5, ids=(1,))
        self.assertEqual(self.step(g, 0.91, ids=(1,)), ([1], []))
        self.assertEqual(g.locked, {0, 1})

    def test_l_registers_manual_lock_and_retries_when_guard_disabled(self):
        c = self.console(guard=False)
        leader = Manager()
        c._dispatch("l")
        c.drain(leader)
        self.assertEqual(c.stall.locked, set(m.servo_ids[:-1]))
        self.assertFalse(c.feedback.enabled)
        with patch.object(m.time, "monotonic", return_value=1e12):
            c.refresh(leader)
        self.assertEqual(sum(x[0] == "hold" for x in leader.calls), 2)

    def test_manual_lock_survives_low_error(self):
        c = self.console()
        leader = Manager()
        c._dispatch("st lock")
        c.drain(leader)
        for t in (0, 1, 2):
            self.assertEqual(c.stall.update(dict.fromkeys(m.servo_ids, 0), now=t), ([], []))
        self.assertEqual(c.stall.locked, set(m.servo_ids))

    def test_feedback_cannot_overwrite_manual_lock(self):
        c = self.console()
        leader = Manager()
        follower = Manager()
        c._dispatch("st lock")
        c.drain(leader)
        leader.calls.clear()
        c.follower_tick(leader, [follower], [10] * 7)
        self.assertFalse(any(call[0] == "damping" for call in leader.calls))

    def test_fb_off_keeps_explicit_lock(self):
        c = self.console()
        leader = Manager()
        c._dispatch("st lock")
        c._dispatch("fb off")
        c.drain(leader)
        self.assertFalse(
            any(call[0] == "stop" and call[2] == m.METHOD_RELEASE for call in leader.calls)
        )
        self.assertEqual(c.stall.locked, set(m.servo_ids))

    def test_relock_independent_of_monitor_failure(self):
        c = self.console()
        leader = Manager()
        follower = Manager()
        c._dispatch("st lock")
        c.drain(leader)
        follower.fail = True
        c.follower_tick(leader, [follower], [10] * 7)
        with patch.object(m.time, "monotonic", return_value=1e12):
            c.refresh(leader)
        self.assertEqual(sum(x[0] == "hold" for x in leader.calls), 2)

    def test_relock_retains_original_position(self):
        c = self.console()
        leader = Manager()
        c._dispatch("st lock")
        c.drain(leader)
        first = next(x[1] for x in leader.calls if x[0] == "hold")
        for info in leader.servos.values():
            info.angle_monitor = 99
        with patch.object(m.time, "monotonic", return_value=1e12):
            c.refresh(leader)
        self.assertEqual([x[1] for x in leader.calls if x[0] == "hold"][-1], first)

    def test_old_sdk_cache_is_rejected(self):
        leader = Manager()
        leader.angles = {}
        leader.currents = {}
        found, missing = m._read_follower_monitors([leader], m.servo_ids, [0] * 7)
        self.assertEqual(found, {})
        self.assertEqual(missing, m.servo_ids)

    def test_multiple_followers_use_largest_error(self):
        a = Manager({0: 10}, {0: 200})
        b = Manager({0: -40}, {0: 900})
        found, missing = m._read_follower_monitors([a, b], [0], [0])
        self.assertEqual(found[0], {"angle": -40, "current": 900})
        self.assertEqual(missing, [])

    def test_missing_follower_prevents_releasing_lock(self):
        a = Manager({0: 10}, {0: 500})
        b = Manager({0: 10}, {0: 500})
        b.angles = {}
        found, missing = m._read_follower_monitors([a, b], [0], [10])
        self.assertEqual(missing, [0])

    def test_commands_are_deferred_and_ordered(self):
        c = self.console()
        leader = Manager()
        c._dispatch("l")
        c._dispatch("n")
        self.assertTrue(c.feedback.enabled)
        self.assertFalse(c.stall.locked)
        c.drain(leader)
        self.assertFalse(c.stall.locked)
        self.assertTrue(any(x[0] == "hold" for x in leader.calls))
        self.assertEqual(leader.calls[-1], ("stop", 6, m.METHOD_RELEASE, 0))

    def test_fbmax_seven_values(self):
        c = self.console()
        c.handle_line("fbmax 100,200,300,400,500,600,700")
        self.assertEqual(c.feedback.current_max[6], 700)

    def test_nonfinite_values_rejected(self):
        for text in ("nan", "inf", "-inf", "1e500"):
            self.assertIsNone(m._parse_float(text))
            self.assertIsNone(m._parse_int(text))

    def test_redirected_output_does_not_print_every_second(self):
        c = self.console()
        output = io.StringIO()
        c._out = output
        with contextlib.redirect_stdout(output):
            c.set_frequency("200Hz")
        self.assertEqual(output.getvalue(), "")

    @unittest.skipUnless(os.name == "nt", "Windows legacy output encoding")
    def test_windows_help_works_with_legacy_output_encoding(self):
        env = dict(os.environ, PYTHONIOENCODING="cp1252", PYTHONUTF8="0")
        result = subprocess.run(
            [sys.executable, str(Path(m.__file__)), "--help"],
            env=env,
            capture_output=True,
            check=False,
        )
        self.assertEqual(result.returncode, 0, result.stderr.decode("utf-8", errors="replace"))
        self.assertIn("主从控制", result.stdout.decode("utf-8"))

    def test_threshold_change_preserves_hysteresis(self):
        g = m.StallGuard([0])
        g.set_thresholds(lock_deg=5)
        self.assertLessEqual(g.release_deg, g.lock_deg)

    def test_input_queue_does_not_drop_accepted_lock(self):
        c = self.console()
        c._dispatch("st lock")
        for _ in range(c.MAX_QUEUE + 5):
            c._dispatch("s")
        self.assertEqual(c._inputs[0], "st lock")

    def run_main_once(self, leader, follower):
        import ast
        from unittest.mock import Mock

        tree = ast.parse(Path(m.__file__).read_text(encoding="utf-8"))
        block = next(
            n
            for n in tree.body
            if isinstance(n, ast.If) and ast.unparse(n.test) == "__name__ == '__main__'"
        )
        with patch.object(m.sys, "argv", ["controller", "--no_console"]):
            exec(
                compile(ast.Module(body=block.body[:-1], type_ignores=[]), m.__file__, "exec"),
                m.__dict__,
            )
        leader.reset_multi_turn_angle = Mock()
        follower.reset_multi_turn_angle = Mock()
        uarts = [SimpleNamespace(close=Mock()), SimpleNamespace(close=Mock())]
        with patch.object(m.serial, "Serial", side_effect=uarts), patch.object(
            m.uservo, "UartServoManager", side_effect=[leader, follower]
        ), patch.object(m.time, "sleep", side_effect=KeyboardInterrupt), contextlib.redirect_stdout(
            io.StringIO()
        ):
            m.main(m.args)
        self.assertTrue(all(u.close.called for u in uarts))
        self.assertEqual(leader.calls[-1], ("stop", 255, m.METHOD_RELEASE, 0))

    def test_main_updates_followers_and_closes_ports(self):
        leader, follower = Manager(), Manager()
        self.run_main_once(leader, follower)
        self.assertTrue(any(c[0] == "hold" for c in follower.calls))

    def test_main_does_not_forward_stale_leader_angles(self):
        leader, follower = Manager(), Manager()
        leader.angles = {}
        self.run_main_once(leader, follower)
        self.assertFalse(any(c[0] == "hold" for c in follower.calls))


if __name__ == "__main__":
    unittest.main(verbosity=2)
