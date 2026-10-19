import contextlib
import csv
import tempfile
import unittest
import sys
from pathlib import Path

from bus import LinkError
from config import Config
from test_native_b601 import CANWire, Clock, LeaderWire, SerialCANWire

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "windows"))
from damping_check import run_check


class DampingCheckTests(unittest.TestCase):
    def wires(self, enabled=False):
        can, leader, clock = CANWire(), LeaderWire(), Clock()
        if enabled:
            can.enabled.add(4)
        bridge = SerialCANWire(can)
        ports = iter((bridge, leader))

        @contextlib.contextmanager
        def factory(*args, **kwargs):
            wire = next(ports)
            try:
                yield wire
            finally:
                wire.close()

        return can, leader, bridge, clock, factory

    def test_check_uses_real_damping_frames_and_never_enables_or_moves_follower(self):
        can, leader, bridge, clock, factory = self.wires()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "check.csv"
            run_check(Config(), output, factory, clock, clock.sleep)
            with output.open(newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertEqual({row["phase"] for row in rows}, {"free", "damping"})
            self.assertTrue(all(row["valid"] == "1" and None not in row for row in rows))
        damping = [raw for raw in leader.sent if raw[2] == 9]
        self.assertTrue(damping)
        self.assertEqual({raw[4] for raw in damping}, {1, 2})
        self.assertEqual({int.from_bytes(raw[5:7], "little") for raw in damping}, {1, 1000})
        self.assertTrue(
            all(can_id == 0x7FF and payload[2] in (0x33, 0xCC) for can_id, payload in can.sent)
        )
        self.assertEqual(can.enabled, set())
        self.assertTrue(all(raw[2] == 24 and raw[5] == 0x10 for raw in leader.sent[-7:]))
        self.assertTrue(leader.closed and bridge.closed)

    def test_enabled_follower_blocks_check_before_leader_commands(self):
        can, leader, bridge, clock, factory = self.wires(enabled=True)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(LinkError, "follower motors disabled"):
                run_check(Config(), Path(directory) / "check.csv", factory, clock, clock.sleep)
        self.assertEqual(leader.sent, [])
        self.assertEqual(can.enabled, {4})
        self.assertTrue(bridge.closed)

    def test_stop_during_damping_releases_all_leader_joints(self):
        can, leader, bridge, clock, factory = self.wires()

        def sleep(seconds):
            clock.sleep(seconds)
            if clock() > 7:
                raise KeyboardInterrupt

        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaises(KeyboardInterrupt):
                run_check(Config(), Path(directory) / "check.csv", factory, clock, sleep)
        self.assertTrue(
            any(raw[2] == 9 and int.from_bytes(raw[5:7], "little") == 1000 for raw in leader.sent)
        )
        self.assertTrue(all(raw[2] == 24 and raw[5] == 0x10 for raw in leader.sent[-7:]))
        self.assertEqual(can.enabled, set())
        self.assertTrue(leader.closed and bridge.closed)

    def test_missing_leader_feedback_releases_output(self):
        can, leader, bridge, clock, factory = self.wires()
        leader.omit.add(2)
        with tempfile.TemporaryDirectory() as directory:
            with self.assertRaisesRegex(LinkError, "Leader feedback timeout"):
                run_check(Config(), Path(directory) / "check.csv", factory, clock, clock.sleep)
        self.assertTrue(all(raw[2] == 24 and raw[5] == 0x10 for raw in leader.sent[-7:]))
        self.assertEqual(can.enabled, set())
