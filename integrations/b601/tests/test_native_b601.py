import copy
import csv
import math
import struct
import subprocess
import sys
import tempfile
import unittest
from dataclasses import replace
from pathlib import Path
from unittest.mock import Mock

from bus import Follower, Leader, LinkError, SerialCAN, State
from config import Config
from controller import Feedback, Motion, Session
from teleop import SimFollower, SimLeader, hardware, main, simulate


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, delay):
        self.now += max(delay, 0.00001)


def leader_frame(servo_id, angle=0, *, invalid=False):
    body = struct.pack(
        "<BHHHHBih", servo_id, 7400, 0, 0, 2000, 0, -235929599 if invalid else round(angle * 10), 0
    )
    frame = b"\x05\x1c\x16\x10" + body
    return frame + bytes((sum(frame) & 255,))


class LeaderWire:
    def __init__(self):
        self.buffer = bytearray()
        self.angles = [0.0] * 8
        self.omit = set()
        self.corrupt = False
        self.partial = None
        self.closed = False
        self.sent = []

    @property
    def in_waiting(self):
        return len(self.buffer)

    def reset_input_buffer(self):
        self.buffer.clear()

    def read(self, size):
        size = min(size, self.partial or size)
        data = bytes(self.buffer[:size])
        del self.buffer[:size]
        return data

    def write(self, raw):
        self.sent.append(bytes(raw))
        if raw[2] == 25:
            for servo_id in raw[7:-1]:
                if servo_id in self.omit:
                    continue
                frame = leader_frame(servo_id, self.angles[servo_id])
                if self.corrupt:
                    frame = frame[:-1] + bytes((frame[-1] ^ 1,))
                self.buffer.extend(frame)
        return len(raw)

    def close(self):
        self.closed = True


def motor_frame(i, *, position=0, torque=0, status=1):
    pmax, vmax, tmax = (12.5, 10, 28) if i < 3 else (12.5, 30, 10)
    p = round((position + pmax) / (2 * pmax) * 65535)
    v = round(4095 / 2)
    t = round((torque + tmax) / (2 * tmax) * 4095)
    return bytes(
        (
            (status << 4) | (i + 1),
            p >> 8,
            p & 255,
            v >> 4,
            ((v & 15) << 4) | (t >> 8),
            t & 255,
            25,
            25,
        )
    )


class CANWire:
    def __init__(self):
        self.sent, self.rx = [], []
        self.enabled = set()
        self.positions = [0.0] * 7
        self.omit = set()
        self.mode_override = None
        self.invalid_limits = False
        self.fail_enable = None
        self.closed = False

    def send(self, can_id, data):
        self.sent.append((can_id, data))
        if data == b"\xff" * 7 + b"\xfc":
            if can_id == self.fail_enable:
                raise OSError("enable failed")
            self.enabled.add(can_id)
        elif data == b"\xff" * 7 + b"\xfd":
            self.enabled.discard(can_id)
        elif can_id == 0x7FF:
            motor_id = int.from_bytes(data[:2], "little")
            i = motor_id - 1
            if data[2] == 0x33:
                rid = data[3]
                if rid == 10:
                    value = struct.pack("<I", self.mode_override or (4 if i == 6 else 2))
                else:
                    defaults = (12.5, 10, 28) if i < 3 else (12.5, 30, 10)
                    value = struct.pack(
                        "<f", math.nan if self.invalid_limits else defaults[rid - 21]
                    )
                self.rx.append((i + 17, bytes((motor_id, 0, 0x33, rid)) + value))
            elif data[2] == 0xCC and i not in self.omit:
                self.rx.append(
                    (
                        i + 17,
                        motor_frame(
                            i, position=self.positions[i], status=int(motor_id in self.enabled)
                        ),
                    )
                )
        elif can_id & 0xF00 in (0x100, 0x300):
            self.positions[(can_id & 255) - 1] = struct.unpack_from("<f", data)[0]

    def poll(self):
        result, self.rx = self.rx, []
        return result

    def close(self):
        self.closed = True


class SerialCANWire:
    def __init__(self, can):
        self.can = can
        self.buffer = bytearray()
        self.closed = False

    @property
    def in_waiting(self):
        for can_id, data in self.can.poll():
            self.buffer.extend(b"\xaa\x11\x08" + struct.pack("<I", can_id) + data + b"\x55")
        return len(self.buffer)

    def read(self, size):
        result = bytes(self.buffer[:size])
        del self.buffer[:size]
        return result

    def write(self, raw):
        self.can.send(struct.unpack_from("<I", raw, 13)[0], bytes(raw[21:29]))
        return len(raw)

    def close(self):
        self.closed = True


class NativeB601Tests(unittest.TestCase):
    def setUp(self):
        self.cfg = Config().validate()
        self.clock = Clock()
        self.leader_wire = LeaderWire()
        self.leader = Leader(self.leader_wire, self.cfg, self.clock, self.clock.sleep)
        self.can = CANWire()
        self.follower = Follower(self.can, self.cfg, self.clock, self.clock.sleep)

    def states(self, now, *, seq=1, torque=0, positions=None):
        positions = positions or [0.0] * 7
        return [State(p, 0, torque, 1, 25, 25, now, seq + i) for i, p in enumerate(positions)]

    def feedback(self):
        return Feedback(self.cfg, SimLeader())

    def test_leader_persistent_zero_remains_valid_for_many_frames(self):
        for _ in range(200):
            self.assertEqual(self.leader.read(), ([0.0] * 7, False))
            self.clock.sleep(0.01)

    def test_leader_zero_after_small_nonzero_transition(self):
        self.leader_wire.angles[0] = 1.2
        self.assertEqual(self.leader.read()[0][0], 1.2)
        self.clock.sleep(0.01)
        self.leader_wire.angles[0] = 0
        self.assertEqual(self.leader.read()[0][0], 0)

    def test_missing_zero_joint_does_not_reuse_previous_zero(self):
        self.leader.read()
        self.leader_wire.omit = {3}
        self.assertIsNone(self.leader.read())

    def test_leader_rejects_checksum_corruption(self):
        self.leader_wire.corrupt = True
        self.assertIsNone(self.leader.read())

    def test_leader_invalid_firmware_sentinel_is_not_a_zero_reading(self):
        original = self.leader_wire.write

        def write(raw):
            result = original(raw)
            if raw[2] == 25:
                self.leader_wire.buffer = bytearray(
                    b"".join(leader_frame(i, invalid=i == 0) for i in range(7))
                )
            return result

        self.leader_wire.write = write
        self.assertIsNone(self.leader.read())

    def test_serial_short_write_is_reported(self):
        self.leader_wire.write = lambda _: 0
        with self.assertRaises(LinkError):
            self.leader.mode(0, 0x12, 1)

    def test_leader_handles_fragmented_packets(self):
        self.leader_wire.partial = 3
        self.assertIsNotNone(self.leader.read())

    def test_leader_rejects_jump_then_accepts_return_to_zero(self):
        self.leader.read()
        self.clock.sleep(0.01)
        self.leader_wire.angles[0] = 150
        self.assertIsNone(self.leader.read())
        self.leader_wire.angles[0] = 0
        self.assertIsNotNone(self.leader.read())

    def test_button_is_not_subject_to_joint_jump_filter(self):
        self.cfg.button_id = 7
        self.leader.read()
        self.leader_wire.angles[7] = 180
        self.assertTrue(self.leader.read()[1])

    def test_leader_mode_frame_matches_protocol_checksum(self):
        self.leader.mode(3, 0x12, 1000)
        raw = self.leader_wire.sent[-1]
        self.assertEqual(raw[:4], b"\x12\x4c\x18\x04")
        self.assertEqual(struct.unpack("<BBH", raw[4:-1]), (3, 0x12, 1000))
        self.assertEqual(raw[-1], sum(raw[:-1]) & 255)

    def test_serial_can_transmit_frame_offsets(self):
        wire = Mock()
        wire.write.return_value = 30
        SerialCAN(wire).send(0x123, b"12345678")
        raw = wire.write.call_args.args[0]
        self.assertEqual(raw[:4], b"\x55\xaa\x1e\x03")
        self.assertEqual(struct.unpack_from("<I", raw, 13)[0], 0x123)
        self.assertEqual(raw[18], 8)
        self.assertEqual(raw[21:29], b"12345678")

    def test_serial_can_resynchronizes_noise_and_fragment(self):
        wire = SerialCANWire(self.can)
        transport = SerialCAN(wire)
        wire.buffer.extend(
            b"noise" + b"\xaa\x11\x08" + struct.pack("<I", 17) + motor_frame(0) + b"\x55"
        )
        self.assertEqual(transport.poll(), [(17, motor_frame(0))])

    def test_motor_register_readback_verifies_mode_and_live_scaling(self):
        self.follower.verify()
        self.assertEqual(self.follower.limits[0], (12.5, 10, 28))
        self.assertEqual(self.follower.limits[6], (12.5, 30, 10))
        self.assertEqual(len(self.can.sent), 28)
        self.assertTrue(all(can_id == 0x7FF and data[2] == 0x33 for can_id, data in self.can.sent))

    def test_wrong_mode_never_writes_motor_parameters(self):
        self.can.mode_override = 1
        with self.assertRaises(LinkError):
            self.follower.verify()
        self.assertEqual(len(self.can.sent), 1)
        self.assertEqual(self.can.sent[0][1][2], 0x33)

    def test_invalid_register_scaling_is_rejected(self):
        self.can.invalid_limits = True
        with self.assertRaises(LinkError):
            self.follower.verify()

    def test_follower_identical_new_frames_advance_sequence(self):
        first = self.follower.read()
        second = self.follower.read()
        self.assertTrue(
            all(a.pos == b.pos and b.sequence > a.sequence for a, b in zip(first, second))
        )

    def test_cached_can_feedback_cannot_replace_missing_reply(self):
        self.follower.read()
        self.can.omit = {0}
        self.assertIsNone(self.follower.read())

    def test_follower_jump_is_rejected_and_return_recovers(self):
        self.follower.read()
        self.can.positions[0] = 2.0
        self.assertIsNone(self.follower.read())
        self.can.positions[0] = 0.0
        self.assertIsNotNone(self.follower.read())

    def test_register_frame_does_not_advance_sensor_sequence(self):
        self.can.rx.append((17, b"\x01\x00\x33\x15" + struct.pack("<f", 12.5)))
        self.follower.poll()
        self.assertEqual(self.follower.sequence, 0)

    def test_force_position_gripper_preserves_ratio_and_units(self):
        self.follower.command([0.0] * 6 + [-0.3])
        can_id, data = self.can.sent[-1]
        self.assertEqual(can_id, 0x307)
        pos, speed, effort = struct.unpack("<fHH", data)
        self.assertAlmostEqual(pos, -0.3)
        self.assertEqual(speed, int(math.radians(self.cfg.joints[6].motor_speed) * 100))
        self.assertEqual(effort, 1000)
        self.assertEqual(self.can.sent[0][0], 0x101)

    def test_invalid_command_batch_does_not_partially_move(self):
        with self.assertRaises(ValueError):
            self.follower.command([0.0] * 6 + [math.nan])
        self.assertEqual(self.can.sent, [])

    def test_motor_catchup_speed_does_not_increase_target_trajectory_limit(self):
        self.follower.command([0.0] * 7)
        self.assertAlmostEqual(
            struct.unpack("<ff", self.can.sent[0][1])[1], math.radians(100), places=6
        )
        motion = Motion(self.cfg, [0.0] * 7)
        previous = 0.0
        for _ in range(100):
            target = motion.step([-140.0] + [0.0] * 6, 0.01)[0]
            self.assertLessEqual(abs(target - previous), math.radians(50) * 0.01 + 1e-10)
            previous = target

    def test_legacy_config_without_motor_speed_keeps_existing_speed(self):
        self.cfg.joints[0].motor_speed_deg_s = None
        self.cfg.validate()
        self.follower.command([0.0] * 7)
        self.assertAlmostEqual(
            struct.unpack("<ff", self.can.sent[0][1])[1], math.radians(50), places=6
        )

    def test_motor_speed_lower_than_trajectory_limit_is_rejected(self):
        self.cfg.joints[0].motor_speed_deg_s = 20
        with self.assertRaises(ValueError):
            self.cfg.validate()

    def test_cleanup_disables_all_motors_even_when_one_write_fails(self):
        sent = []

        def send(can_id, data):
            sent.append(can_id)
            if can_id == 1:
                raise OSError("failure")

        self.can.send = send
        with self.assertRaises(LinkError):
            self.follower.disable()
        self.assertEqual(sent, list(range(1, 8)))

    def test_motion_filter_reduces_stationary_noise(self):
        motion = Motion(self.cfg, [0.0] * 7)
        motion.step([0.0] * 7, 0.01)
        filtered = []
        for i in range(100):
            motion.step([0.5 if i % 2 else -0.5] * 7, 0.01)
            filtered.append(motion.filtered[0])
        self.assertLess(max(abs(v) for v in filtered[20:]), 0.2)

    def test_motion_caps_long_gap_and_acceleration(self):
        motion = Motion(self.cfg, [0.0] * 7)
        target = motion.step([-90.0] * 7, 1.0)
        self.assertLessEqual(max(abs(v) for v in target), math.radians(2))
        self.assertLessEqual(max(abs(v) for v in motion.velocity), math.radians(300 * 0.05) + 1e-10)

    def test_motion_reversal_respects_joint_limits(self):
        motion = Motion(self.cfg, [0.0] * 7)
        for tick in range(400):
            targets = motion.step([1000.0 if tick < 200 else -1000.0] * 7, 0.01)
            for j, value in zip(self.cfg.joints, targets):
                self.assertGreaterEqual(value, math.radians(j.lower_deg))
                self.assertLessEqual(value, math.radians(j.upper_deg))

    def test_reversal_does_not_continue_commanding_in_old_direction(self):
        motion = Motion(self.cfg, [0.0] * 7)
        for _ in range(30):
            motion.step([-80.0] + [0.0] * 6, 0.01)
        old = motion.commanded[0]
        # 用已在滤波输出后方的目标直接检查轨迹反转，滤波本身不参与此断言。
        motion.filtered[0] = 0
        motion.input_speed[0] = 0
        command = motion.step([0.0] * 7, 0.01)[0]
        self.assertLessEqual(command, old)

    def test_stationary_goal_does_not_generate_self_oscillating_reference(self):
        motion = Motion(self.cfg, [0.0] * 7)
        samples = []
        for _ in range(1000):
            samples.append(motion.step([-30.0] + [0.0] * 6, 0.01)[0])
        self.assertTrue(all(0 <= x <= math.radians(30) for x in samples))
        self.assertTrue(all(b >= a - 1e-12 for a, b in zip(samples, samples[1:])))
        self.assertAlmostEqual(samples[-1], math.radians(30))

    def test_motion_direction_and_home_roundtrip(self):
        self.cfg.joints[0].leader_home_deg = 3.0
        motion = Motion(self.cfg, [0] * 7)
        values = [j.leader_home_deg for j in self.cfg.joints]
        values[0] -= 10
        self.assertAlmostEqual(motion.forward(values)[0], math.radians(10))
        for a, b in zip(motion.inverse(motion.forward(values)), values):
            self.assertAlmostEqual(a, b)

    def test_hd_freezes_six_targets_but_gripper_can_still_follow(self):
        motion = Motion(self.cfg, [0.0] * 7)
        values = [-30.0] * 6 + [10.0]
        result = motion.step(values, 0.01, held=range(6))
        self.assertEqual(result[:6], [0.0] * 6)
        self.assertLess(result[6], 0)

    def test_boundary_quantization_is_clipped_even_during_hold(self):
        motion = Motion(self.cfg, [0.0002] * 7)
        targets = motion.step([0.0] * 7, 0.01, held=True)
        self.assertEqual(targets[1], 0)
        self.assertEqual(targets[2], 0)
        self.assertEqual(targets[6], 0)
        self.follower.command(targets)

    def test_empty_static_high_background_torque_has_no_drag_or_hold(self):
        fb = self.feedback()
        fb.prime([3.0] * 7)
        for tick in range(300):
            fb.observe([40.0] * 7, [0.0] * 7, [3.0] * 7, tick * 0.01)
            fb.apply(tick * 0.01)
        self.assertEqual(fb.loads, [0.0] * 7)
        self.assertFalse(fb.triggers)
        self.assertTrue(all(mode == 0x12 and power == 1 for _, mode, power in fb.leader.calls))

    def test_slow_normal_following_is_not_stall_even_with_low_ratio(self):
        fb = self.feedback()
        for tick in range(251):
            now = tick * 0.01
            fb.observe([40 + now * 40] * 7, [now * 2] * 7, [5.0] * 7, now)
        self.assertFalse(fb.triggers)

    def test_real_static_stall_needs_sustained_excess_load_and_two_windows(self):
        fb = self.feedback()
        for tick in range(51):
            fb.observe([40.0] * 7, [0.0] * 7, [5.0] * 7, tick * 0.01)
        self.assertFalse(fb.triggers)
        for tick in range(51, 180):
            fb.observe([40.0] * 7, [0.0] * 7, [5.0] * 7, tick * 0.01)
        self.assertTrue(fb.triggers)

    def test_large_error_alone_does_not_trigger(self):
        fb = self.feedback()
        for tick in range(300):
            fb.observe([40.0] * 7, [0.0] * 7, [0.0] * 7, tick * 0.01)
        self.assertFalse(fb.triggers)

    def test_short_load_spike_does_not_trigger(self):
        fb = self.feedback()
        for tick in range(200):
            fb.observe([40.0] * 7, [0.0] * 7, [5.0 if tick < 10 else 0.0] * 7, tick * 0.01)
        self.assertFalse(fb.triggers)

    def test_recovery_needs_stable_error_and_hd_cannot_unlock_stall(self):
        fb = self.feedback()
        fb.triggers.add(0)
        fb.apply(0, manual_hold=True)
        fb.apply(0.1, manual_hold=False)
        self.assertTrue(all(mode == 0x11 for _, mode, power in fb.leader.calls))
        fb.observe([0.0] * 7, [0.0] * 7, [0.0] * 7, 0.2)
        self.assertTrue(fb.triggers)
        fb.observe([0.0] * 7, [0.0] * 7, [0.0] * 7, 0.61)
        self.assertFalse(fb.triggers)
        fb.apply(0.61)
        self.assertEqual(fb.leader.calls[-7:], [(i, 0x12, 1) for i in range(7)])

    def test_interruption_resets_stall_and_release_confirmation(self):
        fb = self.feedback()
        fb.triggers.add(0)
        fb.observe([0.0] * 7, [0.0] * 7, [0.0] * 7, 0)
        fb.interrupt()
        fb.observe([0.0] * 7, [0.0] * 7, [0.0] * 7, 1)
        self.assertTrue(fb.triggers)

    def test_holding_is_periodically_reissued(self):
        fb = self.feedback()
        fb.apply(0, fault=True)
        fb.apply(0.5, fault=True)
        self.assertEqual(len(fb.leader.calls), 7)
        fb.apply(1.01, fault=True)
        self.assertEqual(len(fb.leader.calls), 14)

    def test_no_load_return_always_restores_minimum_power_even_inside_deadband(self):
        fb = self.feedback()
        fb.loads = [0.01] * 7
        fb.apply(0)
        fb.loads = [0.0] * 7
        fb.apply(0.01)
        self.assertEqual(fb.leader.calls[-7:], [(i, 0x12, 1) for i in range(7)])

    def test_reverse_motion_under_load_is_not_treated_as_progress(self):
        fb = self.feedback()
        for tick in range(200):
            fb.observe([40.0] * 7, [-tick * 0.1] * 7, [5.0] * 7, tick * 0.01)
        self.assertTrue(fb.triggers)

    def test_transient_loss_skips_target_without_strong_hold(self):
        initial = self.states(0)
        follower = SimFollower(self.cfg)
        follower.command = Mock()
        session = Session(self.cfg, SimLeader(), follower, initial, 0)
        self.assertFalse(session.tick(None, None, 0.02))
        self.assertFalse(session.fault)
        self.assertTrue(all(mode == 0x12 for _, mode, power in session.feedback.leader.calls))
        follower.command.assert_not_called()
        self.assertTrue(session.tick([0.0] * 7, self.states(0.03, seq=20), 0.03))

    def test_timeout_and_reused_sequence_latch_fault_until_restart(self):
        initial = self.states(0)
        follower = SimFollower(self.cfg)
        follower.command = Mock()
        session = Session(self.cfg, SimLeader(), follower, initial, 0)
        self.assertFalse(session.tick([0.0] * 7, self.states(0.3), 0.3))
        self.assertTrue(session.fault)
        self.assertFalse(session.tick([0.0] * 7, self.states(0.4, seq=100), 0.4))
        follower.command.assert_not_called()

    def test_motor_fault_blocks_command_immediately(self):
        initial = self.states(0)
        follower = SimFollower(self.cfg)
        follower.command = Mock()
        session = Session(self.cfg, SimLeader(), follower, initial, 0)
        states = self.states(0.01, seq=20)
        states[0] = replace(states[0], status=0xA)
        self.assertFalse(session.tick([0.0] * 7, states, 0.01))
        self.assertTrue(session.fault)
        follower.command.assert_not_called()

    def test_invalid_nonfinite_input_is_not_forwarded(self):
        follower = SimFollower(self.cfg)
        session = Session(self.cfg, SimLeader(), follower, self.states(0), 0)
        self.assertFalse(session.tick([math.nan] * 7, self.states(0.01, seq=20), 0.01))

    def test_capture_is_read_only_and_does_not_reset_zero_or_enable(self):
        self.cfg.adapter = "socketcan"
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "paired.json"
            hardware(
                self.cfg,
                capture=output,
                serial_factory=lambda *a, **kw: self.leader_wire,
                socket_factory=lambda _: self.can,
                clock=self.clock,
                sleep=self.clock.sleep,
            )
            paired = Config.load(output)
            self.assertFalse(paired.calibration_verified)
        self.assertTrue(all(frame[2] == 25 for frame in self.leader_wire.sent))
        self.assertTrue(all(can_id == 0x7FF for can_id, data in self.can.sent))
        self.assertTrue(self.can.closed and self.leader_wire.closed)

    def test_preview_never_sends_motion_or_leader_modes(self):
        self.cfg.adapter = "socketcan"
        hardware(
            self.cfg,
            duration=0.06,
            serial_factory=lambda *a, **kw: self.leader_wire,
            socket_factory=lambda _: self.can,
            clock=self.clock,
            sleep=self.clock.sleep,
        )
        self.assertTrue(all(frame[2] == 25 for frame in self.leader_wire.sent))
        self.assertTrue(all(can_id == 0x7FF for can_id, data in self.can.sent))

    def test_execute_end_to_end_serial_bridge_and_diagnostics(self):
        self.cfg.calibration_verified = True
        self.cfg.baseline_s = 0.2
        wire = SerialCANWire(self.can)
        ports = iter((self.leader_wire, wire))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "run.csv"
            session = hardware(
                self.cfg,
                execute=True,
                duration=0.06,
                diagnostics=output,
                serial_factory=lambda *a, **kw: next(ports),
                clock=self.clock,
                sleep=self.clock.sleep,
            )
            with output.open(encoding="utf-8", newline="") as stream:
                rows = list(csv.DictReader(stream))
            self.assertGreater(len(rows), 0)
            self.assertTrue(all(row["valid"] == "1" for row in rows))
        self.assertGreater(session.cycles, 0)
        self.assertEqual(self.can.enabled, set())
        self.assertTrue(self.leader_wire.closed and wire.closed)
        self.assertEqual(sum(data == b"\xff" * 7 + b"\xfc" for _, data in self.can.sent), 7)
        self.assertEqual(sum(data == b"\xff" * 7 + b"\xfd" for _, data in self.can.sent), 7)
        self.assertTrue(all(raw[4 + 1] == 0x10 for raw in self.leader_wire.sent[-7:]))

    def test_partial_enable_failure_still_disables_every_motor_and_closes_ports(self):
        self.cfg.calibration_verified = True
        self.cfg.adapter = "socketcan"
        self.can.fail_enable = 4
        with self.assertRaises(OSError):
            hardware(
                self.cfg,
                execute=True,
                serial_factory=lambda *a, **kw: self.leader_wire,
                socket_factory=lambda _: self.can,
                clock=self.clock,
                sleep=self.clock.sleep,
            )
        self.assertEqual(self.can.enabled, set())
        self.assertEqual(sum(data == b"\xff" * 7 + b"\xfd" for _, data in self.can.sent), 7)
        self.assertTrue(self.can.closed and self.leader_wire.closed)

    def test_small_zero_drift_at_soft_limit_starts_with_bounded_commands(self):
        self.cfg.calibration_verified = True
        self.cfg.adapter = "socketcan"
        self.cfg.baseline_s = 0.2
        self.leader_wire.angles[1] = -0.1
        hardware(
            self.cfg,
            execute=True,
            duration=0.03,
            serial_factory=lambda *a, **kw: self.leader_wire,
            socket_factory=lambda _: self.can,
            clock=self.clock,
            sleep=self.clock.sleep,
        )
        positions = [
            struct.unpack_from("<f", data)[0] for can_id, data in self.can.sent if can_id == 0x102
        ]
        self.assertTrue(positions)
        self.assertTrue(all(p <= 0 for p in positions))

    def test_large_startup_mismatch_still_blocks_enable(self):
        self.cfg.calibration_verified = True
        self.cfg.adapter = "socketcan"
        self.leader_wire.angles[0] = -30
        with self.assertRaisesRegex(LinkError, "Startup pose"):
            hardware(
                self.cfg,
                execute=True,
                serial_factory=lambda *a, **kw: self.leader_wire,
                socket_factory=lambda _: self.can,
                clock=self.clock,
                sleep=self.clock.sleep,
            )
        self.assertFalse(any(data == b"\xff" * 7 + b"\xfc" for _, data in self.can.sent))

    def test_second_port_failure_closes_first_without_motion(self):
        factory = Mock(side_effect=[self.leader_wire, OSError("port busy")])
        with self.assertRaises(OSError):
            hardware(self.cfg, serial_factory=factory)
        self.assertTrue(self.leader_wire.closed)
        self.assertEqual(self.leader_wire.sent, [])

    def test_unverified_config_rejected_before_opening_ports(self):
        factory = Mock(side_effect=AssertionError("opened device"))
        with self.assertRaises(ValueError):
            hardware(self.cfg, execute=True, serial_factory=factory)
        factory.assert_not_called()

    def test_invalid_profiles_are_rejected(self):
        for changes in ({"direction": 0}, {"torque_reference_nm": 0}, {"speed_deg_s": math.inf}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                cfg = copy.deepcopy(self.cfg)
                for key, value in changes.items():
                    setattr(cfg.joints[0], key, value)
                cfg.validate()
        for key, value in (
            ("loop_hz", 0),
            ("power_min_mw", 0),
            ("confirm_windows", 0),
            ("link_timeout_s", 0.001),
            ("button_id", 0),
            ("baseline_s", 0),
        ):
            with self.subTest(key=key), self.assertRaises(ValueError):
                cfg = copy.deepcopy(self.cfg)
                setattr(cfg, key, value)
                cfg.validate()

    def test_simulation_runs_without_any_installed_external_package(self):
        app = Path(__file__).resolve().parents[1] / "teleop.py"
        result = subprocess.run(
            [sys.executable, "-S", str(app), "--simulate", "--duration", "0.1"],
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stderr)

    def test_simulation_distinguishes_free_following_and_real_stall(self):
        normal = simulate(self.cfg, 5)
        stalled = simulate(self.cfg, 5, stalled=0)
        self.assertFalse(normal.feedback.triggers)
        self.assertEqual(stalled.feedback.triggers, {0})


if __name__ == "__main__":
    unittest.main()
