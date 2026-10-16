import contextlib
import copy
import io
import json
import math
import struct
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import b601_dm as dm
import stararm102_b601_dm as app

ROOT = Path(__file__).resolve().parents[1]
EXAMPLE = ROOT / "configs/b601_dm.example.json"


def rx(can_id, data):
    return b"\xaa\x11\x08" + struct.pack("<I", can_id) + data + b"\x55"


def feedback(motor_id=1, status=1):
    return bytes([(status << 4) | motor_id, 0x80, 0, 0x80, 8, 0, 25, 26])


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, dt):
        self.now += dt


class Wire:
    def __init__(self, respond=True):
        self.data = bytearray()
        self.sent = []
        self.respond = respond
        self.mode = 2
        self.enabled = False
        self.closed = False
        self.drop_ids = set()
        self.short = False

    def close(self):
        self.closed = True

    @property
    def in_waiting(self):
        return len(self.data)

    def read(self, count):
        data = bytes(self.data[:count])
        del self.data[:count]
        return data

    def write(self, frame):
        self.sent.append(frame)
        if self.short:
            return 0
        if frame[21:28] == b"\xff" * 7:
            if frame[28] == 0xFC:
                self.enabled = True
            elif frame[28] == 0xFD:
                self.enabled = False
        if self.respond and struct.unpack_from("<I", frame, 13)[0] == 0x7FF:
            payload = frame[21:29]
            motor_id = struct.unpack_from("<H", payload)[0]
            if motor_id in self.drop_ids:
                return len(frame)
            if payload[2] == 0xCC:
                self.data.extend(rx(motor_id + 0x10, feedback(motor_id, 1 if self.enabled else 0)))
            elif payload[2] == 0x33:
                rid = payload[3]
                value = (
                    struct.pack("<I", self.mode)
                    if rid == 10
                    else struct.pack(
                        "<f",
                        {
                            21: 12.5,
                            22: 10.0 if motor_id < 4 else 30.0,
                            23: 28.0 if motor_id < 4 else 10.0,
                        }[rid],
                    )
                )
                self.data.extend(rx(motor_id + 0x10, bytes([motor_id, 1, 0x33, rid]) + value))
        return len(frame)


class B601Tests(unittest.TestCase):
    def setUp(self):
        self.profile = dm.Profile.load(EXAMPLE)

    def backend(self, respond=True):
        clock, wire = Clock(), Wire(respond)
        return dm.DMSerial(wire, self.profile.joints, clock, clock.sleep), wire, clock

    def profile_error(self, edit):
        data = copy.deepcopy(self.profile.data)
        edit(data)
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder) / "p.json"
            path.write_text(json.dumps(data), encoding="utf8")
            with self.assertRaises(ValueError):
                dm.Profile.load(path)

    def states(self, now, sequence=10, status=1):
        return {i: dm.State(0.0, 0.0, 0.02, status, 25, 25, now, sequence + i) for i in range(7)}

    def session(self):
        leader = app.SimLeader()
        follower = app.SimFollower(self.profile.joints)
        session = app.Session(self.profile, leader, follower, self.states(0, 1), 0)
        session.console.emit = lambda msg: None
        return session, leader, follower

    def test_example_config_is_unverified_and_complete(self):
        self.assertFalse(self.profile.data["calibration_verified"])
        self.assertEqual([j.motor_id for j in self.profile.joints], list(range(1, 8)))

    def test_invalid_profiles_rejected(self):
        edits = [
            lambda d: d["joints"][0].update(sign=0),
            lambda d: d["joints"][0].update(scale=0),
            lambda d: d["joints"][0].update(torque_reference_nm=float("nan")),
            lambda d: d["joints"][1].update(motor_id=1),
            lambda d: d["joints"][1].update(feedback_id=0x11),
            lambda d: d["joints"][1].update(lower_rad=2),
            lambda d: d.update(loop_hz=0),
            lambda d: d.update(leader_port=d["follower_port"]),
            lambda d: d["stall"].update(release_deg=20),
        ]
        for edit in edits:
            with self.subTest(edit=edit):
                self.profile_error(edit)

    def test_mapping_round_trip_and_gripper_scale(self):
        mapping = dm.JointMap(self.profile.joints)
        angles = dict.fromkeys(range(7), 10.0)
        targets, limited = mapping.forward(angles)
        self.assertFalse(limited)
        self.assertAlmostEqual(targets[6], -4 * math.radians(10))
        self.assertAlmostEqual(targets[1], -math.radians(10))
        inverse = mapping.inverse(targets)
        for i in angles:
            self.assertAlmostEqual(inverse[i], angles[i])

    def test_limits_reported_and_clipped(self):
        angles = dict.fromkeys(range(7), 0.0)
        angles[0] = 1000
        targets, limited = dm.JointMap(self.profile.joints).forward(angles)
        self.assertEqual(limited, {0})
        self.assertEqual(targets[0], self.profile.joints[0].upper_rad)

    def test_slew_caps_delayed_loop_step(self):
        mapping = dm.JointMap(self.profile.joints)
        result = mapping.slew(dict.fromkeys(range(7), 1.0), dict.fromkeys(range(7), 0.0), 10)
        self.assertAlmostEqual(result[0], 0.4 * 0.05)
        self.assertAlmostEqual(result[6], 0.8 * 0.05)

    def test_rebase_produces_no_target_jump(self):
        mapping = dm.JointMap(self.profile.joints)
        angles = dict.fromkeys(range(7), 30.0)
        positions = {j.leader_id: (j.lower_rad + j.upper_rad) / 2 for j in self.profile.joints}
        mapping.rebase(angles, positions)
        self.assertEqual(mapping.forward(angles), (positions, set()))

    def test_enable_frame_matches_reference_wire_format(self):
        expected = bytes.fromhex(
            "55 aa 1e 03 01 00 00 00 0a 00 00 00 00 01 00 00 00 00 08 00 00 ff ff ff ff ff ff ff fc 00"
        )
        self.assertEqual(dm.encode_frame(1, b"\xff" * 7 + b"\xfc"), expected)

    def test_receive_fragmentation_and_noise(self):
        backend, _, _ = self.backend(False)
        packet = rx(0x11, feedback())
        backend.feed(b"garbage" + packet[:7])
        self.assertEqual(backend.states, {})
        backend.feed(packet[7:])
        state = backend.states[0]
        self.assertEqual(state.status, 1)
        self.assertAlmostEqual(state.pos, 12.5 / 65535)
        self.assertAlmostEqual(state.torque, 28 / 4095)
        self.assertEqual((state.mos_c, state.rotor_c), (25, 26))

    def test_foreign_can_id_and_wrong_motor_rejected(self):
        backend, _, _ = self.backend(False)
        backend.feed(rx(0x77, feedback()) + rx(0x11, feedback(2)))
        self.assertEqual(backend.states, {})

    def test_bad_flags_and_trailer_rejected(self):
        backend, _, _ = self.backend(False)
        for offset, value in ((1, 0x12), (2, 0x48), (15, 0x54)):
            packet = bytearray(rx(0x11, feedback()))
            packet[offset] = value
            backend.feed(packet)
        self.assertEqual(backend.states, {})

    def test_register_reply_is_not_motor_feedback(self):
        backend, _, _ = self.backend(False)
        backend.pending_registers.add((0, 10))
        backend.feed(rx(0x11, bytes([1, 1, 0x33, 10, 2, 0, 0, 0])))
        self.assertEqual(backend.states, {})
        self.assertEqual(backend.replies[(0, 10)], struct.pack("<I", 2))

    def test_unrequested_register_reply_is_ignored(self):
        backend, _, _ = self.backend(False)
        backend.feed(rx(0x11, bytes([1, 1, 0x33, 10, 2, 0, 0, 0])))
        self.assertEqual(backend.replies, {})

    def test_cached_states_not_returned_as_new_samples(self):
        backend, wire, _ = self.backend(False)
        wire.data.extend(rx(0x11, feedback()))
        self.assertEqual(backend.read(0.005), {})

    def test_identical_position_with_new_frame_is_fresh(self):
        backend, _, _ = self.backend()
        first = backend.read()
        second = backend.read()
        self.assertEqual(len(first), 7)
        self.assertEqual(first[0].pos, second[0].pos)
        self.assertGreater(second[0].sequence, first[0].sequence)

    def test_partial_feedback_preserves_missing_joint(self):
        backend, wire, _ = self.backend()
        wire.drop_ids = {3}
        self.assertEqual(set(backend.read(0.005)), {0, 1, 3, 4, 5, 6})

    def test_verify_reads_mode_and_actual_feedback_scaling(self):
        backend, wire, _ = self.backend()
        backend.verify()
        self.assertEqual(backend.limits[0], (12.5, 10, 28))
        self.assertEqual(backend.limits[6], (12.5, 30, 10))
        self.assertTrue(all(f[23] == 0x33 for f in wire.sent))

    def test_wrong_motor_mode_fails_without_writing_parameters(self):
        backend, wire, _ = self.backend()
        wire.mode = 1
        with self.assertRaises(dm.LinkError):
            backend.verify()
        self.assertEqual(len(wire.sent), 1)
        self.assertEqual(wire.sent[0][23], 0x33)

    def test_position_command_uses_mode_offset_and_rad_units(self):
        backend, wire, _ = self.backend()
        targets = dict.fromkeys(range(7), 0.0)
        targets[0] = 0.1
        backend.command(targets)
        self.assertEqual(struct.unpack_from("<I", wire.sent[0], 13)[0], 0x101)
        p, v = struct.unpack("<ff", wire.sent[0][21:29])
        self.assertAlmostEqual(p, 0.1, places=6)
        self.assertAlmostEqual(v, 0.4, places=6)

    def test_invalid_target_batch_sends_nothing(self):
        backend, wire, _ = self.backend()
        targets = dict.fromkeys(range(7), 0.0)
        targets[6] = 100
        with self.assertRaises(ValueError):
            backend.command(targets)
        self.assertEqual(wire.sent, [])

    def test_live_initialization_enable_and_cleanup(self):
        self.profile.data["calibration_verified"] = True
        leader = app.SimLeader()

        def monitor(ids):
            for i in ids:
                leader.servos[i].angle_monitor = 0.0
            return leader.servos

        leader.send_sync_servo_monitor = monitor
        leader_uart, wire = Wire(False), Wire()
        with patch.object(app.serial, "Serial", side_effect=[leader_uart, wire]), patch.object(
            app.uservo, "UartServoManager", return_value=leader
        ), patch.object(
            app.time, "sleep", side_effect=KeyboardInterrupt
        ), contextlib.redirect_stdout(
            io.StringIO()
        ):
            app.hardware(self.profile)
        self.assertTrue(wire.closed and leader_uart.closed)
        self.assertFalse(wire.enabled)
        self.assertEqual(sum(f[28] == 0xFC and f[21:28] == b"\xff" * 7 for f in wire.sent), 7)
        self.assertEqual(sum(f[28] == 0xFD and f[21:28] == b"\xff" * 7 for f in wire.sent), 7)
        self.assertEqual(leader.calls[-1], ("stop", (255, app.legacy.METHOD_RELEASE, 0)))

    def test_capture_is_read_only_and_never_certifies_calibration(self):
        leader = app.SimLeader()

        def monitor(ids):
            for i in ids:
                leader.servos[i].angle_monitor = 0.0
            return leader.servos

        leader.send_sync_servo_monitor = monitor
        leader_uart, wire = Wire(False), Wire()
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder) / "captured.json"
            with patch.object(app.serial, "Serial", side_effect=[leader_uart, wire]), patch.object(
                app.uservo, "UartServoManager", return_value=leader
            ), contextlib.redirect_stdout(io.StringIO()):
                app.hardware(self.profile, output)
            captured = dm.Profile.load(output)
            self.assertFalse(captured.data["calibration_verified"])
        self.assertTrue(all(struct.unpack_from("<I", f, 13)[0] == 0x7FF for f in wire.sent))
        self.assertEqual(leader.calls, [])
        self.assertTrue(wire.closed and leader_uart.closed)

    def test_second_port_failure_closes_first_port(self):
        self.profile.data["calibration_verified"] = True
        leader_uart = Wire(False)
        with patch.object(
            app.serial, "Serial", side_effect=[leader_uart, OSError("port busy")]
        ), patch.object(app.uservo, "UartServoManager", return_value=app.SimLeader()):
            with self.assertRaises(OSError):
                app.hardware(self.profile)
        self.assertTrue(leader_uart.closed)

    def test_short_write_is_an_error(self):
        backend, wire, _ = self.backend()
        wire.short = True
        with self.assertRaises(dm.LinkError):
            backend.enable()

    def test_disable_attempts_every_motor_after_write_failure(self):
        backend, wire, _ = self.backend()
        wire.short = True
        with self.assertRaises(dm.LinkError):
            backend.disable()
        self.assertEqual(len(wire.sent), 7)

    def test_torque_feedback_uses_small_nm_references(self):
        fb = app.TorqueFeedback(self.profile.joints)
        fb.set_current_max([0.5] * 7)
        self.assertAlmostEqual(fb.reference(3), 0.5)
        self.assertAlmostEqual(fb.ratio_for(3, -0.275), 0.5)
        self.assertEqual(fb.power_for(3, 0.05), 1)

    def test_console_reports_torque_units(self):
        session, _, _ = self.session()
        messages = []
        session.console.emit = messages.append
        session.console.print_feedback_status()
        self.assertIn("Nm", "".join(messages))
        self.assertNotIn("mA", "".join(messages))

    def test_torque_console_accepts_decimal_references(self):
        session, _, _ = self.session()
        session.console.handle_line("fbmax 0.5,0.5,0.5,0.5,0.5,0.5,0.5")
        self.assertEqual(session.console.feedback.current_max[0], 0.5)

    def test_timeout_latches_hold_and_locks_leader(self):
        session, _, _ = self.session()
        session.step(None, {}, 0.3)
        self.assertTrue(session.paused)
        self.assertEqual(session.console.stall.locked, set(range(7)))
        session.step(dict.fromkeys(range(7), 0.0), self.states(0.4), 0.4)
        self.assertTrue(session.paused)

    def test_reused_sequence_cannot_clear_link_timeout(self):
        session, _, _ = self.session()
        session.step(dict.fromkeys(range(7), 0.0), self.states(0.3, 1), 0.3)
        self.assertTrue(session.paused)

    def test_resume_rebases_to_current_pose(self):
        session, _, follower = self.session()
        session.step(None, {}, 0.3)
        session.resume_requested = True
        session.step(dict.fromkeys(range(7), 25.0), self.states(0.4), 0.4)
        self.assertFalse(session.paused)
        self.assertEqual(follower.targets, dict.fromkeys(range(7), 0.0))

    def test_motor_fault_cannot_be_resumed(self):
        session, _, _ = self.session()
        session.resume_requested = True
        session.step(dict.fromkeys(range(7), 0.0), self.states(0.1, status=0xA), 0.1)
        self.assertTrue(session.paused)
        self.assertFalse(session.resume_requested)

    def test_limit_violation_holds_instead_of_sending_clipped_motion(self):
        session, _, follower = self.session()
        angles = dict.fromkeys(range(7), 0.0)
        angles[0] = 1000
        session.step(angles, self.states(0.02), 0.02)
        self.assertTrue(session.paused)
        self.assertEqual(follower.targets, dict.fromkeys(range(7), 0.0))

    def test_button_does_not_release_console_lock(self):
        session, _, _ = self.session()
        session.console.handle_line("st lock")
        session.console.drain(session.leader)
        session.button(True)
        session.button(False)
        self.assertEqual(session.console.stall.locked, set(range(7)))

    def test_button_own_lock_is_released(self):
        session, _, _ = self.session()
        session.button(True)
        self.assertEqual(session.console.stall.manual_source, "b601-button")
        session.button(False)
        self.assertFalse(session.console.stall.locked)

    def test_zero_quantization_does_not_fault(self):
        session, _, _ = self.session()
        states = self.states(0.02)
        for i in (1, 2, 6):
            states[i] = dm.State(0.0002, 0, 0.02, 1, 25, 25, 0.02, 20 + i)
        session.step(dict.fromkeys(range(7), 0.0), states, 0.02)
        self.assertFalse(session.paused)

    def test_simulation_exercises_stall_lock_without_serial(self):
        with patch.object(
            app.serial, "Serial", side_effect=AssertionError("hardware used")
        ), contextlib.redirect_stdout(io.StringIO()):
            session = app.simulate(self.profile, 5, 0)
        self.assertEqual(session.console.stall.lock_count, 1)
        self.assertEqual(session.console.stall.locked, set(range(7)))

    def test_unverified_profile_cannot_open_ports(self):
        with patch.object(app.serial, "Serial", side_effect=AssertionError("hardware used")):
            with self.assertRaises(ValueError):
                app.hardware(self.profile)

    def test_default_cli_is_simulation(self):
        with patch.object(
            app.serial, "Serial", side_effect=AssertionError("hardware used")
        ), contextlib.redirect_stdout(io.StringIO()):
            self.assertEqual(app.main(["--duration", ".1"]), 0)


if __name__ == "__main__":
    unittest.main()
