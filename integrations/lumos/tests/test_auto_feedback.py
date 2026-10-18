import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from lumos_auto_feedback import FeedbackConfig, LumosAutoFeedback
from test_follow_control import teleop


class AutoFeedbackTests(unittest.TestCase):
    def setUp(self):
        self.now = 0.0
        self.calls = []
        self.positions = [0.0] * 6
        self.torques = [0.0] * 6
        self.gripper = SimpleNamespace(
            position=0.0,
            effort_nm=0.0,
            feedback_valid=True,
            feedback_age_ms=1.0,
            enabled=True,
            error_code=0,
        )
        self.leader = SimpleNamespace(stop_on_control_mode=lambda *args: self.calls.append(args))
        self.arm = SimpleNamespace(
            get_joint_positions=lambda: self.positions,
            get_joint_torques=lambda: self.torques,
            get_gripper_state=lambda: self.gripper,
        )
        self.core = self.make()

    def make(self, **kwargs):
        return LumosAutoFeedback(
            self.leader,
            self.arm,
            kwargs.pop("joint_map", range(6)),
            clock=lambda: self.now,
            **kwargs
        )

    def tick(self, degrees=30.0, dt=0.41, **kwargs):
        self.now += dt
        return self.core.update([math.radians(degrees)] * 6, **kwargs)

    def lock(self):
        self.torques = [5.0] * 6
        self.tick()
        self.tick()
        self.assertTrue(self.core.triggers)

    def test_no_load_uses_minimum_damping_and_suppresses_duplicate_writes(self):
        self.tick(0)
        self.assertEqual(self.calls, [(i, 0x12, 1) for i in range(6)])
        self.tick(0)
        self.assertEqual(len(self.calls), 6)

    def test_load_clamps_and_uses_custom_joint_mapping(self):
        self.core = self.make(joint_map=(5, 4, 3, 2, 1, 0))
        self.torques[0] = -100.0
        self.tick(0)
        self.assertIn((5, 0x12, 1200), self.calls)
        self.assertIn((4, 0x12, 1), self.calls)

    def test_power_deadband_filters_small_load_changes(self):
        self.tick(0)
        self.torques[0] = 0.11
        self.tick(0)
        self.assertEqual(len(self.calls), 6)
        self.torques[0] = 1.0
        self.tick(0)
        self.assertEqual(len(self.calls), 7)

    def test_high_error_without_load_never_locks(self):
        for _ in range(5):
            self.tick()
        self.assertFalse(self.core.triggers)
        self.assertTrue(all(mode == 0x12 for _, mode, _ in self.calls))

    def test_stall_requires_window_and_holds_all_joints(self):
        self.torques[0] = 5.0
        self.tick()
        self.assertFalse(self.core.triggers)
        self.tick()
        self.assertEqual(self.core.triggers, {0})
        self.assertEqual(self.calls[-6:], [(i, 0x11, 6000) for i in range(6)])

    def test_normal_moving_follower_does_not_lock_despite_large_lag(self):
        self.torques = [5.0] * 6
        self.tick(30)
        self.positions = [math.radians(10)] * 6
        self.tick(40)
        self.assertFalse(self.core.triggers)

    def test_converging_on_stationary_target_does_not_lock(self):
        self.torques = [5.0] * 6
        self.tick(40)
        self.positions = [math.radians(5)] * 6
        self.tick(40)
        self.assertFalse(self.core.triggers)

    def test_reverse_sliding_is_not_normal_following(self):
        self.torques = [5.0] * 6
        self.tick(30)
        self.positions = [math.radians(-10)] * 6
        self.tick(40)
        self.assertTrue(self.core.triggers)

    def test_multiple_confirmation_windows(self):
        self.core = self.make(config=FeedbackConfig(confirm_windows=2))
        self.torques = [5.0] * 6
        self.tick()
        self.tick()
        self.assertFalse(self.core.triggers)
        self.tick()
        self.assertTrue(self.core.triggers)

    def test_recovery_requires_stable_small_error_on_every_trigger(self):
        self.lock()
        self.positions = [math.radians(30)] * 6
        self.tick(dt=0.04)
        self.assertTrue(self.core.triggers)
        self.positions[0] = 0.0
        self.tick(dt=0.2)
        self.positions[0] = math.radians(30)
        self.tick(dt=0.2)
        self.assertEqual(self.core.triggers, {0})
        self.tick(dt=0.31)
        self.assertFalse(self.core.triggers)
        self.assertTrue(all(mode == 0x12 for _, mode, _ in self.calls[-6:]))

    def test_hold_reissued_even_when_feedback_sampling_is_not_due(self):
        self.core = self.make(config=FeedbackConfig(sample_hz=0.1, relock_s=0.2))
        self.tick(0, manual_hold=True)
        self.tick(0, dt=0.21, manual_hold=True)
        self.assertEqual(len(self.calls), 12)
        self.assertTrue(all(mode == 0x11 for _, mode, _ in self.calls))

    def test_hd_release_cannot_override_stall_and_no_release_command_is_sent(self):
        self.lock()
        self.tick(manual_hold=True)
        self.tick(manual_hold=False)
        self.assertTrue(self.core.triggers)
        self.assertTrue(all(mode != 0x10 for _, mode, _ in self.calls))
        self.assertEqual(self.core.state, "堵转保持")

    def test_manual_hold_excludes_gripper(self):
        self.core = self.make(gripper_span_deg=90)
        self.tick(0, gripper_target=0, manual_hold=True)
        self.assertIn((6, 0x12, 1), self.calls)
        self.assertEqual(sum(mode == 0x11 for _, mode, _ in self.calls), 6)

    def test_gripper_stall_uses_calibrated_range_and_holds_all_seven(self):
        self.core = self.make(gripper_span_deg=-90)
        self.gripper.effort_nm = 1.0
        self.tick(0, gripper_target=1.0)
        self.tick(0, gripper_target=1.0)
        self.assertEqual(self.core.triggers, {6})
        self.assertEqual(self.calls[-7:], [(i, 0x11, 6000) for i in range(7)])

    def test_gripper_freshness_fault_stops_new_commands_and_latches(self):
        self.core = self.make(gripper_span_deg=90)
        self.gripper.feedback_age_ms = 251
        self.assertFalse(self.tick(0))
        self.assertEqual(self.calls[-7:], [(i, 0x11, 6000) for i in range(7)])
        self.gripper.feedback_age_ms = 1
        self.assertFalse(self.tick(0))
        self.assertTrue(self.tick(0, dt=0.31))

    def test_invalid_gripper_state_is_rejected(self):
        for field, value in (
            ("position", math.nan),
            ("effort_nm", math.inf),
            ("enabled", False),
            ("feedback_valid", False),
            ("error_code", 1),
        ):
            with self.subTest(field=field):
                old = getattr(self.gripper, field)
                setattr(self.gripper, field, value)
                core = self.make(gripper_span_deg=90)
                self.assertFalse(core.update([0.0] * 6))
                setattr(self.gripper, field, old)

    def test_fault_recovery_requires_positions_to_match_and_stay_stable(self):
        self.positions = [math.nan] * 6
        self.assertFalse(self.tick())
        self.positions = [0.0] * 6
        self.assertFalse(self.tick())
        self.positions = [math.radians(30)] * 6
        self.assertFalse(self.tick())
        self.positions[0] = 0
        self.assertFalse(self.tick(dt=0.2))
        self.positions[0] = math.radians(30)
        self.assertFalse(self.tick(dt=0.2))
        self.assertTrue(self.tick(dt=0.31))

    def test_sdk_exception_and_invalid_array_latch_fault(self):
        for bad in ([0.0] * 5, [math.nan] * 6):
            self.torques = bad
            self.assertFalse(self.tick(0))
        self.arm.get_joint_positions = Mock(side_effect=RuntimeError("CAN failed"))
        self.assertFalse(self.tick(0))

    def test_gain_normalizes_stall_error_to_leader_degrees(self):
        self.core = self.make(gain=0.5)
        self.torques = [5.0] * 6
        self.tick(11)
        self.tick(11)
        self.assertTrue(self.core.triggers)

    def test_cleanup_releases_only_managed_joints_and_is_idempotent(self):
        self.core.close()
        self.assertEqual(self.calls, [])
        self.core = self.make()
        self.lock()
        self.core.close()
        self.assertEqual(self.calls[-6:], [(i, 0x10, 0) for i in range(6)])
        count = len(self.calls)
        self.core.close()
        self.assertEqual(len(self.calls), count)
        self.assertFalse(self.tick())

    def test_cleanup_continues_after_one_joint_release_fails(self):
        self.tick(0)

        def stop(servo_id, mode, power):
            self.calls.append((servo_id, mode, power))
            if servo_id == 0:
                raise OSError("UART failed")

        self.leader.stop_on_control_mode = stop
        with self.assertLogs("lumos_auto_feedback", level="ERROR"):
            self.core.close()
        self.assertEqual(self.calls[-1], (5, 0x10, 0))

    def test_unsupported_sdk_fails_at_startup(self):
        self.arm.get_joint_torques = None
        with self.assertRaisesRegex(RuntimeError, "SDK"):
            self.make()

    def test_invalid_settings_rejected(self):
        for kwargs in (
            {"hold_mw": 0},
            {"release_deg": 21},
            {"confirm_windows": 1.5},
            {"window_s": math.nan},
            {"torque_reference_nm": (0,) * 7},
            {"torque_floor_nm": (0,) * 6},
            {"sample_hz": 0},
        ):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                FeedbackConfig(**kwargs)

    def test_hd_button_defers_writes_to_feedback_controller(self):
        self.leader.ping = lambda _: True
        self.leader.servos = {7: SimpleNamespace(angle_monitor=180)}
        button = teleop.HDButton(self.leader, 7)
        button.feedback = self.core
        button.update()
        self.leader.servos[7].angle_monitor = 0
        button.update()
        self.assertFalse(button.locked)
        self.assertEqual(self.calls, [])

    def test_fresh_leader_read_rejects_missing_response_instead_of_cached_angle(self):
        self.leader.servos = {i: SimpleNamespace(angle_monitor=12) for i in range(6)}
        self.leader.send_sync_servo_monitor = Mock(return_value={})
        with self.assertRaises(RuntimeError):
            teleop.read_leader(self.leader, range(6), fresh=True)
        self.leader.send_sync_servo_monitor.assert_called_once_with(list(range(6)), realtime=True)


if __name__ == "__main__":
    unittest.main()
