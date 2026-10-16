from __future__ import annotations

import argparse
import copy
import json
import math
import os
import sys
import time
from pathlib import Path
from types import SimpleNamespace

import serial
import fashionstar_uart_sdk as uservo

import stararm102_ro as legacy
from b601_dm import DMSerial, JointMap, LinkError, Profile, State, number

FEEDBACK_LIMIT_MARGIN_RAD = 0.01


def bounded_positions(profile, states):
    return {
        j.leader_id: min(j.upper_rad, max(j.lower_rad, states[j.leader_id].pos))
        for j in profile.joints
    }


DEFAULT_PROFILE = Path(__file__).resolve().parent / "configs" / "b601_dm.example.json"


class TorqueFeedback(legacy.LoadFeedback):
    def __init__(self, joints, power_min=1, power_max=1200, deadband=20, hz=50):
        super().__init__(
            [j.leader_id for j in joints],
            enabled=True,
            hz=hz,
            power_min=power_min,
            power_max=power_max,
            current_max=[j.torque_reference_nm for j in joints],
            deadband=deadband,
        )
        self.floors = {j.leader_id: j.torque_floor_nm for j in joints}
        self.current_floor = 0.0
        self.peaks = dict(self.floors)

    def reference(self, servo_id):
        ref = self.current_max[servo_id] if self.current_max is not None else self.peaks[servo_id]
        return max(self.floors[servo_id] + 1e-6, ref)

    def ratio_for(self, servo_id, torque):
        floor = self.floors[servo_id]
        return min(1.0, max(0.0, (abs(torque) - floor) / (self.reference(servo_id) - floor)))


class B601Console(legacy.DampingConsole):
    def __init__(self, profile):
        fb = profile.data["feedback"]
        feedback = TorqueFeedback(
            profile.joints,
            fb["power_min_mw"],
            fb["power_max_mw"],
            fb["deadband_mw"],
            profile.data["loop_hz"],
        )
        guard = legacy.StallGuard(range(7), **profile.data["stall"])
        super().__init__(range(7), feedback=feedback, stall=guard)
        self.session = None

    def handle_line(self, line):
        tokens = line.strip().lower().split()
        if not tokens:
            return
        command = tokens[0]
        if command in ("h", "help", "?"):
            self.emit("B601-DM | stop 保持 · resume 重新配对 · s 状态 · q 退出")
            self.emit("反馈：fb on|off · fbmax <Nm或7个值或auto> · fbq <Nm> · fbp <min> <max>")
            self.emit(legacy.HELP_STALL)
            return
        if command in ("stop", "pause"):
            self.session.pause("operator stop")
            return
        if command == "resume":
            self.session.resume_requested = True
            return
        if (
            self.session is not None
            and self.session.paused
            and command
            not in (
                "s",
                "status",
                "fb",
                "fbs",
                "st",
                "q",
                "quit",
                "exit",
                "fbmax",
                "fbq",
                "fbp",
                "stlock",
                "strel",
                "stn",
                "stwin",
                "sttrack",
                "stload",
                "ststatic",
                "sthold",
                "stmethod",
                "strelock",
                "stscope",
            )
        ):
            self.emit("[保持] 使用 resume 恢复配对或 q 退出")
            return
        if (
            self.session is not None
            and self.session.paused
            and command in ("st", "stall")
            and len(tokens) > 1
        ):
            self.emit("[保持] 使用 resume 恢复配对")
            return
        if command == "n" and self.session is not None:
            self.session.pause("operator release", lock_leader=False)
        super().handle_line(line)

    def _handle_fbmax(self, rest):
        if len(rest) == 1 and rest[0].lower() == "auto":
            self.feedback.set_current_max(None)
            self.emit("[反馈] 自动峰值参考 Nm")
            return
        try:
            values = [float(v) for v in rest]
            if len(values) not in (1, 7) or any(not math.isfinite(v) or v <= 0 for v in values):
                raise ValueError
            expanded = values * 7 if len(values) == 1 else values
            if any(v <= self.feedback.floors[i] for i, v in enumerate(expanded)):
                raise ValueError
        except ValueError:
            self.emit("[参数] fbmax <正数Nm或7个值>，参考值须高于噪声门槛")
            return
        self.feedback.set_current_max(values)
        self.emit("[反馈] 力矩参考已更新 Nm")

    def _handle_fb_number(self, rest, label, attr, unit, minimum):
        if attr == "current_floor":
            value = legacy._parse_float(rest[0]) if len(rest) == 1 else None
            if (
                value is None
                or value < 0
                or any(value >= self.feedback.reference(i) for i in range(7))
            ):
                self.emit("[参数] fbq <Nm>，须低于各关节参考值")
                return
            self.feedback.floors = dict.fromkeys(range(7), value)
            self.feedback.sent = dict.fromkeys(range(7))
            self.emit(f"[反馈] 力矩噪声门槛 {value:g}Nm")
            return
        if attr == "hz":
            value = legacy._parse_float(rest[0]) if len(rest) == 1 else None
            if value is None or not 1 <= value <= self.session.profile.data["loop_hz"]:
                self.emit("[参数] fbhz 必须在 1 和控制循环频率之间")
                return
        super()._handle_fb_number(rest, label, attr, unit, minimum)

    def print_feedback_status(self):
        fb = self.feedback
        self.emit(
            f'[反馈] {"ON" if fb.enabled else "OFF"} | 力矩 Nm | 阻尼 {fb.power_min}–{fb.power_max}mW'
        )
        for i in self.joint_ids:
            self.emit(
                f"J{i+1} {fb.currents[i]:.3f}Nm / ref {fb.reference(i):.3f}Nm"
                f" | {fb.power_for(i, fb.currents[i])}mW"
            )

    def print_status(self):
        if self.session is not None:
            self.emit(f'[601] {self.session.reason if self.session.paused else "FOLLOW"}')
        super().print_status()

    def process_sample(self, leader, target_deg, actual_deg, torques, now):
        fb, guard = self.feedback, self.stall
        if not (fb.enabled or guard.enabled) or now + 1e-9 < fb.next_at:
            return
        diffs = {i: abs(actual_deg[i] - target_deg[i]) for i in self.joint_ids}
        loads = {i: fb.ratio_for(i, torques[i]) for i in self.joint_ids}
        to_lock, to_release = guard.update(diffs, target_deg, actual_deg, loads, now)
        fb.observe({i: abs(v) for i, v in torques.items()}, now)
        if to_lock:
            for i in to_lock:
                guard.follow_targets[i] = target_deg[i]
            self._lock_joints(leader, to_lock, now)
            self.emit("[锁定] 601 堵转 | " + ",".join(f"J{i+1}" for i in guard.triggers))
        if fb.enabled:
            updates = {i: p for i, p in fb.pending().items() if i not in guard.locked}
            for i, power in updates.items():
                if i not in fb.entered:
                    leader.stop_on_control_mode(i, legacy.METHOD_DAMPING, power)
                    fb.entered.add(i)
                leader.set_damping(i, power)
            fb.commit(updates)
            self.powers.update(updates)
            if updates:
                self.enabled = True
        for i in to_release:
            self._restore_joint_after_stall(leader, i)
        if to_release:
            self.emit("[解锁] 601 跟随恢复")
        self._refresh_status()


class Session:
    def __init__(self, profile, leader, follower, initial, now, console=None):
        self.profile, self.leader, self.follower = profile, leader, follower
        self.mapper = JointMap(profile.joints)
        self.console = console if console is not None else B601Console(profile)
        self.console.session = self
        self.commanded = bounded_positions(profile, initial)
        self.last_sequences = {i: st.sequence for i, st in initial.items()}
        self.button_pressed = False
        self.last_good = now
        self.last_step = now
        self.latest = dict(initial)
        self.paused = False
        self.reason = ""
        self.resume_requested = False

    def pause(self, reason, lock_leader=True):
        if not self.paused:
            self.commanded = bounded_positions(self.profile, self.latest)
            self.console.emit(f"[保持] {reason}")
        self.paused, self.reason = True, reason
        if lock_leader:
            guard = self.console.stall
            guard.auto_locked = False
            guard.manual_source = "b601-pause"
            missing = set(range(7)) - guard.locked
            guard.locked.update(missing)
            if missing:
                self.console._lock_joints(self.leader, sorted(missing), self.last_step)

    def button(self, pressed):
        if (
            not self.paused
            and pressed
            and not self.button_pressed
            and not self.console.stall.locked
        ):
            guard = self.console.stall
            guard.locked.update(range(6))
            guard.manual_source = "b601-button"
            self.console._lock_joints(self.leader, list(range(6)), self.last_step)
        elif (
            not pressed
            and self.button_pressed
            and self.console.stall.manual_source == "b601-button"
        ):
            self.console.release_stalled()
        self.button_pressed = pressed

    def step(self, leader_angles, states, now):
        self.console.apply_pending(self.leader)
        self.console.refresh(self.leader)
        if self.console.stop_requested:
            return
        dt = now - self.last_step
        self.last_step = now
        valid = (
            leader_angles is not None
            and set(leader_angles) == set(range(7))
            and set(states) == set(range(7))
        )
        if valid:
            valid = all(math.isfinite(v) for v in leader_angles.values()) and all(
                all(math.isfinite(v) for v in (st.pos, st.vel, st.torque))
                and 0 <= now - st.received_at <= self.profile.data["link_timeout_s"]
                and st.sequence > self.last_sequences.get(i, 0)
                for i, st in states.items()
            )
        if not valid:
            self.console.stall._windows.clear()
            self.console.stall._release_since.clear()
            self.console.stall._over = dict.fromkeys(range(7), 0)
            if now - self.last_good >= self.profile.data["link_timeout_s"]:
                self.pause("通信反馈超时")
            # No new motion targets are sent from missing or stale samples.
            if self.paused:
                self.follower.command(self.commanded)
            return
        self.latest = dict(states)
        self.last_sequences = {i: st.sequence for i, st in states.items()}
        self.last_good = now
        bad = [
            i
            for i, st in states.items()
            if st.status != 1
            or st.mos_c >= 80
            or st.rotor_c >= 80
            or not self.profile.joints[i].lower_rad - FEEDBACK_LIMIT_MARGIN_RAD
            <= st.pos
            <= self.profile.joints[i].upper_rad + FEEDBACK_LIMIT_MARGIN_RAD
        ]
        if bad:
            self.resume_requested = False
            self.pause("电机状态/温度/限位异常: " + ",".join(f"J{i+1}" for i in bad))
        if self.resume_requested:
            self.resume_requested = False
            self.commanded = bounded_positions(self.profile, states)
            self.mapper.rebase(leader_angles, self.commanded)
            self.console.release_stalled()
            self.paused, self.reason = False, ""
            self.console.emit("[跟随] 已重新配对")
        if self.paused:
            self.follower.command(self.commanded)
            return
        desired, limited = self.mapper.forward(leader_angles)
        if limited:
            self.pause("目标到达关节限位: " + ",".join(f"J{i+1}" for i in sorted(limited)))
            self.follower.command(self.commanded)
            return
        guard = self.console.stall
        previous_deg = self.mapper.inverse(self.commanded)
        for i in guard.locked:
            guard.follow_targets.setdefault(i, previous_deg[i])
        desired_deg = self.mapper.inverse(desired)
        for i in guard.locked:
            desired_deg[i] = guard.follow_targets[i]
        desired, _ = self.mapper.forward(desired_deg)
        targets = self.mapper.slew(desired, self.commanded, dt)
        actual_deg = self.mapper.inverse({i: st.pos for i, st in states.items()})
        target_deg = self.mapper.inverse(targets)
        self.console.process_sample(
            self.leader, target_deg, actual_deg, {i: st.torque for i, st in states.items()}, now
        )
        # Freeze newly locked joints at the same command used by stall detection.
        self.follower.command(targets)
        self.commanded = targets


class SimLeader:
    def __init__(self):
        self.servos = {i: SimpleNamespace(angle_monitor=0.0) for i in range(7)}
        self.calls = []

    def stop_on_control_mode(self, *args):
        self.calls.append(("stop", args))

    def set_damping(self, *args):
        self.calls.append(("damping", args))

    def send_sync_multiturnanglebyinterval(self, *args):
        self.calls.append(("hold", args))


class SimFollower:
    def __init__(self, joints):
        self.joints = joints
        self.targets = {j.leader_id: j.follower_zero_rad for j in joints}
        self.positions = dict(self.targets)

    def command(self, targets):
        self.targets = dict(targets)

    def states(self, now, stalled=None):
        for i, target in self.targets.items():
            if i != stalled:
                self.positions[i] += 0.5 * (target - self.positions[i])
        return {
            i: State(
                p,
                0,
                self.joints[i].torque_reference_nm if i == stalled else 0.02,
                1,
                25,
                25,
                now,
                int(now * 1000) + 1,
            )
            for i, p in self.positions.items()
        }


def simulate(profile, duration, stalled=None):
    leader, follower = SimLeader(), SimFollower(profile.joints)
    session = Session(profile, leader, follower, follower.states(0), 0)
    session.console.emit = lambda message: print(message)
    dt = 1 / profile.data["loop_hz"]
    for tick in range(1, int(duration / dt) + 1):
        now = tick * dt
        angles = {j.leader_id: j.leader_zero_deg + min(50.0, now * 25.0) for j in profile.joints}
        for i, value in angles.items():
            leader.servos[i].angle_monitor = value
        session.step(angles, follower.states(now, stalled), now)
    print(
        f"[模拟] {duration:g}s | 锁定次数 {session.console.stall.lock_count}"
        f' | 状态 {session.reason if session.paused else ("LOCKED" if session.console.stall.locked else "FOLLOW")} | 未访问串口'
    )
    return session


def _initial_states(profile, follower):
    follower.verify()
    states = follower.read(profile.data["read_timeout_s"])
    if set(states) != set(range(7)):
        raise LinkError("startup: missing B601 feedback")
    for i, st in states.items():
        j = profile.joints[i]
        if st.status not in (0, 1) or st.mos_c >= 80 or st.rotor_c >= 80:
            raise LinkError(f"J{i+1}: startup motor fault")
        if (
            not j.lower_rad - FEEDBACK_LIMIT_MARGIN_RAD
            <= st.pos
            <= j.upper_rad + FEEDBACK_LIMIT_MARGIN_RAD
        ):
            raise LinkError(f"J{i+1}: actual position outside configured limits")
    return states


def hardware(profile, capture=None):
    data = profile.data
    if capture is None and not data["calibration_verified"]:
        raise ValueError("set calibration_verified=true after verifying mapping and joint limits")
    leader_uart = follower_uart = leader = follower = console = None
    enabled = False
    try:
        leader_uart = serial.Serial(data["leader_port"], 1000000, timeout=0, write_timeout=0.1)
        leader = uservo.UartServoManager(leader_uart)
        follower_uart = serial.Serial(
            data["follower_port"], data["follower_baud"], timeout=0, write_timeout=0.1
        )
        follower = DMSerial(follower_uart, profile.joints)
        states = _initial_states(profile, follower)
        ids = list(range(7)) + ([data["button_id"]] if data["leader_type"] == "102HD" else [])
        monitors = legacy._read_monitors(leader, ids)
        if any(getattr(monitors.get(i), "angle_monitor", None) is None for i in ids):
            raise LinkError("startup: missing Leader feedback")
        angles = {i: monitors[i].angle_monitor for i in range(7)}
        if capture is not None:
            captured = copy.deepcopy(data)
            captured["calibration_verified"] = False
            for row in captured["joints"]:
                i = row["leader_id"]
                row["leader_zero_deg"] = angles[i]
                row["follower_zero_rad"] = bounded_positions(profile, states)[i]
            Path(capture).write_text(
                json.dumps(captured, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
            )
            print(f"[标定] 已保存配对姿态：{capture}；方向和限位仍需人工验证")
            return
        if any(st.status != 0 for st in states.values()):
            raise LinkError("B601 must be disabled before starting this controller")
        mapper = JointMap(profile.joints)
        desired, limited = mapper.forward(angles)
        actual = mapper.inverse({i: st.pos for i, st in states.items()})
        if limited or any(
            abs(actual[i] - angles[i]) > data["max_start_error_deg"] for i in range(7)
        ):
            raise LinkError("startup pose does not match calibration; align arms before enabling")
        follower.command(bounded_positions(profile, states))
        enabled = True
        follower.enable()
        follower.command(bounded_positions(profile, states))
        states = follower.read(data["read_timeout_s"])
        if set(states) != set(range(7)) or any(st.status != 1 for st in states.values()):
            raise LinkError("B601 enable acknowledgement missing")
        leader.stop_on_control_mode(0xFF, legacy.METHOD_RELEASE, 0)
        session = Session(profile, leader, follower, states, time.monotonic())
        console = session.console
        console.start()
        console._ready.wait(1)
        console.emit("StarArm Leader → B601-DM | h 帮助 · stop 保持 · q 退出")
        period = 1 / data["loop_hz"]
        get_frequency = legacy.measure_frequency()
        while not console.stop_requested:
            t0 = time.monotonic()
            console.refresh(leader)
            monitors = legacy._read_monitors(leader, ids)
            angles = (
                {i: monitors[i].angle_monitor for i in range(7)}
                if all(getattr(monitors.get(i), "angle_monitor", None) is not None for i in ids)
                else None
            )
            if angles is not None and data["leader_type"] == "102HD":
                session.button(monitors[data["button_id"]].angle_monitor == 180)
            try:
                states = follower.read(data["read_timeout_s"])
            except (serial.SerialException, LinkError) as exc:
                states = {}
                session.pause(f"从臂通信异常: {exc}")
            session.step(angles, states, time.monotonic())
            hz = get_frequency()
            if hz is not None:
                console.set_frequency(f"{hz:.1f}Hz")
            time.sleep(max(0.0, period - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        print("\n[退出]")
    finally:
        if console is not None:
            console.close()
        if enabled and follower is not None:
            try:
                follower.disable()
            except Exception as exc:
                print(f"[错误] 601 失能失败: {exc}", file=sys.stderr)
        if enabled and leader is not None:
            try:
                leader.stop_on_control_mode(0xFF, legacy.METHOD_RELEASE, 0)
            except Exception as exc:
                print(f"[错误] Leader 卸力失败: {exc}", file=sys.stderr)
        for uart in (follower_uart, leader_uart):
            if uart is not None:
                uart.close()


def main(argv=None):
    parser = argparse.ArgumentParser(description="StarArm Leader / Seeed B601-DM")
    parser.add_argument("--config", type=Path, default=DEFAULT_PROFILE)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--simulate", action="store_true", help="不访问硬件，默认模式")
    modes.add_argument("--check", action="store_true", help="仅校验配置")
    modes.add_argument("--capture", type=Path, metavar="OUTPUT_JSON", help="只读采集主从配对姿态")
    modes.add_argument("--live", action="store_true", help="连接并使能实机")
    parser.add_argument("--duration", type=float, default=5.0, help="模拟时长 s")
    parser.add_argument(
        "--simulate-stall-joint", type=int, choices=range(7), help="模拟堵转，Leader ID"
    )
    args = parser.parse_args(argv)
    try:
        profile = Profile.load(args.config)
        number(args.duration, "duration", 0.1, 3600)
        if args.check:
            print(f'[配置] 7 关节有效 | 实机标定确认 {profile.data["calibration_verified"]}')
        elif args.live or args.capture is not None:
            hardware(profile, args.capture)
        else:
            simulate(profile, args.duration, args.simulate_stall_joint)
        return 0
    except (ValueError, KeyError, TypeError, OSError, LinkError) as exc:
        print(f"[错误] {exc}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    if os.name == "nt":
        for stream in (sys.stdout, sys.stderr):
            if hasattr(stream, "reconfigure"):
                stream.reconfigure(encoding="utf-8", errors="replace")
    raise SystemExit(main())
