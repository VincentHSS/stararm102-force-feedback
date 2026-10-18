"""StarArm-102 → B601-DM 独立 Python 遥操作；不加载 LeRobot 或 MotorBridge SDK。"""

from __future__ import annotations

import argparse
import copy
import csv
import json
import logging
import math
import statistics
import time
from contextlib import ExitStack
from pathlib import Path

from bus import Follower, Leader, LinkError, SerialCAN, SocketCAN, State
from config import Config
from controller import Motion, Session

logger = logging.getLogger(__name__)
DEFAULT_CONFIG = Path(__file__).with_name("b601.example.json")


def check_states(cfg, states, enabled=None):
    if states is None or len(states) != 7:
        raise LinkError("Incomplete B601 feedback")
    for j, state in zip(cfg.joints, states):
        if (
            not all(
                math.isfinite(v)
                for v in (state.pos, state.vel, state.torque, state.mos_c, state.rotor_c)
            )
            or state.status not in ((0, 1) if enabled is None else (int(enabled),))
            or max(state.mos_c, state.rotor_c) >= cfg.motor_temperature_c
            or not j.lower_deg - 0.1 <= math.degrees(state.pos) <= j.upper_deg + 0.1
        ):
            raise LinkError(f"J{j.leader_id + 1}: invalid position/status/temperature")


def bounded(cfg, states):
    return [
        min(math.radians(j.upper_deg), max(math.radians(j.lower_deg), s.pos))
        for j, s in zip(cfg.joints, states)
    ]


def baseline(cfg, follower, targets, clock=time.perf_counter, sleep=time.sleep):
    started, last_good = clock(), clock()
    rows = []
    while clock() - started < cfg.baseline_s:
        tick = clock()
        states = follower.read()
        if states is None:
            if clock() - last_good > cfg.link_timeout_s:
                raise LinkError("Baseline feedback timeout")
        else:
            check_states(cfg, states, enabled=True)
            if any(abs(s.pos - target) > math.radians(2) for s, target in zip(states, targets)):
                raise LinkError(
                    "B601 moved during baseline collection; restart in a stable, unloaded pose"
                )
            rows.append([abs(s.torque) for s in states])
            last_good = clock()
            follower.command(targets)
        sleep(max(0, 1 / cfg.loop_hz - (clock() - tick)))
    if len(rows) < 3:
        raise LinkError("Insufficient baseline feedback")
    return [statistics.median(row[i] for row in rows) for i in range(7)]


def hardware(
    cfg,
    *,
    execute=False,
    capture=None,
    diagnostics=None,
    collect_unloaded=False,
    duration=0,
    serial_factory=None,
    socket_factory=SocketCAN,
    # Windows Python <=3.12 的 monotonic 可只有 15.625ms 精度；QPC 避免 dt=0/16ms。
    clock=time.perf_counter,
    sleep=time.sleep,
):
    cfg.validate()
    if execute and not cfg.calibration_verified:
        raise ValueError(
            "Capture/verify paired pose and set calibration_verified=true before --execute"
        )
    if capture is not None and Path(capture).exists():
        raise FileExistsError(f"Capture output already exists: {capture}")
    observer = None
    if collect_unloaded:
        if not execute or diagnostics is None or capture is not None:
            raise ValueError("Unloaded collection requires execute, a new CSV and no pose capture")
        path = Path(diagnostics)
        metadata_path = path.with_suffix(path.suffix + ".meta.json")
        if path.exists() or metadata_path.exists():
            raise FileExistsError("Unloaded collection must use new CSV and metadata paths")
        from payload import signature

        path.parent.mkdir(parents=True, exist_ok=True)
    elif cfg.feedback_mode == "payload" and capture is None:
        from payload import Calibration, PayloadObserver

        # 标定、模型文件与配对签名都在打开设备前校验。
        observer = PayloadObserver(cfg, Calibration.load(cfg.payload_calibration, cfg))
    if serial_factory is None:
        from serial import Serial

        serial_factory = Serial
    with ExitStack() as stack:
        uart = serial_factory(cfg.leader_port, 1000000, timeout=0, write_timeout=0.05)
        stack.callback(uart.close)
        leader = Leader(uart, cfg, clock, sleep)
        if cfg.adapter == "damiao":
            wire = serial_factory(
                cfg.follower_port, cfg.follower_baud, timeout=0, write_timeout=0.05
            )
            transport = SerialCAN(wire)
        else:
            transport = socket_factory(cfg.follower_port)
        stack.callback(transport.close)
        follower = Follower(transport, cfg, clock, sleep)
        follower.verify()
        initial = follower.read()
        check_states(cfg, initial)
        first = leader.read()
        if first is None:
            raise LinkError("Incomplete Leader startup feedback")
        angles, _ = first
        if capture is not None:
            paired = copy.deepcopy(cfg)
            for j, angle, position in zip(paired.joints, angles, bounded(cfg, initial)):
                j.leader_home_deg, j.follower_home_rad = angle, position
            paired.calibration_verified = False
            paired.save(capture)
            logger.info("Paired pose saved: %s; verify directions/limits before execute", capture)
            return
        targets = bounded(cfg, initial)
        motion = Motion(cfg, targets)
        if execute:
            check_states(cfg, initial, enabled=False)
            mapped = [
                j.follower_home_rad + j.direction * math.radians(a - j.leader_home_deg)
                for j, a in zip(cfg.joints, angles)
            ]
            if any(
                abs(math.degrees((goal - actual) / j.direction)) > cfg.startup_error_deg
                for j, goal, actual in zip(cfg.joints, mapped, targets)
            ):
                raise LinkError("Startup pose differs from captured pairing")
            # 零位附近的微小漂移可以略超软限位；仍检查原始姿态误差，实际命令始终限位钳制。
            # 先登记所有清理动作，再使能；部分使能失败也会尝试逐个断使能。
            stack.callback(leader.release)
            stack.callback(follower.disable)
            follower.command(targets)
            follower.enable()
            follower.command(targets)
            initial = follower.read()
            check_states(cfg, initial, enabled=True)
            tare = baseline(cfg, follower, targets, clock, sleep)
            initial = follower.read()
            check_states(cfg, initial, enabled=True)
            session = Session(
                cfg,
                leader,
                follower,
                initial,
                clock(),
                baseline=tare,
                payload_observer=observer,
                calibrating=collect_unloaded,
            )
        else:
            session = None
        stream, writer = None, None
        if diagnostics is not None:
            path = Path(diagnostics)
            path.parent.mkdir(parents=True, exist_ok=True)
            stream = stack.enter_context(
                path.open("x" if collect_unloaded else "w", encoding="utf-8", newline="")
            )
            if collect_unloaded:
                with metadata_path.open("x", encoding="utf-8") as metadata_stream:
                    json.dump(
                        {"confirmed_unloaded": True, "config_signature": signature(cfg)},
                        metadata_stream,
                        indent=2,
                    )
            writer = csv.writer(stream)
            writer.writerow(
                [
                    "elapsed_s",
                    "dt_ms",
                    "leader_ms",
                    "follower_ms",
                    "work_ms",
                    "valid",
                    "state",
                    "max_error_deg",
                ]
                + [f"target_j{i}_rad" for i in range(1, 8)]
                + [f"actual_j{i}_rad" for i in range(1, 8)]
                + [f"torque_j{i}_nm" for i in range(1, 8)]
                + [f"baseline_j{i}_nm" for i in range(1, 8)]
                + [f"load_j{i}" for i in range(1, 8)]
                + [f"leader_j{i}_deg" for i in range(1, 8)]
                + [f"filtered_j{i}_deg" for i in range(1, 8)]
                + [f"desired_j{i}_rad" for i in range(1, 8)]
                + [f"reference_j{i}_rad" for i in range(1, 8)]
                + ["leader_valid", "follower_valid", "fault_reason"]
                + [f"velocity_j{i}_rad_s" for i in range(1, 8)]
                + [
                    "unloaded_collection",
                    "payload_kg",
                    "payload_raw_kg",
                    "payload_status",
                    "payload_fit_score",
                    "payload_sigma_kg",
                ]
                + [f"external_j{i}_nm" for i in range(1, 7)]
                + [f"haptic_j{i}" for i in range(1, 8)]
            )
        logger.info("%s | Python UART/CAN | Ctrl+C to stop", "EXECUTE" if execute else "PREVIEW")
        started = last_tick = report = clock()
        loops = sent = 0
        while not duration or clock() - started < duration:
            tick = clock()
            lead = leader.read()
            after_leader = clock()
            states = follower.read()
            now = clock()
            dt, last_tick = now - last_tick, now
            valid = lead is not None and states is not None
            if execute:
                valid = session.tick(
                    None if lead is None else lead[0],
                    states,
                    now,
                    pressed=False if lead is None else lead[1],
                    invalid_reason="; ".join(
                        message for message in (leader.last_error, follower.last_error) if message
                    ),
                )
                targets = session.motion.commanded
                state = session.feedback.state
                sent += int(valid)
            else:
                state = "preview"
                if valid:
                    check_states(cfg, states)
                    targets = motion.step(lead[0], dt)
                    if observer is not None:
                        observer.update(
                            [s.pos for s in states],
                            [s.vel for s in states],
                            [s.torque for s in states],
                            now,
                        )
                elif observer is not None:
                    observer.gap(now)
            work_end = clock()
            loops += 1
            actual = [s.pos for s in states] if states else [math.nan] * 7
            torques = [s.torque for s in states] if states else [math.nan] * 7
            error = (
                max(
                    abs(a - b)
                    for a, b in zip(
                        motion.inverse(session.motion.commanded if session else targets),
                        motion.inverse(actual),
                    )
                )
                if states
                else math.nan
            )
            if writer is not None:
                writer.writerow(
                    [
                        now - started,
                        dt * 1000,
                        (after_leader - tick) * 1000,
                        (now - after_leader) * 1000,
                        (work_end - tick) * 1000,
                        int(valid),
                        state,
                        error,
                    ]
                    + list(targets)
                    + actual
                    + torques
                    + (
                        session.feedback.baseline + session.feedback.loads
                        if session
                        else [math.nan] * 14
                    )
                    + (list(lead[0]) if lead else [math.nan] * 7)
                    + list((session.motion if session else motion).filtered or [math.nan] * 7)
                    + (
                        (session.motion if session else motion).forward(lead[0])
                        if lead
                        else [math.nan] * 7
                    )
                    + list(session.motion.commanded if session else targets)
                    + [
                        int(lead is not None),
                        int(states is not None),
                        session.fault_reason if session else "",
                    ]
                    + ([s.vel for s in states] if states else [math.nan] * 7)
                    + [int(collect_unloaded)]
                    + (
                        [
                            observer.mass_kg,
                            observer.raw_mass_kg,
                            observer.status,
                            observer.score,
                            observer.sigma_kg,
                        ]
                        if observer
                        else [
                            math.nan,
                            math.nan,
                            "unloaded collection" if collect_unloaded else "legacy",
                            math.nan,
                            math.nan,
                        ]
                    )
                    + (observer.residual_nm if observer else [math.nan] * 6)
                    + (
                        observer.loads
                        if observer
                        else (
                            [0.0] * 7
                            if collect_unloaded
                            else (session.feedback.loads if session else [math.nan] * 7)
                        )
                    )
                )
            if work_end - report >= 2.0:
                logger.info(
                    "%s loop=%.1fHz sent=%.1fHz error=%.1fdeg%s",
                    state,
                    loops / (work_end - report),
                    sent / (work_end - report),
                    error,
                    f" payload={observer.mass_kg:.2f}kg ({observer.status})" if observer else "",
                )
                loops = sent = 0
                report = work_end
                if stream is not None:
                    stream.flush()
            sleep(max(0, 1 / cfg.loop_hz - (clock() - tick)))
        return session


class SimLeader:
    def __init__(self):
        self.calls = []

    def mode(self, *args):
        self.calls.append(args)

    def release(self):
        for i in range(7):
            self.mode(i, 0x10, 0)


class SimFollower:
    def __init__(self, cfg, stalled=None, calibration=None, payload_kg=0.0):
        self.cfg, self.stalled = cfg, stalled
        self.positions = [j.follower_home_rad for j in cfg.joints]
        self.targets = list(self.positions)
        self.sequence = 0
        self.calibration, self.payload_kg = calibration, payload_kg

    def command(self, targets):
        self.targets = list(targets)

    def states(self, now):
        result = []
        for i, (position, target) in enumerate(zip(self.positions, self.targets)):
            torque = 0.1
            if i == self.stalled:
                torque += self.cfg.joints[i].torque_reference_nm
            else:
                self.positions[i] += 0.3 * (target - position)
            self.sequence += 1
            result.append(State(self.positions[i], 0, torque, 1, 25, 25, now, self.sequence))
        if self.calibration is not None:
            from gravity import dot
            from payload import friction_features

            values = self.calibration.model.evaluate(self.positions[:6])
            for i in range(6):
                row, old = self.calibration.coefficients[i], result[i]
                torque = (
                    dot(row, friction_features(values["gravity"][i], 0))
                    + row[1] * self.payload_kg * values["unit_mass"][i]
                )
                if i == self.stalled:
                    torque += self.cfg.joints[i].torque_reference_nm
                result[i] = State(old.pos, 0, torque, 1, 25, 25, now, old.sequence)
        return result


def simulate(cfg, duration=5, stalled=None, payload_kg=0.0):
    observer, calibration = None, None
    if cfg.feedback_mode == "payload":
        from payload import Calibration, PayloadObserver

        calibration = Calibration.load(cfg.payload_calibration, cfg)
        observer = PayloadObserver(cfg, calibration)
    elif payload_kg:
        raise ValueError(
            "Payload simulation needs a payload configuration and validated calibration"
        )
    leader, follower = SimLeader(), SimFollower(cfg, stalled, calibration, payload_kg)
    session = Session(
        cfg, leader, follower, follower.states(0), 0, baseline=[0.1] * 7, payload_observer=observer
    )
    dt = 1 / cfg.loop_hz
    for tick in range(1, int(duration / dt) + 1):
        now = tick * dt
        angles = [j.leader_home_deg for j in cfg.joints]
        # shoulder_pan 默认反向，向负 Leader 方向移动以保持从臂目标在默认软限位内。
        angles[0] -= min(60.0, now * 30)
        session.tick(angles, follower.states(now), now)
    logger.info(
        "Simulation: %s, triggers=%s; no device opened",
        session.feedback.state,
        sorted(session.feedback.triggers),
    )
    if observer:
        logger.info(
            "Payload simulation: input=%.2fkg estimate=%.2fkg | %s",
            payload_kg,
            observer.mass_kg,
            observer.status,
        )
    return session


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="StarArm-102 -> B601-DM direct Python teleoperation"
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--execute", action="store_true", help="Enable B601 and send motion targets")
    mode.add_argument(
        "--capture", type=Path, help="Read/save paired pose without motion or zero reset"
    )
    mode.add_argument("--simulate", action="store_true", help="Run without serial/CAN devices")
    parser.add_argument(
        "--simulate-stall",
        type=int,
        choices=range(7),
        help="Stalled joint in simulation, Leader ID 0..6",
    )
    parser.add_argument(
        "--duration", type=float, default=0, help="Runtime seconds; simulation defaults to 5"
    )
    parser.add_argument("--diagnostics-file", type=Path)
    parser.add_argument(
        "--collect-unloaded",
        type=Path,
        help="Collect a new explicitly unloaded CSV while following; requires --execute --confirm-unloaded",
    )
    parser.add_argument(
        "--confirm-unloaded",
        action="store_true",
        help="Confirm empty gripper, no object/contact; collection suppresses ordinary damping",
    )
    parser.add_argument("--simulate-payload-kg", type=float, default=0.0)
    args = parser.parse_args(argv)
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("duration must be finite and nonnegative")
    if args.simulate_stall is not None and not args.simulate:
        parser.error("--simulate-stall requires --simulate")
    if args.collect_unloaded is not None and (
        not args.execute or not args.confirm_unloaded or args.diagnostics_file is not None
    ):
        parser.error(
            "--collect-unloaded requires --execute --confirm-unloaded; do not also set --diagnostics-file"
        )
    if args.confirm_unloaded and args.collect_unloaded is None:
        parser.error("--confirm-unloaded requires --collect-unloaded")
    if (
        not math.isfinite(args.simulate_payload_kg)
        or not 0 <= args.simulate_payload_kg <= 2
        or (args.simulate_payload_kg and not args.simulate)
    ):
        parser.error("--simulate-payload-kg requires --simulate and a mass within 0..2kg")
    logging.basicConfig(level=logging.INFO, format="%(message)s")
    try:
        cfg = Config.load(args.config)
        if args.simulate:
            simulate(cfg, args.duration or 5, args.simulate_stall, args.simulate_payload_kg)
        else:
            hardware(
                cfg,
                execute=args.execute,
                capture=args.capture,
                diagnostics=args.collect_unloaded or args.diagnostics_file,
                collect_unloaded=args.collect_unloaded is not None,
                duration=args.duration,
            )
        return 0
    except KeyboardInterrupt:
        logger.info("Stopped")
        return 0
    except Exception as exc:
        logger.error("Stopped: %s", exc)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
