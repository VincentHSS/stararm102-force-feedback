"""仅检查主臂阻尼输出；从臂只读核对失能，不发送使能或位置指令。"""

import csv
import logging
import time
from contextlib import ExitStack
from pathlib import Path

from bus import Follower, Leader, LinkError, SerialCAN

logger = logging.getLogger(__name__)


def run_check(cfg, output, serial_factory, clock=time.perf_counter, sleep=time.sleep):
    with serial_factory(
        cfg.follower_port, cfg.follower_baud, timeout=0, write_timeout=0.05
    ) as wire:
        follower = Follower(SerialCAN(wire), cfg, clock=clock, sleep=sleep)
        follower.verify()
        states, deadline = None, clock() + cfg.link_timeout_s
        while states is None and clock() < deadline:
            states = follower.read()
        if states is None or any(state.status != 0 for state in states):
            raise LinkError("Leader check requires all follower motors disabled")
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with ExitStack() as stack:
        uart = stack.enter_context(
            serial_factory(cfg.leader_port, 1000000, timeout=0, write_timeout=0.05)
        )
        leader = Leader(uart, cfg, clock=clock, sleep=sleep)
        stack.callback(leader.release)
        stream = stack.enter_context(path.open("x", encoding="utf-8", newline=""))
        writer = csv.writer(stream)
        writer.writerow(
            [
                "elapsed_s",
                "phase",
                "command_mw",
                "valid",
                "j2_deg",
                "j3_deg",
                "j2_reported_mw",
                "j3_reported_mw",
            ]
        )
        started = last_good = clock()
        phases = (("free", 1, 5), ("damping", min(1000, cfg.power_max_mw), 12), ("free", 1, 4))
        for phase, power, seconds in phases:
            logger.info("主臂检查 | %s | J2/J3 %dmW | %d秒 | 从臂保持失能", phase, power, seconds)
            until, refreshed = clock() + seconds, -float("inf")
            while clock() < until:
                now = clock()
                if now - refreshed >= cfg.damping_refresh_s:
                    for servo_id in (1, 2):
                        leader.mode(servo_id, 0x12, power)
                    refreshed = now
                sample = leader.read()
                if sample is not None:
                    last_good = clock()
                elif clock() - last_good >= cfg.link_timeout_s:
                    raise LinkError("Leader feedback timeout during damping check")
                writer.writerow(
                    [
                        clock() - started,
                        phase,
                        power,
                        int(sample is not None),
                        *([sample[0][1], sample[0][2]] if sample else [float("nan")] * 2),
                        *[
                            leader.telemetry.get(i, {}).get("power_mw", float("nan"))
                            for i in (1, 2)
                        ],
                    ]
                )
                sleep(0.02)
            stream.flush()
        logger.info("主臂阻尼检查完成，正在释放；从臂未使能。")
