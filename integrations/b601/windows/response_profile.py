"""本机响应配置：使用已复测的 ACC/DEC，正常退出后恢复原值。"""

import json
import logging
import os
import struct
import sys
import time
from pathlib import Path

from bus import Follower, SerialCAN
from config import Config

ROOT = Path(__file__).resolve().parent.parent / "logs"


def read_register(transport, motor, rid, timeout=0.3):
    transport.poll()
    transport.send(0x7FF, struct.pack("<HBBI", motor, 0x33, rid, 0))
    deadline = time.perf_counter() + timeout
    while time.perf_counter() < deadline:
        for can_id, data in transport.poll():
            if can_id == motor + 16 and data[:4] == bytes((motor, 0, 0x33, rid)):
                return struct.unpack("<f", data[4:])[0]
        time.sleep(0.0005)
    raise RuntimeError(f"J{motor}: register {rid} read timeout")


def write_verified(transport, motor, rid, value):
    if rid not in (4, 5):
        raise ValueError("Only ACC/DEC may be changed in this trial")
    transport.send(0x7FF, struct.pack("<HBBf", motor, 0x55, rid, value))
    time.sleep(0.01)
    if abs(read_register(transport, motor, rid) - value) > 1e-5:
        raise RuntimeError(f"J{motor}: register {rid} verification failed")


def apply_trial(cfg, original, serial_factory):
    with serial_factory(
        cfg.follower_port, cfg.follower_baud, timeout=0, write_timeout=0.05
    ) as uart:
        transport = SerialCAN(uart)
        follower = Follower(transport, cfg)
        follower.verify()
        states = follower.read()
        if states is None or any(state.status != 0 for state in states):
            raise RuntimeError("All motors must be disabled before changing ACC/DEC")
        # 完整备份写盘成功后才修改；写入前登记恢复项，覆盖部分写入失败的情况。
        for motor in range(1, 7):
            for rid in (4, 5):
                original.append((motor, rid, read_register(transport, motor, rid)))
        (ROOT / "acc_dec_original.json").write_text(
            json.dumps(original, indent=2), encoding="utf-8"
        )
        (ROOT / "profile.active").write_text("active", encoding="ascii")
        for motor, rid, _ in original:
            write_verified(transport, motor, rid, 4.0 if rid == 4 else -4.0)


def restore(cfg, original, serial_factory):
    if not original:
        return
    failures = []
    with serial_factory(
        cfg.follower_port, cfg.follower_baud, timeout=0, write_timeout=0.05
    ) as uart:
        transport = SerialCAN(uart)
        follower = Follower(transport, cfg)
        follower.disable()
        for motor, rid, value in original:
            try:
                write_verified(transport, motor, rid, value)
            except Exception as exc:
                failures.append(str(exc))
    if failures:
        raise RuntimeError("Restore failed: " + "; ".join(failures))
    logging.info("Original ACC/DEC restored and verified.")
