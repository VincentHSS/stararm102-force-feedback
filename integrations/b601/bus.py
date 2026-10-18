"""直接 UART/CAN 通信：每轮只接收新回包，连续零位也是有效数据。"""

from __future__ import annotations

import math
import struct
import time
from dataclasses import dataclass


class LinkError(RuntimeError):
    pass


def write_all(uart, data):
    if uart.write(data) != len(data):
        raise LinkError("Serial short write")


class Leader:
    def __init__(self, uart, cfg, clock=time.perf_counter, sleep=time.sleep):
        self.uart, self.cfg, self.clock, self.sleep = uart, cfg, clock, sleep
        self.buffer = bytearray()
        self.previous = {}

    def send(self, code, payload):
        body = b"\x12\x4c" + bytes((code, len(payload))) + payload
        write_all(self.uart, body + bytes((sum(body) & 255,)))

    def mode(self, servo_id, mode, power):
        self.send(24, struct.pack("<BBH", servo_id, mode, power))

    def read(self):
        ids = list(range(7)) + ([] if self.cfg.button_id is None else [self.cfg.button_id])
        # 丢弃上轮残留；不使用 SDK 的可靠性零值启发式，也不复用旧角度。
        self.buffer.clear()
        self.uart.reset_input_buffer()
        self.send(25, bytes((22, 1, len(ids), *ids)))
        deadline = self.clock() + self.cfg.read_timeout_s
        angles = {}
        while self.clock() < deadline:
            chunk = self.uart.read(min(self.uart.in_waiting, 4096))
            if chunk:
                self.buffer.extend(chunk)
            while len(self.buffer) >= 5:
                if self.buffer[:2] != b"\x05\x1c":
                    del self.buffer[0]
                    continue
                size = self.buffer[3] + 5
                if size > 64:
                    del self.buffer[0]
                    continue
                if len(self.buffer) < size:
                    break
                raw = bytes(self.buffer[:size])
                if sum(raw[:-1]) & 255 != raw[-1]:
                    del self.buffer[0]
                    continue
                del self.buffer[:size]
                if raw[2] != 22 or raw[3] != 16:
                    continue
                servo_id, _, _, _, _, _, angle, turn = struct.unpack("<BHHHHBih", raw[4:-1])
                if servo_id not in ids or (angle == -235929599 and turn == 0):
                    continue
                value, now = angle / 10.0, self.clock()
                if servo_id != self.cfg.button_id and servo_id in self.previous:
                    old, last = self.previous[servo_id]
                    allowed = self.cfg.input_jump_margin_deg + self.cfg.input_max_speed_deg_s * min(
                        max(now - last, 0), 0.25
                    )
                    if abs(value - old) > allowed:
                        continue
                # 有完整校验通过的回包才更新，value==0 不需要与上次不同。
                angles[servo_id] = value
            if len(angles) == len(ids):
                now = self.clock()
                self.previous.update({i: (v, now) for i, v in angles.items()})
                return [angles[i] for i in range(7)], bool(
                    self.cfg.button_id is not None and angles[self.cfg.button_id] == 180
                )
            self.sleep(0.0003)
        return None

    def release(self):
        errors = []
        for i in range(7):
            try:
                self.mode(i, 0x10, 0)
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise LinkError("Leader release: " + "; ".join(errors))


class SerialCAN:
    def __init__(self, uart):
        self.uart = uart
        self.buffer = bytearray()

    def send(self, can_id, payload):
        raw = bytearray(30)
        raw[:4] = b"\x55\xaa\x1e\x03"
        struct.pack_into("<II", raw, 4, 1, 10)
        struct.pack_into("<I", raw, 13, can_id)
        raw[18] = 8
        raw[21:29] = payload
        write_all(self.uart, raw)

    def poll(self):
        data = self.uart.read(min(self.uart.in_waiting, 65536))
        self.buffer.extend(data)
        result = []
        while len(self.buffer) >= 16:
            if self.buffer[0] != 0xAA or self.buffer[15] != 0x55:
                del self.buffer[0]
                continue
            raw = bytes(self.buffer[:16])
            del self.buffer[:16]
            if raw[1:3] == b"\x11\x08":
                result.append((struct.unpack_from("<I", raw, 3)[0], raw[7:15]))
        return result

    def close(self):
        self.uart.close()


class SocketCAN:
    def __init__(self, channel):
        import can

        self.can = can
        self.bus = can.Bus(channel=channel, interface="socketcan")

    def send(self, can_id, payload):
        self.bus.send(
            self.can.Message(arbitration_id=can_id, data=payload, is_extended_id=False),
            timeout=0.05,
        )

    def poll(self):
        result = []
        for _ in range(1024):
            frame = self.bus.recv(timeout=0)
            if frame is None:
                break
            if (
                not (frame.is_extended_id or frame.is_remote_frame or frame.is_error_frame)
                and frame.dlc == 8
            ):
                result.append((frame.arbitration_id, bytes(frame.data)))
        return result

    def close(self):
        self.bus.shutdown()


@dataclass(frozen=True)
class State:
    pos: float
    vel: float
    torque: float
    status: int
    mos_c: float
    rotor_c: float
    received_at: float
    sequence: int


class Follower:
    def __init__(self, transport, cfg, clock=time.perf_counter, sleep=time.sleep):
        self.transport, self.cfg, self.clock, self.sleep = transport, cfg, clock, sleep
        self.by_feedback = {j.feedback_id: j for j in cfg.joints}
        self.states, self.replies = {}, {}
        self.pending = set()
        self.sequence = 0
        self.accepted = {}
        self.limits = {i: (12.5, 10.0, 28.0) if i < 3 else (12.5, 30.0, 10.0) for i in range(7)}

    def poll(self):
        for can_id, data in self.transport.poll():
            j = self.by_feedback.get(can_id)
            if j is None or len(data) != 8:
                continue
            if (
                data[0] == j.motor_id
                and data[1] <= 15
                and data[2] in (0x33, 0x55)
                and data[3] in (10, 21, 22, 23)
            ):
                key = (j.leader_id, data[3])
                if data[2] == 0x33 and key in self.pending:
                    self.replies[key] = data[4:8]
                continue
            if data[0] & 15 != j.motor_id:
                continue
            pmax, vmax, tmax = self.limits[j.leader_id]
            pos = int.from_bytes(data[1:3], "big") / 65535 * 2 * pmax - pmax
            vel = ((data[3] << 4) | (data[4] >> 4)) / 4095 * 2 * vmax - vmax
            torque = (((data[4] & 15) << 8) | data[5]) / 4095 * 2 * tmax - tmax
            self.sequence += 1
            self.states[j.leader_id] = State(
                pos, vel, torque, data[0] >> 4, data[6], data[7], self.clock(), self.sequence
            )

    def register(self, joint, rid, timeout=0.3):
        self.poll()
        key = (joint.leader_id, rid)
        self.replies.pop(key, None)
        self.pending.add(key)
        try:
            self.transport.send(0x7FF, struct.pack("<HBBI", joint.motor_id, 0x33, rid, 0))
            deadline = self.clock() + timeout
            while self.clock() < deadline:
                self.poll()
                if key in self.replies:
                    return self.replies.pop(key)
                self.sleep(0.0005)
            raise LinkError(f"J{joint.leader_id + 1}: register {rid} timeout")
        finally:
            self.pending.discard(key)

    def verify(self):
        # 模式与编码量程只读校验，不改电机零位、Flash 或控制模式。
        for j in self.cfg.joints:
            expected = 4 if j.leader_id == 6 else 2
            if struct.unpack("<I", self.register(j, 10))[0] != expected:
                raise LinkError(
                    f"J{j.leader_id + 1}: expected mode {expected}; configure POS_VEL / gripper FORCE_POS with vendor tools"
                )
            limits = tuple(struct.unpack("<f", self.register(j, rid))[0] for rid in (21, 22, 23))
            if any(not math.isfinite(v) or v <= 0 for v in limits):
                raise LinkError("Invalid DM encoder scaling")
            if (
                max(abs(math.radians(j.lower_deg)), abs(math.radians(j.upper_deg))) > limits[0]
                or math.radians(j.motor_speed) > limits[1]
            ):
                raise LinkError("Configured limits exceed motor protocol range")
            self.limits[j.leader_id] = limits

    def read(self):
        self.poll()
        before = self.sequence
        for j in self.cfg.joints:
            self.transport.send(0x7FF, struct.pack("<HBBI", j.motor_id, 0xCC, 0, 0))
        deadline = self.clock() + self.cfg.read_timeout_s
        while self.clock() < deadline:
            self.poll()
            fresh = {i: st for i, st in self.states.items() if st.sequence > before}
            if len(fresh) == 7:
                bad = False
                for i, st in fresh.items():
                    if i in self.accepted:
                        old, last = self.accepted[i]
                        allowed = math.radians(
                            5
                            + 2
                            * self.cfg.joints[i].motor_speed
                            * min(max(st.received_at - last, 0), 0.25)
                        )
                        if abs(st.pos - old) > allowed:
                            bad = True
                if bad:
                    self.sleep(0.0003)
                    continue
                self.accepted.update({i: (s.pos, s.received_at) for i, s in fresh.items()})
                return [fresh[i] for i in range(7)]
            self.sleep(0.0003)
        return None

    def command(self, targets):
        if len(targets) != 7:
            raise ValueError("Expected seven targets")
        packets = []
        for j, target in zip(self.cfg.joints, targets):
            if (
                not math.isfinite(target)
                or not math.radians(j.lower_deg) - 1e-6
                <= target
                <= math.radians(j.upper_deg) + 1e-6
            ):
                raise ValueError("Invalid/out-of-range command")
            # 目标轨迹仍按 speed_deg_s 限速；电机速度单独留出追赶位置误差的余量。
            speed = math.radians(j.motor_speed)
            if j.leader_id == 6:
                payload = struct.pack(
                    "<fHH", target, int(speed * 100), int(self.cfg.gripper_force_ratio * 10000)
                )
                packets.append((j.motor_id + 0x300, payload))
            else:
                packets.append((j.motor_id + 0x100, struct.pack("<ff", target, speed)))
        # 先验证整批数据，避免最后一个关节无效时前几个已收到运动指令。
        for can_id, payload in packets:
            self.transport.send(can_id, payload)

    def enable(self):
        for j in self.cfg.joints:
            self.transport.send(j.motor_id, b"\xff" * 7 + b"\xfc")

    def disable(self):
        errors = []
        for j in self.cfg.joints:
            try:
                self.transport.send(j.motor_id, b"\xff" * 7 + b"\xfd")
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise LinkError("Follower disable: " + "; ".join(errors))
