from __future__ import annotations

import json
import math
import struct
import time
from dataclasses import dataclass
from pathlib import Path


class LinkError(RuntimeError):
    pass


def number(value, name, minimum=None, maximum=None):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(f"{name}: expected finite number")
    if minimum is not None and value < minimum or maximum is not None and value > maximum:
        raise ValueError(f"{name}: out of range")
    return float(value)


def integer(value, name, minimum, maximum):
    result = number(value, name, minimum, maximum)
    if result != int(result):
        raise ValueError(f"{name}: expected integer")
    return int(result)


@dataclass(frozen=True)
class Joint:
    leader_id: int
    motor_id: int
    feedback_id: int
    model: str
    sign: int
    scale: float
    leader_zero_deg: float
    follower_zero_rad: float
    lower_rad: float
    upper_rad: float
    speed_rad_s: float
    torque_reference_nm: float
    torque_floor_nm: float

    @classmethod
    def parse(cls, row):
        if set(row) != set(cls.__dataclass_fields__):
            raise ValueError("joint fields differ from the documented schema")
        obj = cls(
            integer(row["leader_id"], "leader_id", 0, 6),
            integer(row["motor_id"], "motor_id", 1, 15),
            integer(row["feedback_id"], "feedback_id", 1, 0x7FF),
            row["model"],
            integer(row["sign"], "sign", -1, 1),
            number(row["scale"], "scale", 0.001, 20),
            number(row["leader_zero_deg"], "leader_zero_deg"),
            number(row["follower_zero_rad"], "follower_zero_rad"),
            number(row["lower_rad"], "lower_rad"),
            number(row["upper_rad"], "upper_rad"),
            number(row["speed_rad_s"], "speed_rad_s", 0.001, 2),
            number(row["torque_reference_nm"], "torque_reference_nm", 0.001),
            number(row["torque_floor_nm"], "torque_floor_nm", 0),
        )
        if obj.model not in ("4340P", "4310") or obj.sign not in (-1, 1):
            raise ValueError("unsupported motor model or sign")
        if (
            not obj.lower_rad < obj.upper_rad
            or not obj.lower_rad <= obj.follower_zero_rad <= obj.upper_rad
        ):
            raise ValueError("invalid joint limits or zero")
        if obj.torque_floor_nm >= obj.torque_reference_nm:
            raise ValueError("torque floor must be below reference")
        return obj


@dataclass(frozen=True)
class Profile:
    data: dict
    joints: tuple[Joint, ...]

    @classmethod
    def load(cls, path):
        data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
        allowed = {
            "schema_version",
            "calibration_verified",
            "leader_port",
            "leader_type",
            "button_id",
            "follower_port",
            "follower_baud",
            "loop_hz",
            "read_timeout_s",
            "link_timeout_s",
            "max_start_error_deg",
            "feedback",
            "stall",
            "joints",
        }
        if set(data) != allowed or data["schema_version"] != 1:
            raise ValueError("invalid profile schema")
        if not isinstance(data["calibration_verified"], bool):
            raise ValueError("calibration_verified must be boolean")
        if data["leader_type"] not in ("102LD", "102HD"):
            raise ValueError("leader_type must be 102LD or 102HD")
        for key in ("leader_port", "follower_port"):
            if not isinstance(data[key], str) or not data[key].strip():
                raise ValueError(f"{key} is required")
        if data["leader_port"].strip().casefold() == data["follower_port"].strip().casefold():
            raise ValueError("leader and follower must use different ports")
        integer(data["button_id"], "button_id", 7, 253)
        integer(data["follower_baud"], "follower_baud", 9600, 3000000)
        number(data["loop_hz"], "loop_hz", 1, 100)
        number(data["read_timeout_s"], "read_timeout_s", 0.001, 1)
        number(data["link_timeout_s"], "link_timeout_s", data["read_timeout_s"], 3)
        number(data["max_start_error_deg"], "max_start_error_deg", 0.1, 20)
        fb = data["feedback"]
        if set(fb) != {"power_min_mw", "power_max_mw", "deadband_mw"}:
            raise ValueError("invalid feedback fields")
        lo = integer(fb["power_min_mw"], "power_min_mw", 1, 65535)
        integer(fb["power_max_mw"], "power_max_mw", lo, 65535)
        number(fb["deadband_mw"], "deadband_mw", 0)
        st = data["stall"]
        defaults = {
            "lock_deg": 20,
            "release_deg": 8,
            "confirm": 1,
            "track_win": 0.4,
            "track_ratio": 0.35,
            "track_min_deg": 1.5,
            "min_load_ratio": 0.3,
            "static_deg": 1,
            "hold_mw": 6000,
            "relock_s": 1,
        }
        if set(st) != set(defaults):
            raise ValueError("invalid stall fields")
        for key in st:
            number(st[key], key, 0)
        if st["lock_deg"] <= 0 or st["release_deg"] >= st["lock_deg"]:
            raise ValueError("release_deg must be below lock_deg")
        integer(st["confirm"], "confirm", 1, 100)
        integer(st["hold_mw"], "hold_mw", 1, 65535)
        if st["track_win"] < 0.05 or st["relock_s"] < 0.05:
            raise ValueError("track_win and relock_s must be >= 0.05s")
        if st["track_ratio"] > 1 or st["min_load_ratio"] > 1:
            raise ValueError("load and tracking ratios must be <= 1")
        joints = tuple(Joint.parse(row) for row in data["joints"])
        if len(joints) != 7 or {j.leader_id for j in joints} != set(range(7)):
            raise ValueError("configure exactly one mapping for each Leader ID 0..6")
        for attr in ("motor_id", "feedback_id"):
            if len({getattr(j, attr) for j in joints}) != 7:
                raise ValueError(f"duplicate {attr}")
        if {j.motor_id for j in joints} & {j.feedback_id for j in joints}:
            raise ValueError("command and feedback CAN IDs must not overlap")
        return cls(data, tuple(sorted(joints, key=lambda j: j.leader_id)))


class JointMap:
    def __init__(self, joints):
        self.joints = tuple(joints)
        self.leader_zero = {j.leader_id: j.leader_zero_deg for j in joints}
        self.follower_zero = {j.leader_id: j.follower_zero_rad for j in joints}

    def forward(self, leader_deg):
        targets, limited = {}, set()
        for j in self.joints:
            raw = self.follower_zero[j.leader_id] + j.sign * j.scale * math.radians(
                leader_deg[j.leader_id] - self.leader_zero[j.leader_id]
            )
            target = min(j.upper_rad, max(j.lower_rad, raw))
            if target != raw:
                limited.add(j.leader_id)
            targets[j.leader_id] = target
        return targets, limited

    def inverse(self, follower_rad):
        return {
            j.leader_id: self.leader_zero[j.leader_id]
            + math.degrees(
                (follower_rad[j.leader_id] - self.follower_zero[j.leader_id]) / (j.sign * j.scale)
            )
            for j in self.joints
        }

    def rebase(self, leader_deg, follower_rad):
        self.leader_zero = dict(leader_deg)
        self.follower_zero = dict(follower_rad)

    def slew(self, desired, previous, dt):
        # A delayed loop never authorizes a large catch-up step.
        dt = min(max(dt, 0), 0.05)
        return {
            j.leader_id: previous[j.leader_id]
            + min(
                j.speed_rad_s * dt,
                max(-j.speed_rad_s * dt, desired[j.leader_id] - previous[j.leader_id]),
            )
            for j in self.joints
        }


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


def encode_frame(can_id, payload):
    if not 0 <= can_id <= 0x7FF or len(payload) != 8:
        raise ValueError("expected standard CAN ID and eight data bytes")
    frame = bytearray(30)
    frame[:4] = b"\x55\xaa\x1e\x03"
    struct.pack_into("<II", frame, 4, 1, 10)
    struct.pack_into("<I", frame, 13, can_id)
    frame[18] = 8
    frame[21:29] = payload
    return bytes(frame)


class DMSerial:
    """Damiao 30-byte TX / 16-byte RX serial bridge; all I/O stays on one thread."""

    def __init__(self, uart, joints, clock=time.monotonic, sleep=time.sleep):
        self.uart, self.joints = uart, tuple(joints)
        self.clock, self.sleep = clock, sleep
        self.buffer = bytearray()
        self.states = {}
        self.replies = {}
        self.pending_registers = set()
        self.sequence = 0
        self.by_feedback = {j.feedback_id: j for j in joints}
        self.limits = {
            j.leader_id: (12.5, 10.0, 28.0) if j.model == "4340P" else (12.5, 30.0, 10.0)
            for j in joints
        }

    def send(self, can_id, data):
        frame = encode_frame(can_id, data)
        if self.uart.write(frame) != len(frame):
            raise LinkError("short write to Damiao bridge")

    def feed(self, chunk):
        self.buffer.extend(chunk)
        while len(self.buffer) >= 16:
            if self.buffer[0] != 0xAA or self.buffer[15] != 0x55:
                del self.buffer[0]
                continue
            raw = bytes(self.buffer[:16])
            del self.buffer[:16]
            if raw[1] != 0x11 or raw[2] != 8:
                continue
            can_id = struct.unpack_from("<I", raw, 3)[0]
            joint = self.by_feedback.get(can_id)
            if joint is None:
                continue
            data = raw[7:15]
            if (
                data[0] == joint.motor_id
                and data[1] <= 15
                and data[2] in (0x33, 0x55)
                and data[3] in (10, 21, 22, 23)
            ):
                key = (joint.leader_id, data[3])
                if data[2] == 0x33 and key in self.pending_registers:
                    self.replies[key] = data[4:8]
                continue
            if data[0] & 15 != joint.motor_id:
                continue
            pmax, vmax, tmax = self.limits[joint.leader_id]
            pos = int.from_bytes(data[1:3], "big") / 65535 * 2 * pmax - pmax
            vel = ((data[3] << 4) | (data[4] >> 4)) / 4095 * 2 * vmax - vmax
            torque = (((data[4] & 15) << 8) | data[5]) / 4095 * 2 * tmax - tmax
            self.sequence += 1
            self.states[joint.leader_id] = State(
                pos, vel, torque, data[0] >> 4, data[6], data[7], self.clock(), self.sequence
            )
        if len(self.buffer) > 4096:
            self.buffer.clear()

    def poll(self):
        remaining = min(self.uart.in_waiting, 65536)
        while remaining > 0:
            data = self.uart.read(min(remaining, 4096))
            if not data:
                break
            self.feed(data)
            remaining -= len(data)

    def read_register(self, joint, rid, timeout=0.2):
        self.poll()
        key = (joint.leader_id, rid)
        self.replies.pop(key, None)
        self.pending_registers.add(key)
        try:
            self.send(0x7FF, struct.pack("<HBBI", joint.motor_id, 0x33, rid, 0))
            deadline = self.clock() + timeout
            while self.clock() < deadline:
                self.poll()
                if key in self.replies:
                    return self.replies.pop(key)
                self.sleep(0.001)
            raise LinkError(f"J{joint.leader_id + 1}: register {rid} timeout")
        finally:
            self.pending_registers.discard(key)

    def verify(self):
        for j in self.joints:
            mode = struct.unpack("<I", self.read_register(j, 10))[0]
            if mode != 2:
                raise LinkError(f"J{j.leader_id + 1}: set POS_VEL mode with vendor tools first")
            limits = tuple(
                struct.unpack("<f", self.read_register(j, rid))[0] for rid in (21, 22, 23)
            )
            if any(not math.isfinite(v) or v <= 0 for v in limits):
                raise LinkError(f"J{j.leader_id + 1}: invalid feedback scaling")
            if j.lower_rad < -limits[0] or j.upper_rad > limits[0] or j.speed_rad_s > limits[1]:
                raise LinkError(f"J{j.leader_id + 1}: profile exceeds motor protocol range")
            self.limits[j.leader_id] = limits

    def read(self, timeout=0.05):
        self.poll()
        before = {i: st.sequence for i, st in self.states.items()}
        for j in self.joints:
            self.send(0x7FF, struct.pack("<HBBI", j.motor_id, 0xCC, 0, 0))
        deadline = self.clock() + timeout
        fresh = {}
        while self.clock() < deadline:
            self.poll()
            fresh = {
                j.leader_id: self.states[j.leader_id]
                for j in self.joints
                if j.leader_id in self.states
                and self.states[j.leader_id].sequence > before.get(j.leader_id, 0)
            }
            if len(fresh) == len(self.joints):
                break
            self.sleep(0.001)
        return fresh

    def command(self, targets):
        packets = [
            (
                j.motor_id + 0x100,
                struct.pack(
                    "<ff",
                    number(targets[j.leader_id], "target", j.lower_rad, j.upper_rad),
                    j.speed_rad_s,
                ),
            )
            for j in self.joints
        ]
        for can_id, payload in packets:
            self.send(can_id, payload)

    def enable(self):
        for j in self.joints:
            self.send(j.motor_id, b"\xff" * 7 + b"\xfc")

    def disable(self):
        errors = []
        for j in self.joints:
            try:
                self.send(j.motor_id, b"\xff" * 7 + b"\xfd")
            except Exception as exc:
                errors.append(str(exc))
        if errors:
            raise LinkError("; ".join(errors))
