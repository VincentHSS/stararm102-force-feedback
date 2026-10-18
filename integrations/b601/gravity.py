"""纯 Python 静态模型：本体重力与末端竖直负载的关节力矩。"""

from __future__ import annotations

import hashlib
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def add(a, b):
    return tuple(x + y for x, y in zip(a, b))


def sub(a, b):
    return tuple(x - y for x, y in zip(a, b))


def mv(matrix, vector):
    return tuple(dot(row, vector) for row in matrix)


def mm(a, b):
    return tuple(tuple(dot(row, column) for column in zip(*b)) for row in a)


IDENTITY = ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))


def rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return (
        (x * x * v + c, x * y * v - z * s, x * z * v + y * s),
        (y * x * v + z * s, y * y * v + c, y * z * v - x * s),
        (z * x * v - y * s, z * y * v + x * s, z * z * v + c),
    )


def vector(text, default=(0.0, 0.0, 0.0)):
    values = tuple(float(v) for v in text.split()) if text is not None else default
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError("Expected three finite model coordinates")
    return values


def origin(element):
    if element is None:
        return IDENTITY, (0.0, 0.0, 0.0)
    r, p, y = vector(element.get("rpy"))
    # URDF 的固定轴 RPY 为 Rz(yaw) Ry(pitch) Rx(roll)。
    matrix = mm(mm(rotation((0, 0, 1), y), rotation((0, 1, 0), p)), rotation((1, 0, 0), r))
    return matrix, vector(element.get("xyz"))


class GravityModel:
    def __init__(
        self, path, directions=None, offsets=None, gravity=(0, 0, -9.80665), tip="end_link"
    ):
        self.path = Path(path).resolve()
        data = self.path.read_bytes()
        self.sha256 = hashlib.sha256(data).hexdigest()
        self.directions = list(directions if directions is not None else [1.0] * 6)
        self.offsets = list(offsets if offsets is not None else [0.0] * 6)
        self.gravity = vector(" ".join(map(str, gravity)))
        if (
            len(self.directions) != 6
            or len(self.offsets) != 6
            or any(not math.isfinite(x) for x in self.directions + self.offsets)
            or any(abs(abs(x) - 1) > 1e-9 for x in self.directions)
        ):
            raise ValueError("Model mapping needs six signs and six finite offsets")
        if not 9 <= math.sqrt(dot(self.gravity, self.gravity)) <= 10.5:
            raise ValueError("Set the base-frame gravity vector in m/s^2")
        root = ET.fromstring(data)
        self.bodies, self.children = {}, {}
        for link in root.findall("link"):
            name = link.get("name")
            if not name or name in self.bodies:
                raise ValueError("Duplicate/missing URDF link")
            inertial = link.find("inertial")
            mass_element = link.find("inertial/mass")
            mass = 0.0 if mass_element is None else float(mass_element.get("value"))
            if not math.isfinite(mass) or mass < 0:
                raise ValueError("Invalid link mass")
            self.bodies[name] = (
                mass,
                origin(None if inertial is None else inertial.find("origin"))[1],
            )
        parents, indices, unsupported_branches = set(), set(), []
        for joint in root.findall("joint"):
            parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
            kind, name = joint.get("type"), joint.get("name")
            index = (
                [f"joint{i}" for i in range(1, 7)].index(name)
                if name in {f"joint{i}" for i in range(1, 7)}
                else None
            )
            if parent not in self.bodies or child not in self.bodies or child in parents:
                raise ValueError("Invalid URDF tree")
            if index is not None:
                if kind not in ("revolute", "continuous") or index in indices:
                    raise ValueError("Expected six distinct revolute arm joints")
                indices.add(index)
            elif kind != "fixed":
                # 官方指爪模型的移动分支无惯量；不将未知的有质量运动轴当作固定连杆。
                if kind != "prismatic" or self.bodies[child][0] != 0:
                    raise ValueError("Unsupported moving branch; use the B601-DM model")
                unsupported_branches.append(child)
            axis = vector(
                None if joint.find("axis") is None else joint.find("axis").get("xyz"), (1, 0, 0)
            )
            length = math.sqrt(dot(axis, axis))
            if index is not None and length < 1e-9:
                raise ValueError("Zero joint axis")
            axis = tuple(v / length for v in axis) if length else (0, 0, 0)
            self.children.setdefault(parent, []).append(
                (child, index, origin(joint.find("origin")), axis)
            )
            parents.add(child)
        roots = set(self.bodies) - parents
        if len(roots) != 1 or indices != set(range(6)) or tip not in self.bodies:
            raise ValueError("Expected one root, joint1..joint6 and an end_link")
        self.root, self.tip = roots.pop(), tip

        def branch_has_mass(link, seen):
            if link in seen:
                raise ValueError("Cyclic URDF tree")
            seen = seen | {link}
            return self.bodies[link][0] > 0 or any(
                branch_has_mass(row[0], seen) for row in self.children.get(link, [])
            )

        if any(branch_has_mass(link, set()) for link in unsupported_branches):
            raise ValueError("Unsupported moving branch contains mass")
        self.evaluate([0.0] * 6)

    def evaluate(self, positions):
        if len(positions) != 6 or not all(math.isfinite(v) for v in positions):
            raise ValueError("Expected six finite joint positions")
        q = [s * x + o for s, x, o in zip(self.directions, positions, self.offsets)]
        torques, axes, pivots, visited = [0.0] * 6, {}, {}, set()
        potential, tip_position, tip_ancestors = 0.0, None, None

        def walk(link, matrix, position, ancestors):
            nonlocal potential, tip_position, tip_ancestors
            if link in visited:
                raise ValueError("Cyclic URDF tree")
            visited.add(link)
            mass, com = self.bodies[link]
            center = add(position, mv(matrix, com))
            force = tuple(mass * g for g in self.gravity)
            potential -= dot(force, center)
            for i in ancestors:
                # 先保留方向：支撑重力所需力矩 = -J^T F_g，不对测量值预先取绝对值。
                torques[i] -= self.directions[i] * dot(
                    axes[i], cross(sub(center, pivots[i]), force)
                )
            if link == self.tip:
                tip_position, tip_ancestors = position, ancestors
            for child, i, (local_r, local_p), axis in self.children.get(link, []):
                frame_r, frame_p = mm(matrix, local_r), add(position, mv(matrix, local_p))
                branch = ancestors
                if i is not None:
                    axes[i], pivots[i] = mv(frame_r, axis), frame_p
                    frame_r = mm(frame_r, rotation(axis, q[i]))
                    branch = ancestors + (i,)
                walk(child, frame_r, frame_p, branch)

        walk(self.root, IDENTITY, (0, 0, 0), ())
        if visited != set(self.bodies) or set(tip_ancestors or ()) != set(range(6)):
            raise ValueError("Disconnected model or tip outside the six-joint arm")
        unit_mass, moment_x, moment_y = [], [], []
        magnitude = math.sqrt(dot(self.gravity, self.gravity))
        down = tuple(g / magnitude for g in self.gravity)
        seed = (1, 0, 0) if abs(down[0]) < 0.9 else (0, 1, 0)
        tangent = sub(seed, tuple(dot(seed, down) * g for g in down))
        length = math.sqrt(dot(tangent, tangent))
        tangent = tuple(v / length for v in tangent)
        other_tangent = cross(down, tangent)
        for i in range(6):
            axis = tuple(v * self.directions[i] for v in axes[i])
            unit_mass.append(-dot(axis, cross(sub(tip_position, pivots[i]), self.gravity)))
            moment_x.append(-dot(axis, tangent))
            moment_y.append(-dot(axis, other_tangent))
        return {
            "gravity": torques,
            "unit_mass": unit_mass,
            "moment_x": moment_x,
            "moment_y": moment_y,
            "potential": potential,
            "tip": tip_position,
        }
