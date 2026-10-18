"""B601 控制核心：自适应滤波、限速、负载去底、堵转与按钮仲裁。"""

from __future__ import annotations

import logging
import math

logger = logging.getLogger(__name__)


class Motion:
    def __init__(self, cfg, initial):
        self.cfg = cfg
        if len(initial) != 7 or not all(math.isfinite(v) for v in initial):
            raise ValueError("Expected seven finite initial positions")
        # DM 在零位边界有编码量化误差；冻结/HD 保持也必须使用限位内的目标。
        self.commanded = [
            min(math.radians(j.upper_deg), max(math.radians(j.lower_deg), p))
            for j, p in zip(cfg.joints, initial)
        ]
        self.filtered = None
        self.raw = None
        self.input_speed = [0.0] * 7
        self.velocity = [0.0] * 7

    def forward(self, angles):
        return [
            min(
                math.radians(j.upper_deg),
                max(
                    math.radians(j.lower_deg),
                    j.follower_home_rad + j.direction * math.radians(a - j.leader_home_deg),
                ),
            )
            for j, a in zip(self.cfg.joints, angles)
        ]

    def inverse(self, positions):
        return [
            j.leader_home_deg + math.degrees((p - j.follower_home_rad) / j.direction)
            for j, p in zip(self.cfg.joints, positions)
        ]

    def step(self, angles, dt, held=False):
        dt = min(max(dt, 1e-6), 0.05)
        if self.raw is None:
            self.raw, self.filtered = list(angles), list(angles)
        speed_alpha = dt / (dt + 1 / (2 * math.pi * 5))
        for i, value in enumerate(angles):
            self.input_speed[i] += speed_alpha * ((value - self.raw[i]) / dt - self.input_speed[i])
            cutoff = self.cfg.filter_cutoff_hz + self.cfg.filter_beta * abs(self.input_speed[i])
            alpha = dt / (dt + 1 / (2 * math.pi * cutoff))
            if abs(value - self.filtered[i]) > self.cfg.input_deadband_deg:
                self.filtered[i] += alpha * (value - self.filtered[i])
        self.raw = list(angles)
        held_ids = set(range(7)) if held is True else set(held or ())
        desired = self.forward(self.filtered)
        output = []
        for i, (j, old, goal) in enumerate(zip(self.cfg.joints, self.commanded, desired)):
            if i in held_ids:
                self.velocity[i] = 0.0
                output.append(old)
                continue
            # 按真实 dt 限速、限制加速度，并封顶单帧步长；长停顿不会授权追赶大步。
            vmax = math.radians(j.speed_deg_s)
            distance = goal - old
            # 目标反转时取消旧方向的虚拟速度，避免软件参考继续朝用户反方向移动。
            if self.velocity[i] * distance < 0:
                self.velocity[i] = 0.0
            braking_speed = math.sqrt(
                2 * math.radians(self.cfg.acceleration_deg_s2) * abs(distance)
            )
            wanted = math.copysign(min(vmax, braking_speed), distance) if distance else 0.0
            accel = math.radians(self.cfg.acceleration_deg_s2) * dt
            velocity = self.velocity[i] + max(-accel, min(accel, wanted - self.velocity[i]))
            step = max(
                -math.radians(self.cfg.max_step_deg),
                min(math.radians(self.cfg.max_step_deg), velocity * dt),
            )
            # 反转时先制动，不能因滤波/加速度状态把目标推过软限位。
            value = min(math.radians(j.upper_deg), max(math.radians(j.lower_deg), old + step))
            if (goal - old) * step >= 0 and abs(step) >= abs(goal - old):
                value = goal
            self.velocity[i] = (value - old) / dt
            output.append(value)
        self.commanded = output
        return list(output)


class Feedback:
    def __init__(self, cfg, leader):
        self.cfg, self.leader = cfg, leader
        self.baseline = [0.0] * 7
        self.torque = [0.0] * 7
        self.loads = [0.0] * 7
        self.triggers = set()
        self.windows, self.counts, self.release_since = {}, {}, {}
        self.sent = {}
        self.last_time = None
        self.closed = False
        self.state = "ready"

    def prime(self, baseline):
        # 启动时原地保持、无外部接触采集底载，避免直接把重力/摩擦当接触阻力。
        if len(baseline) != 7 or any(not math.isfinite(v) for v in baseline):
            raise ValueError("Invalid torque baseline")
        self.baseline = [abs(v) for v in baseline]
        self.torque = list(self.baseline)

    def interrupt(self):
        # 短暂丢包不立刻大力锁主臂；同时打断观察和释放计时，不跨缺失帧累计。
        self.windows.clear()
        self.counts.clear()
        self.release_since.clear()

    def observe(self, target, actual, torques, now, manual_hold=False):
        dt = (
            1 / self.cfg.loop_hz
            if self.last_time is None
            else min(max(now - self.last_time, 0), 0.1)
        )
        self.last_time = now
        cfg = self.cfg
        alpha = dt / (cfg.torque_filter_s + dt)
        for i, (j, goal, position, torque) in enumerate(zip(cfg.joints, target, actual, torques)):
            self.torque[i] += alpha * (abs(torque) - self.torque[i])
            error = abs(goal - position)
            # 只有跟随误差小时缓慢跟踪底载；堵住或保持时不把持续接触学习成空载。
            if error <= cfg.release_deg and not self.triggers and not manual_hold:
                self.baseline[i] += (
                    dt / (cfg.baseline_adapt_s + dt) * (self.torque[i] - self.baseline[i])
                )
            excess = max(0.0, self.torque[i] - self.baseline[i] - j.torque_deadband_nm)
            self.loads[i] = min(1.0, excess / (j.torque_reference_nm - j.torque_deadband_nm))
            if i in self.triggers:
                if error <= cfg.release_deg:
                    since = self.release_since.setdefault(i, now)
                    if now - since >= cfg.release_stable_s:
                        self.triggers.remove(i)
                        self.release_since.pop(i, None)
                else:
                    self.release_since.pop(i, None)
                continue
            if error < cfg.lock_deg or self.loads[i] < cfg.minimum_load_ratio:
                self.windows.pop(i, None)
                self.counts[i] = 0
                continue
            window = self.windows.setdefault(i, (now, goal, position, error))
            if now - window[0] < cfg.window_s:
                continue
            self.windows[i] = (now, goal, position, error)
            toward = math.copysign(1.0, window[1] - window[2]) * (position - window[2])
            # 关键：从臂仍朝目标移动或角差在收敛，就不按“跟随比例低”判堵转。
            # 高角差 + 超出底载的持续负载 + 实际无进展，连续多个窗口才保持。
            progressing = (
                toward > cfg.stalled_motion_deg or window[3] - error >= cfg.convergence_deg
            )
            if progressing:
                self.counts[i] = 0
                continue
            self.counts[i] = self.counts.get(i, 0) + 1
            if self.counts[i] >= cfg.confirm_windows:
                self.triggers.add(i)
                self.windows.pop(i, None)
                self.counts[i] = 0

    def apply(self, now, manual_hold=False, fault=False):
        state = (
            "fault hold: restart required"
            if fault
            else ("stall hold" if self.triggers else ("HD hold" if manual_hold else "feedback"))
        )
        if state != self.state:
            logger.info("Leader: %s", state)
            self.state = state
        for i in range(7):
            hold = fault or bool(self.triggers) or (manual_hold and i < 6)
            mode = 0x11 if hold else 0x12
            power = (
                self.cfg.hold_mw
                if hold
                else round(
                    self.cfg.power_min_mw
                    + self.loads[i] * (self.cfg.power_max_mw - self.cfg.power_min_mw)
                )
            )
            previous = self.sent.get(i)
            if (
                previous is None
                or previous[0] != mode
                or (hold and now - previous[2] >= self.cfg.relock_s)
                or (
                    not hold
                    and (
                        abs(power - previous[1]) >= self.cfg.power_deadband_mw
                        or power == self.cfg.power_min_mw != previous[1]
                    )
                )
            ):
                self.leader.mode(i, mode, power)
                self.sent[i] = (mode, power, now)

    def close(self):
        if not self.closed:
            self.closed = True
            self.leader.release()


class Session:
    def __init__(self, cfg, leader, follower, initial, now, baseline=None):
        self.cfg, self.follower = cfg, follower
        self.motion = Motion(cfg, [s.pos for s in initial])
        self.feedback = Feedback(cfg, leader)
        self.feedback.prime([s.torque for s in initial] if baseline is None else baseline)
        self.last_sequences = [s.sequence for s in initial]
        self.last_good = self.last_tick = now
        self.fault = False
        self.fault_reason = ""
        self.rejected = 0
        self.cycles = 0
        self.started = now

    def fail(self, reason):
        if not self.fault:
            logger.error("B601 paused: %s; restart after checking the devices", reason)
        self.fault = True
        self.fault_reason = reason

    def tick(self, angles, states, now, pressed=False):
        dt = max(0.0, now - self.last_tick)
        self.last_tick = now
        valid = (
            angles is not None
            and states is not None
            and len(angles) == len(states) == 7
            and all(math.isfinite(v) for v in angles)
            and all(
                all(
                    math.isfinite(v)
                    for v in (s.pos, s.vel, s.torque, s.mos_c, s.rotor_c, s.received_at)
                )
                and 0 <= now - s.received_at <= self.cfg.link_timeout_s
                and s.sequence > previous
                for s, previous in zip(states, self.last_sequences)
            )
        )
        if not valid:
            self.rejected += 1
            self.feedback.interrupt()
            if now - self.last_good >= self.cfg.link_timeout_s:
                self.fail("feedback timeout")
            # 本轮无完整反馈就不发送新目标；仅持续超时才请求 Leader 保持并锁存故障。
            self.feedback.apply(now, pressed, self.fault)
            return False
        self.last_good = now
        self.last_sequences = [s.sequence for s in states]
        for j, s in zip(self.cfg.joints, states):
            if (
                s.status != 1
                or max(s.mos_c, s.rotor_c) >= self.cfg.motor_temperature_c
                or not j.lower_deg - 0.1 <= math.degrees(s.pos) <= j.upper_deg + 0.1
            ):
                self.fail(
                    f"J{j.leader_id + 1}: status={s.status}, temperature={max(s.mos_c, s.rotor_c):.1f}C, position={math.degrees(s.pos):.1f}deg"
                )
        if self.fault:
            self.feedback.apply(now, pressed, True)
            return False
        target = self.motion.inverse(self.motion.commanded)
        actual = self.motion.inverse([s.pos for s in states])
        self.feedback.observe(target, actual, [s.torque for s in states], now, pressed)
        self.feedback.apply(now, pressed)
        held = True if self.feedback.triggers else (range(6) if pressed else False)
        command = self.motion.step(angles, dt, held=held)
        self.follower.command(command)
        self.cycles += 1
        return True
