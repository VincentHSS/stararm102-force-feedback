import argparse
import collections
import math
import os
import shutil
import signal
import struct
import sys
import threading
import time

import serial
import fashionstar_uart_sdk as uservo

try:
    import termios
    import tty
except ImportError:
    termios = None
    tty = None

SERVO_BAUDRATE = 1000000
LEADER_PORT_NAME = "/dev/ttyUSB0"
FOLLOWER_PORT_NAME_Arr = ["/dev/ttyUSB1"]

METHOD_RELEASE = 0x10
METHOD_LOCK = 0x11
METHOD_DAMPING = 0x12

POWER_MAX = 65535

FB_CURRENT_MAX = [500, 1500, 3500, 3000, 1000, 1000, 1000]

FB_CURRENT_FLOOR = 30

FB_POWER_MIN = 1
FB_POWER_MAX = 1200

STALL_LOCK_DEG = 20.0
STALL_RELEASE_DEG = 8.0
STALL_RELEASE_HOLD_S = 0.3

STALL_LOCK_METHOD = "both"
STALL_LOCK_SCOPE = "all"
STALL_HOLD_MW = 6000

STALL_RELOCK_S = 1.0

STALL_TRACK_WIN_S = 0.4
STALL_TRACK_RATIO = 0.35
STALL_TRACK_MIN_DEG = 1.5
STALL_CONFIRM = 1

STALL_REQUIRE_MAX_LOAD = True
STALL_MIN_LOAD_RATIO = 0.3
STALL_REQUIRE_STATIC = True
STALL_STATIC_DEG = 1.0

HOLD_INTERVAL_MS = 100
HOLD_T_ACC_MS = 50
HOLD_T_DEC_MS = 50

DAMPING_BANNER = "StarArm 102 控制台 | h 帮助 · s 状态 · q 退出"

HELP_FEEDBACK = "反馈：fb on|off|status\n参数：fbmax <mA|7个值|auto> · fbp <min> <max>\n调节：fbq <mA> · fbdb <mW> · fbhz <Hz>\n阻尼：<mW> / 7个值 / J=功率；0=协议最大功率，n=卸力"

HELP_STALL = "锁定：l（J1–J6） · st lock（全部） · st unlock\n保护：st on|off|reset|status\n阈值：stlock <deg> · strel <deg> · stn <次数>\n判定：stwin <s> · sttrack <0~1> · sttrackmin <deg>\n负载：stload <0~1|off> · ststatic <deg|off>\n锁力：stscope all|stalled · sthold <mW>\n方式：stmethod both|stop|hold · strelock <s>\n自动解锁：误差持续 0.3s 低于解除阈值"


class LoadFeedback:
    def __init__(
        self,
        joint_ids,
        enabled=False,
        hz=30.0,
        power_min=FB_POWER_MIN,
        power_max=FB_POWER_MAX,
        current_max=FB_CURRENT_MAX,
        current_floor=FB_CURRENT_FLOOR,
        deadband=20.0,
        auto_tau=10.0,
    ):
        self.joint_ids = list(joint_ids)
        self.enabled = bool(enabled)
        self.hz = max(1.0, float(hz))
        self.current_floor = max(1.0, float(current_floor))

        self.power_min = max(1, min(int(power_min), POWER_MAX))
        self.power_max = max(self.power_min, min(int(power_max), POWER_MAX))
        self.current_max = _norm_current_max(current_max, self.joint_ids)
        self.deadband = max(0.0, float(deadband))
        self.auto_tau = max(0.5, float(auto_tau))

        self.currents = {i: 0.0 for i in self.joint_ids}
        self.peaks = {i: self.current_floor for i in self.joint_ids}
        self.sent = {i: None for i in self.joint_ids}
        self.entered = set()
        self.read_ms = 0.0
        self.failures = 0
        self.next_at = 0.0
        self.last_at = None

    def reference(self, servo_id):
        if self.current_max is not None:
            return max(self.current_floor + 1.0, float(self.current_max[servo_id]))
        return max(self.current_floor + 1.0, self.peaks[servo_id])

    def set_current_max(self, current_max):
        self.current_max = _norm_current_max(current_max, self.joint_ids)
        self.sent = {i: None for i in self.joint_ids}

    def set_power_range(self, power_min, power_max):
        self.power_min = max(1, min(int(power_min), POWER_MAX))
        self.power_max = max(self.power_min, min(int(power_max), POWER_MAX))
        self.sent = {i: None for i in self.joint_ids}

    def ratio_for(self, servo_id, current):
        span = self.reference(servo_id) - self.current_floor
        ratio = (float(current) - self.current_floor) / span if span > 0 else 0.0
        return 0.0 if ratio < 0 else (1.0 if ratio > 1 else ratio)

    def power_for(self, servo_id, current):
        ratio = self.ratio_for(servo_id, current)
        return int(round(self.power_min + ratio * (self.power_max - self.power_min)))

    def observe(self, currents, now):
        dt = (now - self.last_at) if self.last_at is not None else 0.0
        self.last_at = now
        decay = math.exp(-dt / self.auto_tau) if dt > 0 else 1.0
        for servo_id in self.joint_ids:
            value = currents.get(servo_id)
            if value is None:
                continue
            value = float(value)
            self.currents[servo_id] = value
            if self.current_max is None:
                self.peaks[servo_id] = max(value, self.peaks[servo_id] * decay, self.current_floor)
        self.next_at = now + 1.0 / self.hz

    def pending(self):
        updates = {}
        for servo_id in self.joint_ids:
            target = self.power_for(servo_id, self.currents[servo_id])
            last = self.sent[servo_id]
            if last is None or abs(target - last) >= self.deadband:
                updates[servo_id] = target
        return updates

    def commit(self, updates):
        self.sent.update(updates)

    def reset_state(self):
        self.sent = {i: None for i in self.joint_ids}
        self.entered.clear()
        self.next_at = 0.0
        self.last_at = None

    def describe(self):
        if not self.enabled:
            return "负载反馈 OFF"
        src = "固定参考" if self.current_max is not None else "自动峰值"
        return (
            f"负载反馈 ON {self.hz:g}Hz {src} 读{self.read_ms:.1f}ms "
            f"｜ 电流mA {' '.join(f'{self.currents[i]:.0f}' for i in self.joint_ids)}"
            f" ｜ 阻尼mW {' '.join(str(self.power_for(i, self.currents[i])) for i in self.joint_ids)}"
        )


class StallGuard:
    def __init__(
        self,
        joint_ids,
        enabled=True,
        lock_deg=STALL_LOCK_DEG,
        release_deg=STALL_RELEASE_DEG,
        confirm=STALL_CONFIRM,
        lock_method=STALL_LOCK_METHOD,
        lock_scope=STALL_LOCK_SCOPE,
        hold_mw=STALL_HOLD_MW,
        relock_s=STALL_RELOCK_S,
        track_win=STALL_TRACK_WIN_S,
        track_ratio=STALL_TRACK_RATIO,
        track_min_deg=STALL_TRACK_MIN_DEG,
        require_max_load=STALL_REQUIRE_MAX_LOAD,
        min_load_ratio=STALL_MIN_LOAD_RATIO,
        require_static=STALL_REQUIRE_STATIC,
        static_deg=STALL_STATIC_DEG,
    ):
        self.joint_ids = list(joint_ids)
        self.enabled = bool(enabled)
        self.lock_deg = max(0.1, float(lock_deg))
        self.release_deg = max(0.0, min(float(release_deg), self.lock_deg))
        self.confirm = max(1, int(confirm))
        self.lock_method = lock_method if lock_method in ("both", "stop", "hold") else "both"
        self.lock_scope = lock_scope if lock_scope in ("all", "stalled") else "all"
        self.hold_mw = max(0, min(int(hold_mw), POWER_MAX))
        self.relock_s = max(0.0, float(relock_s))

        self.track_win = max(0.05, float(track_win))
        self.track_ratio = max(0.0, min(float(track_ratio), 1.0))
        self.track_min_deg = max(0.0, float(track_min_deg))
        self.require_max_load = bool(require_max_load)
        self.min_load_ratio = max(0.0, min(float(min_load_ratio), 1.0))
        self.require_static = bool(require_static)
        self.static_deg = max(0.0, float(static_deg))
        self.diffs = {i: 0.0 for i in self.joint_ids}
        self.load_ratios = {i: 0.0 for i in self.joint_ids}
        self.reasons = {i: "" for i in self.joint_ids}
        self.triggers = set()
        self.locked = set()
        self.auto_locked = False
        self.lock_count = 0
        self.follow_targets = {}
        self.lock_angles = {}
        self.locked_at = {}
        self._over = {i: 0 for i in self.joint_ids}
        self._release_since = {}
        self.manual_source = None
        self._windows = {}

    def lock_targets(self):
        if self.lock_scope == "all":
            return list(self.joint_ids)
        return sorted(self.triggers)

    def _verdict(self, win, ld, fl, diff):
        if win.get("ld") is None or win.get("fl") is None or ld is None or fl is None:
            return None, "缺少角度读数"
        ld_moved = abs(float(ld) - float(win["ld"]))
        ld_delta = float(ld) - float(win["ld"])
        fl_delta = float(fl) - float(win["fl"])
        fl_moved = fl_delta if ld_delta >= 0 else -fl_delta
        err_delta = float(diff) - float(win["err"])
        if ld_moved > 0 and ld_moved >= self.track_min_deg:
            ratio = fl_moved / ld_moved
            if ratio >= self.track_ratio:
                return True, f"从臂在跟随（{ratio * 100:.0f}%，主臂动 {ld_moved:.1f}°）"
            return False, f"从臂没跟上（{ratio * 100:.0f}%，主臂动 {ld_moved:.1f}°）"

        if self.require_static and err_delta <= -self.static_deg:
            return True, f"误差在缩小（{err_delta:+.1f}°，主臂几乎没动）"
        if self.require_static:
            return False, f"主臂几乎不动、误差也没缩小（{err_delta:+.1f}°）"
        return False, f"主臂几乎不动（ststatic off，误差 {err_delta:+.1f}°）"

    def update(self, diffs, ld_targets=None, fl_angles=None, loads=None, now=None):
        to_lock, to_release = [], []
        if not self.enabled or (self.locked and not self.auto_locked):
            return to_lock, to_release
        now = time.monotonic() if now is None else float(now)

        for servo_id in self.joint_ids:
            diff = diffs.get(servo_id)
            if diff is None or not math.isfinite(float(diff)):
                self._windows.pop(servo_id, None)
                self._release_since.pop(servo_id, None)
                self._over[servo_id] = 0
                continue
            diff = float(diff)
            self.diffs[servo_id] = diff
            if loads is not None and loads.get(servo_id) is not None:
                self.load_ratios[servo_id] = float(loads[servo_id])
            ld = ld_targets.get(servo_id) if ld_targets else None
            fl = fl_angles.get(servo_id) if fl_angles else None

            if servo_id in self.triggers:
                if diff <= self.release_deg:
                    since = self._release_since.setdefault(servo_id, now)
                    if now - since >= STALL_RELEASE_HOLD_S:
                        self.triggers.discard(servo_id)
                        self._release_since.pop(servo_id, None)
                        self._over[servo_id] = 0
                        self._windows.pop(servo_id, None)
                        self.reasons[servo_id] = ""
                else:
                    self._release_since.pop(servo_id, None)
                continue

            if diff < self.lock_deg:
                self._over[servo_id] = 0
                self._windows.pop(servo_id, None)
                self.reasons[servo_id] = ""
                continue

            win = self._windows.get(servo_id)
            if win is None:
                self._windows[servo_id] = {"t": now, "ld": ld, "fl": fl, "err": diff}
                if not self.reasons[servo_id]:
                    self.reasons[servo_id] = f"观察中 0.0/{self.track_win:g}s"
                continue
            age = now - win["t"]
            if age < self.track_win:
                if not self.reasons[servo_id]:
                    self.reasons[servo_id] = f"观察中 {age:.1f}/{self.track_win:g}s"
                continue

            following, reason = self._verdict(win, ld, fl, diff)
            self._windows[servo_id] = {"t": now, "ld": ld, "fl": fl, "err": diff}

            if following is None:
                self.reasons[servo_id] = reason
                continue
            if following:
                self._over[servo_id] = 0
                self.reasons[servo_id] = reason
                continue

            ratio = self.load_ratios.get(servo_id, 0.0)
            if self.require_max_load and (loads is None or servo_id not in loads):
                self._over[servo_id] = 0
                self.reasons[servo_id] = "电流数据缺失"
                continue
            if self.require_max_load and loads is not None and ratio < self.min_load_ratio:
                self._over[servo_id] = 0
                self.reasons[servo_id] = (
                    f"负载 {ratio * 100:.0f}% 未到 {self.min_load_ratio * 100:.0f}%"
                )
                continue

            self._over[servo_id] += 1
            if self._over[servo_id] >= self.confirm:
                self.triggers.add(servo_id)
                self._over[servo_id] = 0
                self._windows.pop(servo_id, None)
                self.reasons[servo_id] = "已判定堵转"
            else:
                self.reasons[servo_id] = (
                    f"没在跟随，确认 {self._over[servo_id]}/" f"{self.confirm}（{reason}）"
                )

        if self.triggers:
            to_lock = sorted(set(self.lock_targets()) - self.locked)
            self.locked.update(to_lock)
            self.auto_locked = True
            if to_lock:
                self.lock_count += 1
        elif self.auto_locked and self.locked and not self.triggers:
            to_release = sorted(self.locked)
            self.locked.clear()
            self.auto_locked = False
            for servo_id in to_release:
                self.follow_targets.pop(servo_id, None)
                self.lock_angles.pop(servo_id, None)
                self.locked_at.pop(servo_id, None)
        return to_lock, to_release

    def remember_lock(self, servo_id, angle, now):
        if angle is not None:
            self.lock_angles[servo_id] = float(angle)
        self.locked_at[servo_id] = now

    def due_for_relock(self, now):
        if self.relock_s <= 0:
            return []
        return [i for i in sorted(self.locked) if now - self.locked_at.get(i, 0.0) >= self.relock_s]

    def set_thresholds(self, lock_deg=None, release_deg=None, confirm=None):
        if lock_deg is not None:
            self.lock_deg = max(0.1, float(lock_deg))
            self.release_deg = min(self.release_deg, self.lock_deg)
        if release_deg is not None:
            self.release_deg = max(0.0, min(float(release_deg), self.lock_deg))
        if confirm is not None:
            self.confirm = max(1, int(confirm))

    def reset(self):
        stale = sorted(self.locked)
        self.locked.clear()
        self.triggers.clear()
        self.auto_locked = False
        self.diffs = {i: 0.0 for i in self.joint_ids}
        self._over = {i: 0 for i in self.joint_ids}
        self._windows.clear()
        self._release_since.clear()
        self.manual_source = None
        self.reasons = {i: "" for i in self.joint_ids}
        self.follow_targets.clear()
        self.lock_angles.clear()
        self.locked_at.clear()
        return stale

    def describe(self):
        state = "开启" if self.enabled else "关闭"
        gates = [
            f"观察窗口 {self.track_win:g}s",
            f"跟随率≥{self.track_ratio * 100:.0f}% 视为在跟随",
        ]
        if self.require_max_load:
            gates.append(f"需负载≥{self.min_load_ratio * 100:.0f}%")
        if self.require_static:
            gates.append(f"误差缩小≥{self.static_deg:g}° 视为在收敛")
        head = (
            f"[阻尼] 堵转保护：{state} ｜ 锁定阈值 {self.lock_deg:.0f}° ｜ 解除阈值 "
            f"{self.release_deg:.0f}° ｜ 连续确认 {self.confirm} 个窗口 ｜ 累计锁定 {self.lock_count} 次"
        )
        head2 = (
            f"[阻尼] 判据：{' ｜ '.join(gates)} ｜ 锁定范围 {self.lock_scope} ｜ "
            f"方式 {self.lock_method} ｜ 保持功率 {self.hold_mw}mW ｜ 重发 {self.relock_s:g}s"
        )
        rows = []
        for i in self.joint_ids:
            mark = "(锁)" if i in self.locked else ("(堵转)" if i in self.triggers else "")
            rows.append(f"J{i + 1}:{self.diffs[i]:.1f}°/{self.load_ratios[i] * 100:.0f}%{mark}")
        return head, head2, "  ".join(rows)

    def describe_short(self):
        if self.locked:
            if len(self.locked) == len(self.joint_ids):
                scope = f"全部 {len(self.locked)} 关节"
            else:
                scope = "、".join(f"J{i + 1}" for i in sorted(self.locked))
            if self.triggers:
                detail = "、".join(f"J{i + 1} {self.diffs[i]:.0f}°" for i in sorted(self.triggers))
            else:
                detail = "手动锁定"
            return f"⚠锁定 {scope}（{detail}）"
        if not self.enabled:
            return "堵转保护 OFF"
        worst = max(self.joint_ids, key=lambda i: self.diffs[i], default=None)
        if worst is None:
            return "堵转保护 ON"
        return f"堵转 OK 最大误差 {self.diffs[worst]:.1f}°@J{worst + 1}"


def _norm_current_max(current_max, joint_ids):
    if current_max is None:
        return None
    if isinstance(current_max, (int, float)):
        return {i: float(current_max) for i in joint_ids}
    values = list(current_max)
    if len(values) == 1:
        return {i: float(values[0]) for i in joint_ids}
    return {i: float(values[n]) for n, i in enumerate(joint_ids) if n < len(values)}


def _parse_current_max_arg(text, parser, joint_ids):
    text = (text or "").strip()
    if text.lower() in ("", "auto", "none", "自动"):
        return None
    values = []
    for token in text.replace("，", ",").split(","):
        token = token.strip()
        if not token:
            continue
        value = _parse_int(token)
        if value is None or value <= 0:
            parser.error(f"--fb_current_max 取值无效：{token}（应为 >0 的整数 mA，或 auto）")
        values.append(value)
    if len(values) not in (1, len(joint_ids)):
        parser.error(
            f"--fb_current_max 需要 1 个或 {len(joint_ids)} 个值，"
            f"例如 --fb_current_max 3000 或 --fb_current_max 800,1200,1500,2000,1500,1200,800"
        )
    return values


def _read_follower_monitors(follower_controls, joint_ids, targets=None):
    found, missing = {}, set()
    for manager in follower_controls:
        monitors = _read_monitors(manager, joint_ids)
        for servo_id in joint_ids:
            info = monitors.get(servo_id)
            angle = getattr(info, "angle_monitor", None)
            current = getattr(info, "current", None)
            if angle is None:
                missing.add(servo_id)
                continue
            entry = found.setdefault(servo_id, {})
            if (
                "angle" not in entry
                or targets is not None
                and abs(angle - targets[servo_id]) > abs(entry["angle"] - targets[servo_id])
            ):
                entry["angle"] = angle
            if current is not None:
                entry["current"] = max(abs(current), entry.get("current", 0.0))
    return found, sorted(missing | (set(joint_ids) - found.keys()))


def _read_monitors(manager, joint_ids):
    # SDK 超时会保留旧值，查询前清空监控字段。
    for servo_id in joint_ids:
        info = manager.servos.get(servo_id)
        if info is not None:
            info.angle_monitor = None
            info.current = None
    monitors = manager.send_sync_servo_monitor(list(joint_ids)) or {}
    for info in monitors.values():
        if info is None:
            continue
        for field in ("angle_monitor", "current"):
            value = getattr(info, field, None)
            if value is not None and not math.isfinite(float(value)):
                setattr(info, field, None)
    return monitors


class Button:
    key_state_0 = 0
    key_state_1 = 180
    key_state_3 = 90
    state = {"robot_unlock_state": key_state_0, "lock": key_state_1, "unlock": key_state_3}

    def __init__(self, id=7):
        self.id = id
        self.flag = False


servo_ids = [0, 1, 2, 3, 4, 5, 6]


class DampingConsole(threading.Thread):
    MAX_QUEUE = 32
    POLL_INTERVAL = 0.2
    PROMPT = "阻尼> "

    def __init__(
        self, joint_ids, initial_power=None, refresh_interval=0.0, feedback=None, stall=None
    ):
        super().__init__(daemon=True, name="damping-console")
        self.joint_ids = list(joint_ids)
        self.feedback = feedback if feedback is not None else LoadFeedback(self.joint_ids)
        self.stall = stall if stall is not None else StallGuard(self.joint_ids)
        self._lock = threading.Lock()
        self._queue = collections.deque()
        self._inputs = collections.deque()
        self._quit = False
        self._refresh_interval = max(0.0, float(refresh_interval))
        self._last_refresh = 0.0
        self.enabled = False
        self.powers = {}

        self._out = sys.stdout
        self._io_lock = threading.RLock()
        self._live_terminal = False
        self._saved_termios = None
        self._tty_fd = None
        self._buffer = ""
        self._pending_bytes = b""
        self._esc_state = 0
        self._submitted = []
        self._freq_text = "--"
        self._status = ""
        self._shown_status = None
        self._dirty = True
        self._ready = threading.Event()

        if initial_power is not None:
            self.submit_damping({i: int(initial_power) for i in self.joint_ids})

    def apply_pending(self, leader_control):
        with self._lock:
            if self._queue:
                cmd = self._queue.popleft()
            elif self._inputs:
                cmd = ("input", self._inputs.popleft(), None)
            else:
                cmd = None
        if cmd is None:
            return
        kind, ids, powers = cmd
        if kind == "input":
            try:
                self.handle_line(ids)
            except (ValueError, TypeError, OverflowError) as exc:
                self.emit(f"[错误] 命令无效：{exc}")
            return
        if kind == "damping":
            self.stall.reset()
            for i, power in sorted(powers.items()):
                leader_control.stop_on_control_mode(i, METHOD_DAMPING, power)

                leader_control.set_damping(i, power)
            self.enabled = True
            self.powers.update(powers)
            self._last_refresh = time.monotonic()
            self.emit(f"[阻尼] 主臂已进入阻尼：{_powers_text(powers)}")
            if any(p == 0 for p in powers.values()):
                self.emit("[提示] 功率 0=协议最大功率；n=卸力")
        elif kind == "release":
            for i in self.joint_ids:
                if i not in self.stall.locked:
                    leader_control.stop_on_control_mode(i, METHOD_RELEASE, 0x00)
            self.enabled = False
            self.powers.clear()
            self.emit("[锁定] 保持" if self.stall.locked else "[卸力] 主臂")
        elif kind == "lock":
            for servo_id in self.stall.reset():
                if servo_id not in self.joint_ids[:-1]:
                    self._restore_joint_after_stall(leader_control, servo_id)
            self.stall.locked.update(self.joint_ids[:-1])
            self.stall.manual_source = "console"
            self._lock_joints(leader_control, self.joint_ids[:-1], time.monotonic())
            self.enabled = False
            self.powers.clear()
            self.emit("[锁定] J1–J6")
        elif kind == "unlock":
            for servo_id in ids:
                self._restore_joint_after_stall(leader_control, servo_id)
        elif kind == "stallock":
            self.stall.locked.update(ids)
            self.stall.auto_locked = False
            self.stall.manual_source = "console"
            self._lock_joints(leader_control, ids, time.monotonic())
            self.emit(f"[锁定] {_ids_text(ids)} | {self.stall.hold_mw}mW")
        self._refresh_status()

    def refresh(self, leader_control):
        now = time.monotonic()
        # 锁力维护独立于从臂通信状态。
        relock = self.stall.due_for_relock(now)
        if relock:
            self._lock_joints(leader_control, relock, now)
        if not self.enabled or self._refresh_interval <= 0:
            return
        if now - self._last_refresh < self._refresh_interval:
            return
        self._last_refresh = now
        for i, power in self.powers.items():
            if i in self.stall.locked:
                continue
            leader_control.set_damping(i, power)

    def drain(self, leader_control):
        for _ in range(self.MAX_QUEUE * 4):
            with self._lock:
                if not self._queue and not self._inputs:
                    return
            self.apply_pending(leader_control)

    def follower_tick(self, leader_control, follower_controls, target_angles):
        fb, guard = self.feedback, self.stall
        if not (fb.enabled or guard.enabled):
            return
        now = time.monotonic()
        if now < fb.next_at:
            return

        t0 = time.perf_counter()
        try:
            monitors, _missing = _read_follower_monitors(
                follower_controls, self.joint_ids, target_angles
            )
        except Exception as exc:
            fb.read_ms = (time.perf_counter() - t0) * 1000.0
            self._feedback_backoff(now, f"读取 Follower 数据异常：{exc}")
            return
        fb.read_ms = (time.perf_counter() - t0) * 1000.0
        if not monitors:
            self._feedback_backoff(now, "未读到 Follower 数据（检查从臂串口/供电）")
            return
        if _missing:
            for servo_id in _missing:
                monitors.pop(servo_id, None)
            self._feedback_backoff(now, "从臂监控数据不完整")
        else:
            fb.failures = 0

        diffs = {}
        fl_angles = {}
        ld_targets = {}
        for servo_id in self.joint_ids:
            entry = monitors.get(servo_id) or {}
            angle = entry.get("angle")
            if angle is not None:
                fl_angles[servo_id] = float(angle)
            target = target_angles[servo_id] if servo_id < len(target_angles) else None
            if target is not None:
                ld_targets[servo_id] = float(target)
            if angle is None or target is None:
                continue
            diffs[servo_id] = abs(float(angle) - float(target))
        currents = {i: e["current"] for i, e in monitors.items() if e.get("current") is not None}
        loads = {i: fb.ratio_for(i, c) for i, c in currents.items()}
        to_lock, to_release = guard.update(
            diffs, ld_targets=ld_targets, fl_angles=fl_angles, loads=loads, now=now
        )

        fb.observe(currents, now)
        if _missing:
            fb.next_at = max(fb.next_at, now + 0.2)
        updates = fb.pending() if fb.enabled else {}
        updates = {i: p for i, p in updates.items() if i not in guard.locked and i in currents}

        if to_lock:
            for servo_id in to_lock:
                guard.follow_targets[servo_id] = target_angles[servo_id]
            self._lock_joints(leader_control, to_lock, now)
            for servo_id in to_lock:
                fb.sent[servo_id] = None
                fb.entered.discard(servo_id)
            detail = "、".join(f"J{i + 1} 误差 {diffs[i]:.0f}°" for i in sorted(guard.triggers))
            scope = (
                ("主臂全部 %d 个关节" % len(to_lock))
                if guard.lock_scope == "all"
                else "、".join(f"J{i + 1}" for i in to_lock)
            )
            self.emit(f"[锁定] {scope} | {detail}")

        for servo_id, power in sorted(updates.items()):
            if servo_id not in fb.entered:
                leader_control.stop_on_control_mode(servo_id, METHOD_DAMPING, power)
                fb.entered.add(servo_id)
            leader_control.set_damping(servo_id, power)
        if updates:
            fb.commit(updates)
            self.enabled = True
            self.powers.update(updates)

        if to_release:
            for servo_id in to_release:
                self._restore_joint_after_stall(leader_control, servo_id)
            if set(to_release) == set(self.joint_ids):
                detail = "主臂全部关节"
            else:
                detail = "、".join(f"J{i + 1}" for i in to_release)
            self.emit(f"[解锁] {detail}")

        self._refresh_status()

    def _lock_joints(self, leader_control, ids, now):
        guard = self.stall
        ids = [i for i in ids if i in self.joint_ids]
        if not ids:
            return
        angles = {}
        for servo_id in ids:
            angle = guard.lock_angles.get(servo_id)
            if angle is None:
                info = (
                    leader_control.servos.get(servo_id)
                    if hasattr(leader_control, "servos")
                    else None
                )
                angle = getattr(info, "angle_monitor", None) if info is not None else None
            if angle is not None:
                angles[servo_id] = float(angle)

        if guard.lock_method != "hold":
            for servo_id in ids:
                leader_control.stop_on_control_mode(servo_id, METHOD_LOCK, guard.hold_mw)

        if guard.lock_method != "stop" and angles:
            command_data_list = [
                struct.pack(
                    "<BlLHHH",
                    servo_id,
                    int(angles[servo_id] * 10),
                    HOLD_INTERVAL_MS,
                    HOLD_T_ACC_MS,
                    HOLD_T_DEC_MS,
                    guard.hold_mw,
                )
                for servo_id in sorted(angles)
            ]
            leader_control.send_sync_multiturnanglebyinterval(
                14, len(command_data_list), command_data_list
            )

        for servo_id in ids:
            if guard.lock_method == "hold" and servo_id not in angles:
                leader_control.stop_on_control_mode(servo_id, METHOD_LOCK, guard.hold_mw)
                self.emit(f"[锁定] J{servo_id + 1} 角度缺失，使用停止保持")
            guard.remember_lock(servo_id, angles.get(servo_id), now)
            self.feedback.entered.discard(servo_id)
            self.feedback.sent[servo_id] = None

    def _restore_joint_after_stall(self, leader_control, servo_id):
        if self.feedback.enabled:
            power = self.feedback.power_for(servo_id, self.feedback.currents[servo_id])
            leader_control.stop_on_control_mode(servo_id, METHOD_DAMPING, power)
            leader_control.set_damping(servo_id, power)
            self.feedback.sent[servo_id] = power
            self.feedback.entered.add(servo_id)
            self.powers[servo_id] = power
            self.enabled = True
        elif self.enabled and servo_id in self.powers:
            power = self.powers[servo_id]
            leader_control.stop_on_control_mode(servo_id, METHOD_DAMPING, power)
            leader_control.set_damping(servo_id, power)
        else:
            leader_control.stop_on_control_mode(servo_id, METHOD_RELEASE, 0x00)

    def _feedback_backoff(self, now, reason):
        fb = self.feedback
        self.stall._windows.clear()
        self.stall._release_since.clear()
        self.stall._over = dict.fromkeys(self.joint_ids, 0)
        fb.failures += 1
        fb.next_at = now + max(1.0 / fb.hz, 0.2)
        if fb.failures in (1, 10) or fb.failures % 100 == 0:
            self.emit(f"[阻尼] {reason}（第 {fb.failures} 次），已自动退避重试")
        self._refresh_status()

    def set_frequency(self, fps_text):
        self._freq_text = fps_text
        self._refresh_status(emit_plain=True)

    def _refresh_status(self, emit_plain=False):
        text = f"[状态] 运行频率 {self._freq_text} ｜ {self._state_text()}"
        text = _clip_line(text, self._term_width() - 1)
        with self._io_lock:
            self._status = text
        if emit_plain and not self._live_terminal and self._out.isatty():
            print(text)

    def _state_text(self):
        parts = [self.stall.describe_short()]
        if self.feedback.enabled:
            powers = " ".join(
                str(self.feedback.power_for(i, self.feedback.currents[i])) for i in self.joint_ids
            )
            parts.append(f"阻尼mW {powers}")
        elif self.enabled:
            powers = " ".join(str(self.powers.get(i, "-")) for i in self.joint_ids)
            parts.append(f"阻尼mW(手动) {powers}")
        else:
            parts.append("主臂无阻尼")
        return " ｜ ".join(parts)

    def emit(self, text):
        if not self._live_terminal:
            print(text)
            return
        with self._io_lock:
            if not self._live_terminal:
                print(text)
                return

            self._out.write("\x1b[1A\r\x1b[K")
            self._out.write(text + "\n")
            self._out.write("\r\x1b[K" + self._status + "\n")
            self._out.write("\r\x1b[K" + self.PROMPT + self._display_buffer())
            self._out.flush()
            self._shown_status = self._status

    def _term_width(self):
        try:
            return shutil.get_terminal_size().columns
        except Exception:
            return 80

    def _display_buffer(self):
        limit = max(10, self._term_width() - len(self.PROMPT) - 1)
        if len(self._buffer) <= limit:
            return self._buffer
        return "…" + self._buffer[-(limit - 1) :]

    def _init_lines(self):
        self._out.write("\n")
        self._out.write(self.PROMPT)
        self._out.flush()

    def _redraw_locked(self):
        self._out.write("\x1b[1A\r\x1b[K" + self._status)
        self._out.write("\x1b[1B\r\x1b[K" + self.PROMPT + self._display_buffer())
        self._out.flush()
        self._shown_status = self._status
        self._dirty = False

    def _restore_terminal(self):
        with self._io_lock:
            if self._saved_termios is not None and self._tty_fd is not None and termios is not None:
                try:
                    termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, self._saved_termios)
                except Exception:
                    pass
                self._saved_termios = None
            if self._live_terminal:
                try:
                    self._out.write("\r\x1b[K" + self.PROMPT + self._display_buffer() + "\n")
                    self._out.flush()
                except Exception:
                    pass
                self._live_terminal = False

    def close(self):
        self._quit = True
        self._restore_terminal()

    @property
    def stop_requested(self):
        return self._quit

    def submit_damping(self, powers):
        with self._lock:
            self._push(("damping", None, dict(powers)))

    def submit_release(self):
        with self._lock:
            self._push(("release", None, None))

    def submit_lock(self):
        with self._lock:
            self._push(("lock", None, None))

    def submit_unlock(self, ids):
        if not ids:
            return
        with self._lock:
            self._push(("unlock", list(ids), None))

    def release_stalled(self, reason=""):
        stale = self.stall.reset()
        if stale:
            self.submit_unlock(stale)
            if reason:
                self.emit(
                    f"[阻尼] 已解除 J{'、J'.join(str(i + 1) for i in stale)} 的堵转锁定（{reason}）"
                )

    def _push(self, cmd):
        if len(self._queue) >= self.MAX_QUEUE:
            self.emit("[错误] 命令队列已满，请重试")
            return
        self._queue.append(cmd)

    def run(self):
        if termios is not None and sys.stdin.isatty() and sys.stdout.isatty():
            try:
                self._run_pty()
                return
            except Exception as exc:
                self._restore_terminal()
                print(f"[阻尼] 终端交互界面不可用，回退到简单输入模式：{exc}")
        self._ready.set()
        self._run_plain()

    def _run_pty(self):
        import select

        fd = sys.stdin.fileno()
        self._tty_fd = fd
        self._saved_termios = termios.tcgetattr(fd)
        tty.setcbreak(fd)
        self._install_signal_guards()
        self._live_terminal = True
        with self._io_lock:
            self._init_lines()
            self._refresh_status()
        self._ready.set()

        try:
            while not self._quit and self._live_terminal:
                try:
                    ready, _, _ = select.select([fd], [], [], self.POLL_INTERVAL)
                except (OSError, ValueError):
                    break

                lines = []
                with self._io_lock:
                    if ready:
                        try:
                            data = os.read(fd, 1024)
                        except OSError:
                            break
                        if not data:
                            return
                        self._handle_input_bytes(data)
                    if self._dirty or self._status != self._shown_status:
                        self._redraw_locked()
                    if self._submitted:
                        lines, self._submitted = self._submitted, []
                        self._redraw_locked()
                for line in lines:
                    self._dispatch(line)
        finally:
            self._restore_terminal()

    def _run_plain(self):
        buf = b""
        while not self._quit:
            try:
                import select

                fd = sys.stdin.fileno()
                ready, _, _ = select.select([fd], [], [], self.POLL_INTERVAL)
                if not ready:
                    continue
                chunk = os.read(fd, 4096)
            except (ImportError, AttributeError, OSError, ValueError):
                try:
                    line = input()
                except (EOFError, KeyboardInterrupt):
                    return
                except Exception:
                    return
                if self._quit:
                    return
                self._dispatch(line)
                continue
            if not chunk:
                return
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                if self._quit:
                    return
                self._dispatch(line.decode("utf-8", "replace"))

    def _install_signal_guards(self):
        def _on_signal(_signum, _frame):
            self._restore_terminal()
            os._exit(0)

        try:
            signal.signal(signal.SIGTERM, _on_signal)
        except (ValueError, OSError):
            pass

    def _handle_input_bytes(self, data):
        for byte in data:
            if self._esc_state:
                if self._esc_state == 1 and byte in (0x5B, 0x4F):
                    self._esc_state = 2
                elif 0x40 <= byte <= 0x7E:
                    self._esc_state = 0
                continue

            if byte == 0x1B:
                self._esc_state = 1
                continue
            if byte in (0x0D, 0x0A):
                line, self._buffer = self._buffer, ""
                self._pending_bytes = b""
                self._dirty = True
                if line.strip():
                    self._submitted.append(line)
                continue
            if byte in (0x7F, 0x08):
                self._buffer = self._buffer[:-1]
                self._dirty = True
                continue
            if byte == 0x15:
                self._buffer = ""
                self._pending_bytes = b""
                self._dirty = True
                continue
            if byte == 0x04:
                self._quit = True
                continue

            self._pending_bytes += bytes([byte])
            try:
                text = self._pending_bytes.decode("utf-8")
            except UnicodeDecodeError as exc:
                if "unexpected end of data" in str(exc.reason):
                    continue
                text = self._pending_bytes.decode("utf-8", "replace")
            self._pending_bytes = b""
            self._buffer += text
            self._dirty = True

    def _dispatch(self, line):
        with self._lock:
            if len(self._inputs) >= self.MAX_QUEUE:
                self.emit("[错误] 命令队列已满，请重试")
                return
            self._inputs.append(line)

    def handle_line(self, line):
        raw = (line or "").strip()
        if not raw:
            return
        parts = raw.replace(",", " ").replace("，", " ").split()
        key = parts[0].lower()

        if key in ("h", "help", "?", "？"):
            self.emit(DAMPING_BANNER.strip("\n"))
            self.emit(HELP_FEEDBACK.strip("\n"))
            self.emit(HELP_STALL.strip("\n"))
            return
        if key in ("q", "quit", "exit"):
            self._quit = True
            self.emit("[阻尼] 正在退出")
            return
        if key in ("n", "normal", "release", "r"):
            self._disable_feedback_by_manual("按了 n")
            if self.stall.enabled:
                self.stall.enabled = False
                self.emit("[阻尼] 堵转保护 OFF")

            self.stall.reset()
            self.submit_release()
            return
        if key in ("l", "lock"):
            self._disable_feedback_by_manual("按了 l")
            self.submit_lock()
            return
        if key in ("s", "status"):
            self.print_status()
            return
        if key in ("st", "sts", "stall", "堵转"):
            self._handle_stall_command(parts[1:])
            return
        if key == "stlock":
            self._handle_stall_number(parts[1:], "堵转锁定角度阈值", "lock_deg", "°", 0.1)
            return
        if key == "strel":
            self._handle_stall_number(parts[1:], "堵转解除角度阈值", "release_deg", "°", 0.0)
            return
        if key == "stn":
            self._handle_stall_number(parts[1:], "堵转连续确认次数", "confirm", "次", 1)
            return
        if key in ("sthold", "stholdmw"):
            self._handle_stall_number(parts[1:], "堵转保持功率", "hold_mw", "mW", 0.0)
            return
        if key == "stmethod":
            self._handle_stall_method(parts[1:])
            return
        if key in ("stscope", "stallscope"):
            self._handle_stall_scope(parts[1:])
            return
        if key in ("stload", "stloadmax"):
            self._handle_stall_load(parts[1:])
            return
        if key in ("ststatic", "stmove"):
            self._handle_stall_static(parts[1:])
            return
        if key in ("stwin", "stwindow"):
            self._handle_stall_number(parts[1:], "观察窗口", "track_win", "秒", 0.05)
            return
        if key == "sttrack":
            self._handle_stall_number(parts[1:], "跟随率门槛", "track_ratio", "（0~1）", 0.0, 1.0)
            return
        if key == "sttrackmin":
            self._handle_stall_number(parts[1:], "主臂最小位移", "track_min_deg", "°", 0.0)
            return
        if key == "strelock":
            self._handle_stall_number(parts[1:], "堵转锁定重发间隔", "relock_s", "s", 0.0)
            return
        if key in ("fb", "fbs", "fbstatus", "反馈", "负载"):
            self._handle_feedback_command(parts[1:])
            return
        if key in ("fbmax", "fb_max"):
            self._handle_fbmax(parts[1:])
            return
        if key in ("fbp", "fbpower"):
            self._handle_fbp(parts[1:])
            return
        if key in ("fbq", "fbfloor"):
            self._handle_fb_number(parts[1:], "电流噪声门槛", "current_floor", "mA", 1.0)
            return
        if key in ("fbdb", "fbdeadband"):
            self._handle_fb_number(parts[1:], "阻尼死区", "deadband", "mW", 0.0)
            return
        if key in ("fbhz",):
            self._handle_fb_number(parts[1:], "电流读取频率", "hz", "Hz", 1.0)
            return

        if key in ("d", "damp", "damping", "阻尼"):
            rest = parts[1:]
        else:
            rest = parts

        powers = self._parse_powers(rest)
        if powers is None:
            return
        self._disable_feedback_by_manual("手动输入了阻尼功率")

        stale = self.stall.reset()
        extra = [i for i in stale if i not in powers]
        if extra:
            self.submit_unlock(extra)
        self.submit_damping(powers)

    def _parse_powers(self, tokens):
        if not tokens:
            self.emit(
                "[阻尼] 用法：500 ｜ 500 600 700 800 900 1000 1100 ｜ 3=800 4=900 ｜ 3 800，输入 h 查看帮助"
            )
            return None

        if any("=" in t or ":" in t for t in tokens):
            powers = {}
            for token in tokens:
                pair = token.replace(":", "=").split("=")
                if len(pair) != 2:
                    self.emit(f"[阻尼] 无法识别的输入：{token}（应为 id=功率，例如 3=800）")
                    return None
                servo_id = _parse_int(pair[0])
                power = _parse_int(pair[1])
                if servo_id is None or servo_id not in self.joint_ids:
                    self.emit(
                        f"[阻尼] 关节 ID 无效：{pair[0]}（主臂可用：{_ids_text(self.joint_ids)}）"
                    )
                    return None
                if power is None or not 0 <= power <= POWER_MAX:
                    self.emit(f"[阻尼] 功率无效：{pair[1]}（请输入 0~{POWER_MAX} 的整数，单位 mW）")
                    return None
                powers[servo_id] = power
            return powers

        nums = []
        for token in tokens:
            value = _parse_int(token)
            if value is None:
                self.emit(
                    f"[阻尼] 无法识别的输入：{token}（请输入数字，如 500 或 500 600 700 ...）"
                )
                return None
            nums.append(value)

        if len(nums) == 1:
            plan = {i: nums[0] for i in self.joint_ids}

        elif len(nums) == 2 and nums[0] in self.joint_ids:
            plan = {nums[0]: nums[1]}

        elif (
            len(nums) % 2 == 0
            and len(nums) >= 4
            and all(nums[i] in self.joint_ids for i in range(0, len(nums), 2))
        ):
            plan = {nums[i]: nums[i + 1] for i in range(0, len(nums), 2)}

        elif 2 <= len(nums) <= len(self.joint_ids):
            plan = {i: nums[i] for i in range(len(nums))}
        else:
            self.emit(
                "[阻尼] 数值个数不对：可以给 1 个（全部同功率）、2~7 个（按 J1~J7 顺序），"
                "或写成 3=800 4=900 精确指定；输入 h 查看帮助"
            )
            return None

        for servo_id, power in plan.items():
            if not 0 <= power <= POWER_MAX:
                self.emit(
                    f"[阻尼] 关节 J{servo_id + 1} 的功率超出范围：{power}"
                    f"（有效范围 0~{POWER_MAX} mW）"
                )
                return None
        return plan

    def _disable_feedback_by_manual(self, reason):
        if not self.feedback.enabled:
            return False
        self.feedback.enabled = False
        self.feedback.reset_state()
        self.emit("[反馈] OFF")
        self._refresh_status()
        return True

    def set_feedback(self, enabled):
        fb = self.feedback
        if enabled:
            fb.reset_state()
            fb.enabled = True
            self.emit("[反馈] ON")
        else:
            fb.enabled = False
            fb.reset_state()
            self.enabled = False
            self.powers.clear()
            with self._lock:
                self._push(("release", None, None))
            self.emit("[阻尼] 负载反馈 OFF")
        self._refresh_status()

    def _handle_feedback_command(self, rest):
        action = rest[0].lower() if rest else ""
        if action in ("on", "1", "enable", "开", "开启"):
            self.set_feedback(True)
        elif action in ("off", "0", "disable", "关", "关闭"):
            self.set_feedback(False)
        elif action in ("", "s", "status", "状态"):
            self.print_feedback_status()
        else:
            self.emit(HELP_FEEDBACK.strip("\n"))

    def _handle_fbmax(self, rest):
        if not rest:
            self.emit("[阻尼] 用法：fbmax 3000 ｜ fbmax 800,1200,...（7 个值） ｜ fbmax auto")
            return
        if rest[0].lower() in ("auto", "自动"):
            self.feedback.set_current_max(None)
            self.emit("[阻尼] 参考电流改为“自动跟踪实测峰值”")
            self._refresh_status()
            return
        values = []
        for token in rest:
            token = token.strip()
            if not token:
                continue
            value = _parse_int(token)
            if value is None or value <= 0:
                self.emit(f"[阻尼] 参考电流无效：{token}（应为 >0 的整数，单位 mA）")
                return
            values.append(value)
        if len(values) not in (1, len(self.joint_ids)):
            self.emit(
                f"[阻尼] 参考电流需要 1 个或 {len(self.joint_ids)} 个值，"
                f"例如 fbmax 3000 或 fbmax 800,1200,1500,2000,1500,1200,800"
            )
            return
        self.feedback.set_current_max(values)
        self.emit(f"[阻尼] 参考电流已设为 {'、'.join(str(v) for v in values)} mA")
        self._refresh_status()

    def _handle_fbp(self, rest):
        if len(rest) != 2:
            self.emit("[阻尼] 用法：fbp <下限mW> <上限mW>，例如 fbp 100 1500")
            return
        low, high = _parse_int(rest[0]), _parse_int(rest[1])
        if low is None or high is None or low < 0 or high < low:
            self.emit("[阻尼] 功率范围无效（下限 ≥0，且不能大于上限）")
            return
        if low == 0:
            self.emit("[提示] 反馈下限按 1mW 处理")
        self.feedback.set_power_range(low, high)
        self.emit(
            f"[阻尼] 阻尼功率范围已设为 {self.feedback.power_min}~{self.feedback.power_max} mW"
        )
        self._refresh_status()

    def _handle_fb_number(self, rest, label, attr, unit, minimum):
        if len(rest) != 1:
            self.emit(f"[阻尼] 用法：{label} <{unit}>")
            return
        value = _parse_float(rest[0])
        if value is None or value < minimum:
            self.emit(f"[阻尼] {label}无效：{rest[0]}（应为 ≥ {minimum:g} 的数值）")
            return
        setattr(self.feedback, attr, float(value))
        self.emit(f"[阻尼] {label}已设为 {value:g} {unit}")
        self._refresh_status()

    def print_feedback_status(self):
        fb = self.feedback
        if fb.current_max is None:
            ref_desc = "自动跟踪实测峰值"
        else:
            ref_desc = "固定参考 " + "、".join(
                f"J{i + 1}={fb.current_max[i]:g}mA" for i in self.joint_ids
            )
        self.emit(
            f"[阻尼] 负载反馈：{'开启' if fb.enabled else '关闭'}"
            f" ｜ 读取频率 {fb.hz:g}Hz ｜ 阻尼范围 {fb.power_min}~{fb.power_max}mW"
            f" ｜ 电流门槛 {fb.current_floor:g}mA ｜ 死区 {fb.deadband:g}mW"
        )
        self.emit(f"[阻尼] 参考电流：{ref_desc}")
        self.emit("[阻尼]   关节     电流mA    峰值mA    映射阻尼mW")
        for servo_id in self.joint_ids:
            self.emit(
                f"[阻尼]   J{servo_id + 1}(id{servo_id})"
                f"{fb.currents[servo_id]:9.0f}{fb.peaks[servo_id]:10.0f}"
                f"{fb.power_for(servo_id, fb.currents[servo_id]):12d}"
            )
        if fb.read_ms:
            self.emit(f"[阻尼] 上次读取耗时 {fb.read_ms:.1f}ms，失败 {fb.failures} 次")

    def _handle_stall_command(self, rest):
        action = rest[0].lower() if rest else ""
        if action in ("on", "1", "enable", "开", "开启"):
            self.stall.enabled = True
            self.emit("[阻尼] 堵转保护 ON")
        elif action in ("off", "0", "disable", "关", "关闭"):
            self.stall.enabled = False
            self.release_stalled("关闭堵转保护")
            self.emit("[阻尼] 堵转保护 OFF")
        elif action in ("reset", "清除"):
            self.release_stalled("清除锁定")
            self.emit("[阻尼] 保护状态已重置")
        elif action in ("", "s", "status", "状态"):
            self.print_stall_status()
        elif action in ("lock", "锁定"):
            targets = list(self.joint_ids)
            with self._lock:
                self._push(("stallock", targets, None))
        elif action in ("unlock", "解锁"):
            self.release_stalled("手动解锁")
        else:
            self.emit(HELP_STALL.strip("\n"))
        self._refresh_status()

    def _handle_stall_scope(self, rest):
        if len(rest) != 1 or rest[0].lower() not in ("all", "stalled"):
            self.emit(
                "[阻尼] 用法：stscope all|stalled  "
                "（all=锁住主臂全部关节；stalled=只锁堵转的那个关节）"
            )
            return
        self.stall.lock_scope = rest[0].lower()
        self.emit(f"[阻尼] 锁定范围已设为 {self.stall.lock_scope}")
        self._refresh_status()

    def _handle_stall_number(self, rest, label, attr, unit, minimum, maximum=None):
        if len(rest) != 1:
            self.emit(f"[阻尼] 用法：{label} <{unit}>")
            return
        value = _parse_float(rest[0])
        if value is None or value < minimum or (maximum is not None and value > maximum):
            extra = (
                f"，应 {minimum:g}~{maximum:g}" if maximum is not None else f"（应 ≥ {minimum:g}）"
            )
            self.emit(f"[阻尼] {label}无效：{rest[0]}{extra}")
            return
        if attr == "confirm":
            self.stall.set_thresholds(confirm=int(value))
        elif attr == "hold_mw":
            self.stall.hold_mw = max(0, min(int(value), POWER_MAX))
        elif attr == "relock_s":
            self.stall.relock_s = max(0.0, float(value))
        elif attr == "track_win":
            self.stall.track_win = max(0.05, float(value))
            self.stall._windows.clear()
        elif attr == "track_ratio":
            self.stall.track_ratio = max(0.0, min(float(value), 1.0))
        elif attr == "track_min_deg":
            self.stall.track_min_deg = max(0.0, float(value))
        else:
            self.stall.set_thresholds(**{attr: value})
        self.emit(f"[阻尼] {label}已设为 {value:g} {unit}")
        self._refresh_status()

    def _handle_stall_method(self, rest):
        if len(rest) != 1 or rest[0].lower() not in ("both", "stop", "hold"):
            self.emit(
                "[阻尼] 用法：stmethod both|stop|hold  "
                "（both=停止指令+保持角度；stop=只发 0x11；hold=只发保持角度）"
            )
            return
        self.stall.lock_method = rest[0].lower()
        self.emit(f"[阻尼] 锁定方式已设为 {self.stall.lock_method}")
        self._refresh_status()

    def print_stall_status(self):
        head, config, rows = self.stall.describe()
        self.emit(head)
        self.emit(config)
        self.emit(rows)
        reasons = [f"J{i + 1} {r}" for i, r in self.stall.reasons.items() if r]
        if reasons:
            self.emit("[判定] " + " | ".join(reasons))

    def _handle_stall_load(self, rest):
        if len(rest) != 1:
            self.emit(
                "[阻尼] 用法：stload <0~1>|off  "
                "触发锁定要求的负载比例（1=阻尼已给到最大；off=关掉这个条件）"
            )
            return
        token = rest[0].lower()
        if token in ("off", "no", "关", "关闭"):
            self.stall.require_max_load = False
            self.emit("[保护] 负载门槛 OFF")
        else:
            value = _parse_float(token)
            if value is None or not 0 <= value <= 1:
                self.emit(f"[阻尼] 无效负载比例：{rest[0]}（应 0~1 或 off）")
                return
            self.stall.require_max_load = True
            self.stall.min_load_ratio = value
            self.emit(f"[阻尼] 触发锁定要求负载达到参考电流的 {value * 100:.0f}%")
        self._refresh_status()

    def _handle_stall_static(self, rest):
        if len(rest) != 1:
            self.emit("[阻尼] 用法：ststatic <deg>|off  " "误差收敛阈值（off=关闭）")
            return
        token = rest[0].lower()
        if token in ("off", "no", "关", "关闭"):
            self.stall.require_static = False
            self.emit("[阻尼] 收敛判断 OFF")
        else:
            value = _parse_float(token)
            if value is None or value < 0:
                self.emit(f"[阻尼] 无效角度：{rest[0]}")
                return
            self.stall.require_static = True
            self.stall.static_deg = value
            self.emit(f"[阻尼] 收敛阈值 {value:g}°")
        self._refresh_status()

    def print_status(self):
        with self._lock:
            pending = len(self._queue) + len(self._inputs)
        if self.feedback.enabled:
            self.print_feedback_status()
        elif self.enabled:
            self.emit(f"[阻尼] {_powers_text(self.powers, full=True)}")
        else:
            self.emit("[阻尼] OFF")
        self.print_stall_status()
        if pending:
            self.emit(f"[阻尼] 另有 {pending} 条命令等待主循环执行")


def _parse_gate_arg(text, label, parser):
    token = str(text).strip().lower()
    if token in ("off", "no", "none", "关", "关闭"):
        return None
    try:
        value = float(token)
        if not math.isfinite(value):
            raise ValueError(token)
        return value
    except ValueError:
        parser.error(f"{label} 取值无效：{text}（应为数字，或 off 关闭该条件）")


def _parse_int(text):
    try:
        value = float(str(text).strip())
        return int(value) if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def _parse_float(text):
    try:
        value = float(str(text).strip())
        return value if math.isfinite(value) else None
    except (TypeError, ValueError):
        return None


def _ids_text(ids):
    return ",".join(str(i) for i in ids)


def _clip_line(text, width):
    if width <= 1 or len(text) <= width:
        return text
    return text[: width - 1] + "…"


def _powers_text(powers, full=False):
    if not powers:
        return "无"
    values = set(powers.values())
    if not full and len(values) == 1 and len(powers) == len(servo_ids):
        return f"主臂全部 {len(powers)} 个舵机都是 {powers[min(powers)]}mW"
    return "、".join(f"J{i + 1}(id{i})={p}mW" for i, p in sorted(powers.items()))


def measure_frequency():
    count = 0
    start_time = time.monotonic()

    def get_frequency():
        nonlocal count, start_time
        count += 1
        current_time = time.monotonic()
        elapsed = current_time - start_time

        if elapsed >= 1.0:
            frequency = count / elapsed
            count = 0
            start_time = current_time
            return frequency
        return None

    return get_frequency


def main(args):
    button_enable = args.leader_type == "102HD"
    button_id = 7
    filtered_size = 1

    if args.button is not None:
        button_enable = args.button
    if args.button_enable:
        button_enable = True
    if args.button_disable:
        button_enable = False

    if args.button_id is not None:
        button_id = args.button_id
    if args.filtered_size is not None:
        filtered_size = args.filtered_size

    if filtered_size < 1:
        raise ValueError("filtered_size 必须大于 0")

    leader_uart = serial.Serial(
        port=LEADER_PORT_NAME,
        baudrate=SERVO_BAUDRATE,
        parity=serial.PARITY_NONE,
        stopbits=1,
        bytesize=8,
        timeout=0,
    )
    leader_control = uservo.UartServoManager(leader_uart)
    leader_control.stop_on_control_mode(0xFF, 0x10, 0x00)
    leader_control.reset_multi_turn_angle(0xFF)

    if button_enable:
        button_get = leader_control.ping(button_id)
        if button_get == False:
            raise ValueError(f"找不到button_id={button_id}")

    follower_uart_arr = [
        serial.Serial(
            port=NAME,
            baudrate=SERVO_BAUDRATE,
            parity=serial.PARITY_NONE,
            stopbits=1,
            bytesize=8,
            timeout=0,
        )
        for NAME in FOLLOWER_PORT_NAME_Arr
    ]
    follower_control_arr = [uservo.UartServoManager(uart) for uart in follower_uart_arr]
    for control in follower_control_arr:
        control.stop_on_control_mode(0xFF, METHOD_RELEASE, 0)
        control.reset_multi_turn_angle(0xFF)

    button = Button(id=button_id)

    get_frequency = measure_frequency()

    target_angle = [0.0 for i in range(len(servo_ids))]

    target_angle_buffer = collections.deque(maxlen=filtered_size)

    feedback = LoadFeedback(
        servo_ids,
        enabled=(not args.no_feedback) and args.damping is None,
        hz=args.fb_hz,
        power_min=args.fb_power_min,
        power_max=args.fb_power_max,
        current_max=args.fb_current_max,
        current_floor=args.fb_current_floor,
        deadband=args.fb_deadband,
        auto_tau=args.fb_auto_tau,
    )

    load_gate = _parse_gate_arg(args.stall_min_load_ratio, "--stall_min_load_ratio", parser)
    static_gate = _parse_gate_arg(args.stall_static_deg, "--stall_static_deg", parser)
    stall = StallGuard(
        servo_ids,
        enabled=not args.no_stall,
        lock_deg=args.stall_lock_deg,
        release_deg=args.stall_release_deg,
        confirm=args.stall_confirm,
        track_win=args.stall_track_win,
        track_ratio=args.stall_track_ratio,
        track_min_deg=args.stall_track_min_deg,
        lock_method=args.stall_lock_method,
        lock_scope=args.stall_lock_scope,
        hold_mw=args.stall_hold_mw,
        relock_s=args.stall_relock_s,
        require_max_load=load_gate is not None,
        min_load_ratio=STALL_MIN_LOAD_RATIO if load_gate is None else load_gate,
        require_static=static_gate is not None,
        static_deg=STALL_STATIC_DEG if static_gate is None else static_gate,
    )

    damping_console = DampingConsole(
        servo_ids,
        initial_power=args.damping,
        refresh_interval=args.damping_refresh,
        feedback=feedback,
        stall=stall,
    )
    if not args.no_console:
        damping_console.start()
        damping_console._ready.wait(timeout=1.0)
        damping_console.emit(DAMPING_BANNER.strip("\n"))

    next_leader_warning = 0.0
    try:
        while True:
            damping_console.apply_pending(leader_control)
            damping_console.refresh(leader_control)
            if damping_console.stop_requested:
                damping_console.drain(leader_control)
                break

            leader_servo_board_ids = servo_ids.copy()
            if button_enable:
                leader_servo_board_ids.append(button.id)
            leader_monitors = _read_monitors(leader_control, leader_servo_board_ids)
            if any(
                getattr(leader_monitors.get(i), "angle_monitor", None) is None
                for i in leader_servo_board_ids
            ):
                now = time.monotonic()
                if now >= next_leader_warning:
                    damping_console.emit("[通信] 主臂监控缺失，暂停跟随更新")
                    next_leader_warning = now + 5.0
                time.sleep(0.01)
                continue
            for id in servo_ids:
                target_angle[id] = leader_control.servos[id].angle_monitor
            target_angle[-1] = max(0, min(target_angle[-1] * 1.5, 90))

            target_angle_buffer.append(target_angle.copy())
            filtered_angle = [sum(col) / len(col) for col in zip(*target_angle_buffer)]

            filtered_angle[-1] = target_angle[-1]

            if button_enable:
                pressed = leader_control.servos[button.id].angle_monitor == button.state["lock"]
                if pressed and not button.flag:
                    if not damping_console.stall.locked:
                        guard = damping_console.stall
                        guard.locked.update(servo_ids[:-1])
                        guard.manual_source = "button"
                        damping_console._lock_joints(
                            leader_control, servo_ids[:-1], time.monotonic()
                        )
                    button.flag = True
                elif not pressed and button.flag:
                    if damping_console.stall.manual_source == "button":
                        for servo_id in damping_console.stall.reset():
                            damping_console._restore_joint_after_stall(leader_control, servo_id)
                    button.flag = False

            for servo_id, angle in damping_console.stall.follow_targets.items():
                if servo_id in damping_console.stall.locked:
                    filtered_angle[servo_id] = angle

            command_data_list = [
                struct.pack("<BlLHHH", i, int(filtered_angle[i] * 10), 100, 50, 50, 0)
                for i in servo_ids
            ]

            for control in follower_control_arr:
                control.send_sync_multiturnanglebyinterval(
                    14, len(command_data_list), command_data_list
                )

            damping_console.follower_tick(leader_control, follower_control_arr, filtered_angle)
            time.sleep(0.001)

            freq = get_frequency()
            if freq is not None:
                damping_console.set_frequency(f"{freq:.2f} Hz")
    except KeyboardInterrupt:
        damping_console.close()
        print("\n[阻尼] 正在退出")
    finally:
        damping_console.close()
        try:
            leader_control.stop_on_control_mode(0xFF, METHOD_RELEASE, 0x00)
            print("[阻尼] 主臂已卸力")
        except Exception as exc:
            print(f"[错误] 主臂卸力失败: {exc}")
        finally:
            for uart in [leader_uart, *follower_uart_arr]:
                uart.close()


if __name__ == "__main__":
    if os.name == "nt":
        for stream in (sys.stdout, sys.stderr):
            if hasattr(stream, "reconfigure"):
                stream.reconfigure(encoding="utf-8", errors="replace")

    parser = argparse.ArgumentParser(description="StarArm 102 主从控制")

    parser.add_argument(
        "--leader_type",
        choices=["102HD", "102LD"],
        default="102LD",
        help="设备型号：102HD / 102LD，默认102LD",
    )
    parser.add_argument("--button_enable", action="store_true", help="启用按键")
    parser.add_argument("--button_disable", action="store_true", help="禁用按键")
    parser.add_argument(
        "--button",
        type=lambda x: (str(x).lower() == "true"),
        default=None,
        help="直接指定按键状态，True开启 / False关闭",
    )
    parser.add_argument("--button_id", type=int, default=None, help="按键ID")
    parser.add_argument("--filtered_size", type=int, default=None, help="均值滤波窗口大小")
    parser.add_argument("--damping", type=int, default=None, help="初始阻尼功率 mW，0=协议最大功率")
    parser.add_argument("--damping_refresh", type=float, default=0.0, help="阻尼重发间隔 s，0=禁用")
    parser.add_argument("--no_console", action="store_true", help="禁用终端实时阻尼命令输入")
    parser.add_argument("--no_feedback", action="store_true", help="关闭电流反馈")
    parser.add_argument("--fb_hz", type=float, default=30.0, help="从臂监控频率 Hz")
    parser.add_argument(
        "--fb_power_min", type=int, default=FB_POWER_MIN, help="最小反馈功率 mW，最低按 1 处理"
    )
    parser.add_argument("--fb_power_max", type=int, default=FB_POWER_MAX, help="最大反馈功率 mW")
    parser.add_argument(
        "--fb_current_max",
        type=str,
        default=",".join(str(v) for v in FB_CURRENT_MAX),
        help="参考电流 mA：单值、7个逗号分隔值或 auto",
    )
    parser.add_argument(
        "--fb_current_floor", type=float, default=FB_CURRENT_FLOOR, help="电流噪声门槛 mA"
    )
    parser.add_argument("--fb_deadband", type=float, default=20.0, help="功率更新死区 mW")
    parser.add_argument("--fb_auto_tau", type=float, default=10.0, help="自动峰值衰减时间 s")
    parser.add_argument("--no_stall", action="store_true", help="关闭自动堵转保护")
    parser.add_argument(
        "--stall_lock_deg", type=float, default=STALL_LOCK_DEG, help="堵转误差阈值 °"
    )
    parser.add_argument(
        "--stall_release_deg",
        type=float,
        default=STALL_RELEASE_DEG,
        help="自动解除误差阈值 °，持续 0.3s",
    )
    parser.add_argument(
        "--stall_confirm", type=int, default=STALL_CONFIRM, help="连续堵转确认窗口数"
    )
    parser.add_argument(
        "--stall_track_win", type=float, default=STALL_TRACK_WIN_S, help="堵转观察窗口 s"
    )
    parser.add_argument(
        "--stall_track_ratio", type=float, default=STALL_TRACK_RATIO, help="同向跟随比例 0~1"
    )
    parser.add_argument(
        "--stall_track_min_deg", type=float, default=STALL_TRACK_MIN_DEG, help="主臂有效位移门槛 °"
    )
    parser.add_argument(
        "--stall_lock_method",
        choices=["both", "stop", "hold"],
        default=STALL_LOCK_METHOD,
        help="锁力方式：停止保持、位置保持或两者",
    )
    parser.add_argument(
        "--stall_lock_scope",
        choices=["all", "stalled"],
        default=STALL_LOCK_SCOPE,
        help="锁定全部关节或堵转关节",
    )
    parser.add_argument("--stall_hold_mw", type=int, default=STALL_HOLD_MW, help="锁定保持功率 mW")
    parser.add_argument(
        "--stall_relock_s", type=float, default=STALL_RELOCK_S, help="锁力重发间隔 s，0=禁用"
    )
    parser.add_argument(
        "--stall_min_load_ratio",
        type=str,
        default=str(STALL_MIN_LOAD_RATIO),
        help="负载门槛 0~1，off=禁用",
    )
    parser.add_argument(
        "--stall_static_deg",
        type=str,
        default=str(STALL_STATIC_DEG),
        help="误差收敛门槛 °，off=禁用",
    )

    args = parser.parse_args()
    if args.damping is not None and not 0 <= args.damping <= POWER_MAX:
        parser.error(f"--damping 取值范围 0~{POWER_MAX} mW")
    args.fb_current_max = _parse_current_max_arg(args.fb_current_max, parser, servo_ids)
    main(args)
