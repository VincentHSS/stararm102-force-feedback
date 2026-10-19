"""空载采样节拍提示；运动与退出清理由原启动器处理。"""

import argparse
import csv
import ctypes
import json
import math
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

APP = Path(__file__).resolve().parent
ROOT = APP.parent
sys.path.insert(0, str(ROOT))
TASKS = (
    "动 J2 肩 + J3 肘：均离开折叠位约 30-45 度，再停住。",
    "固定 J2 肩，只动 J3 肘约 45 度，让前臂明显倾斜，再停住。",
    "固定 J3 肘，只动 J2 肩约 30 度，再停住。",
    "固定 J2 肩 + J3 肘，只动 J4 腕部俯仰约 30 度，再停住。",
)
CHECK_TASKS = (
    "动 J2 + J3，看读数：J2 约 -40 度、J3 约 -50 度，再停住。",
    "固定 J2 约 -40 度，只动 J3 到约 -105 度，再停住。",
    "固定 J3 约 -105 度，只动 J2 到约 -68 度，再停住。",
    "固定 J2 / J3，只动 J4 腕部俯仰到约 +20 度，再停住。",
)
WORKSPACE_TASKS = (
    "保持空载，摆到平时夹住水瓶前的取物姿态（J2 肩 / J3 肘 / J4 腕俯仰），再停住。",
    "保持空载，摆到水瓶刚离开台面的姿态，再停住。",
    "保持空载，摆到平时抬起水瓶后的持物姿态，再停住。",
    "保持空载，摆到平时放下水瓶前的姿态，再停住。",
)


def refresh_leader_reference(recover_shoulder=False):
    from serial import Serial
    from bus import Follower, Leader, SerialCAN
    from config import Config
    from payload import signature
    from teleop import startup_states

    cfg = Config.load(ROOT / "paired.json")
    original = signature(cfg)
    if (ROOT / "logs/profile.active").exists():
        raise ValueError("Restore original motor parameters before restarting")
    if not cfg.calibration_verified or cfg.adapter != "damiao":
        raise ValueError("Use this machine's verified serial bridge configuration")
    payload_path = ROOT / "paired_payload.json"
    payload_cfg = Config.load(payload_path) if payload_path.exists() else None
    if payload_cfg is not None and signature(payload_cfg) != original:
        raise ValueError("Payload follower mapping differs; verify calibration before recentering")
    with Serial(cfg.leader_port, 1000000, timeout=0, write_timeout=0.05) as uart, Serial(
        cfg.follower_port, cfg.follower_baud, timeout=0, write_timeout=0.05
    ) as wire:
        follower = Follower(SerialCAN(wire), cfg)
        follower.verify()
        states = startup_states(cfg, follower, enabled=False, shoulder_recovery=recover_shoulder)
        lead = Leader(uart, cfg).read()
        if lead is None:
            raise ValueError("Incomplete Leader reference feedback")
        changes = []
        for j, a, state in zip(cfg.joints, lead[0], states):
            home = a - math.degrees((state.pos - j.follower_home_rad) / j.direction)
            # 此处是操作者已确认对应姿态后的只读重配对，不与旧主臂参考作差值比较。
            # 映射起点等于从臂当前角度，后续使能与轨迹仍由 hardware 检查。
            changes.append((j, home))
        references = ROOT / "logs/references"
        references.mkdir(parents=True, exist_ok=True)
        cfg.save(references / f"paired_{datetime.now():%Y%m%d_%H%M%S_%f}.json")
        for joint, home in changes:
            joint.leader_home_deg = home
        if payload_cfg is not None:
            payload_cfg.save(references / f"payload_{datetime.now():%Y%m%d_%H%M%S_%f}.json")
            for joint, (_, home) in zip(payload_cfg.joints, changes):
                joint.leader_home_deg = home
        if signature(cfg) != original:
            raise ValueError("Follower calibration signature changed")
        cfg.save(ROOT / "paired.json")
        if payload_cfg is not None:
            payload_cfg.save(payload_path)


def latest_row(path):
    try:
        with path.open("rb") as stream:
            fields = stream.readline().decode("utf-8").strip().split(",")
            stream.seek(max(0, path.stat().st_size - 8192))
            lines = stream.read().splitlines()
        for raw in reversed(lines):
            values = next(csv.reader([raw.decode("utf-8")]))
            if len(values) == len(fields) and values[0] != "elapsed_s":
                return dict(zip(fields, values))
    except (FileNotFoundError, UnicodeDecodeError, ValueError):
        pass
    return None


def save_capture_metadata(path, plan):
    metadata_path = path.with_suffix(".csv.meta.json")
    data = json.loads(metadata_path.read_text(encoding="utf-8"))
    data.update(sample_intervals_s=plan.intervals, capture_complete=plan.complete)
    temporary = metadata_path.with_suffix(".tmp")
    temporary.write_text(json.dumps(data, indent=2), encoding="utf-8")
    temporary.replace(metadata_path)


def guided_loop(child, path, tasks, mode):
    import msvcrt
    import winsound
    from config import Config
    from payload import interior_pose
    from guided_plan import PoseCapture

    cfg = Config.load(ROOT / "paired.json")
    plan = PoseCapture()
    shown = None
    targets = ({2: -40, 3: -50}, {2: -40, 3: -105}, {2: -68, 3: -105}, {2: -68, 3: -105, 4: 20})
    while msvcrt.kbhit():
        msvcrt.getwch()
    while child.poll() is None:
        row = latest_row(path)
        if row is None:
            time.sleep(0.1)
            continue
        now = float(row["elapsed_s"])
        if row.get("fault_reason"):
            raise RuntimeError(row["fault_reason"])
        q = [float(row[f"actual_j{i}_rad"]) for i in range(1, 7)]
        v = [float(row[f"velocity_j{i}_rad_s"]) for i in range(1, 7)]
        valid = (
            row["valid"] == "1"
            and row["state"] == "feedback"
            and all(math.isfinite(x) for x in q + v)
        )
        interior = valid and interior_pose(cfg, q)
        angles = [math.degrees(q[i - 1]) for i in (2, 3, 4)] if valid else None
        errors = []
        if mode == "check" and angles is not None:
            errors = [
                f"J{i} 需在 {goal-5}～{goal+5}°，当前 {math.degrees(q[i-1]):.1f}°"
                for i, goal in targets[plan.index].items()
                if abs(math.degrees(q[i - 1]) - goal) > 5
            ]
        eligible = interior and not errors
        steady = eligible and max(map(abs, v)) <= cfg.payload_max_speed_rad_s
        if mode == "workspace":
            steady = eligible and row.get("settled_pose") == "1"
        if shown != (plan.index, plan.collecting):
            print(
                "\n"
                + (
                    "停住：现在累计 10 秒稳定数据，动了会暂停计时。"
                    if plan.collecting
                    else tasks[plan.index] + " 到位后按回车开始本组采样。"
                ),
                flush=True,
            )
            shown = (plan.index, plan.collecting)
        while msvcrt.kbhit():
            key = msvcrt.getwch()
            if key == "\x1b":
                raise KeyboardInterrupt
            if key == "\r" and not plan.collecting:
                if steady:
                    plan.begin(now)
                    try:
                        winsound.Beep(650, 200)
                    except RuntimeError:
                        pass
                else:
                    print("\n请先停稳，并让各关节离开行程边界。" + ("；".join(errors)), flush=True)
        advanced = plan.update(now, steady)
        actual = (
            ""
            if angles is None
            else " | " + " ".join(f"J{i} {a:6.1f}°" for i, a in zip((2, 3, 4), angles))
        )
        status = f"稳定采样 {plan.seconds:4.1f}/10 秒" if plan.collecting else "准备中：到位按回车"
        print(f"\r第 {min(plan.index+1,4)}/4 组 | {status}{actual}       ", end="", flush=True)
        if advanced:
            save_capture_metadata(path, plan)
            try:
                winsound.Beep(1000, 300)
            except RuntimeError:
                pass
            if plan.complete:
                if mode == "workspace":
                    print("\n四组空载记录完成。请先放稳并支撑从臂；准备时间不限。", flush=True)
                    input("放稳后按回车，才会停止跟随并断使能。")
                return True
        time.sleep(0.1)
    return False


def archive(path):
    if not path.exists():
        return
    destination = path.with_name(f"{path.stem}_{datetime.now():%Y%m%d_%H%M%S_%f}.csv")
    path.rename(destination)
    metadata = path.with_suffix(".csv.meta.json")
    if metadata.exists():
        metadata.rename(destination.with_suffix(".csv.meta.json"))
    print(f"已保留上一轮：{destination.name}")


def main(argv=None):
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--collect", choices=("train", "check", "workspace"), default="train")
    parser.add_argument("--self-check", action="store_true")
    parser.add_argument("--recover-shoulder", action="store_true")
    parser.add_argument(
        "--ready", action="store_true", help="Readiness already confirmed by operator"
    )
    args = parser.parse_args(argv)
    tasks = (
        WORKSPACE_TASKS
        if args.collect == "workspace"
        else CHECK_TASKS if args.collect == "check" else TASKS
    )
    if args.self_check:
        from guided_plan import PoseCapture

        assert not PoseCapture().complete
        assert (APP / "launcher.py").is_file()
        json.loads((ROOT / "paired.json").read_text(encoding="utf-8"))
        print("OK | phase timing / launcher / config | no device opened")
        return 0
    if sys.platform == "win32":
        ctypes.windll.kernel32.SetConsoleTitleW(
            "B601-DM 空载训练" if args.collect == "train" else "B601-DM 空载复测"
        )
    print("B601-DM 空载采样 · 由你确认每组何时开始\n")
    print("先停止其他遥操作，将两臂摆回保存的对应参考姿态。")
    print("夹爪不夹物、无外部接触，人员离开从臂活动范围。\n")
    print("确认两臂姿态对应后，只重记主臂参考偏移；从臂绝对角度基准保留。")
    print("共四组。每组准备时间不限，到位后按回车，再保持静止 10 秒。")
    if args.collect == "workspace":
        print("本轮仅验证你实际取放水瓶的空手姿态，全程不要夹物。")
        print("停稳后才累计有效时间；不会拟合或改动现有自重模型。")
        print("四组完成后会等待你放稳从臂、按回车，再停止跟随。")
    print("第一次回车启动跟随；之后每组摆好姿态，再按一次回车开始本组采样。")
    print("J1 底座左右转；J2 肩抬大臂；J3 肘屈伸小臂；J4 腕部上下点头。")
    print("J5 腕部左右偏；J6 末端拧转；J7 夹爪。本轮主要动 J2、J3、J4。\n")
    for number, task in enumerate(tasks, 1):
        print(f"{number}. {task}")
    print("听到提示音后看本窗口：停留期间主臂所有关节保持不动。\n")
    if args.recover_shoulder:
        print("本次已选肩关节启动恢复：J2 最多超上界 3° 时，先回到界内 5°。")
        print("启动会使能七轴，肩关节最多移动 8°；请先将手离开两臂。\n")
    print("采样期间按 Ctrl+C 或双击“停止.cmd”结束。\n")
    if not args.ready:
        input("准备好后按回车开始；关闭此窗口可取消。")
    from config import Config
    from launcher import single_instance

    with single_instance(Config.load(ROOT / "paired.json")):
        refresh_leader_reference(args.recover_shoulder)
    path = ROOT / "data" / (args.collect + ".csv")
    path.parent.mkdir(exist_ok=True)
    archive(path)
    logs = ROOT / "logs"
    logs.mkdir(exist_ok=True)
    complete = False
    child = None
    close_handler = None
    if sys.platform == "win32":

        @ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_uint)
        def close_handler(event):
            if event not in (2, 5, 6):
                return False
            (ROOT / "stop.request").write_text("stop", encoding="ascii")
            if child is not None:
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    pass
            return True

        ctypes.windll.kernel32.SetConsoleCtrlHandler(close_handler, True)
    try:
        with (logs / "guided_collection.log").open("w", encoding="utf-8") as stream:
            command = [
                sys.executable,
                "-u",
                str(APP / "launcher.py"),
                "--collect",
                args.collect,
                "--guided",
            ]
            if args.recover_shoulder:
                command.append("--recover-shoulder")
            child = subprocess.Popen(
                command,
                cwd=ROOT,
                stdout=stream,
                stderr=subprocess.STDOUT,
                creationflags=subprocess.CREATE_NO_WINDOW if sys.platform == "win32" else 0,
            )
            print("正在检查设备，请保持原位……", flush=True)
            complete = guided_loop(child, path, tasks, args.collect)
    except KeyboardInterrupt:
        print("\n正在停止并恢复参数……", flush=True)
    finally:
        if child is not None and child.poll() is None:
            (ROOT / "stop.request").write_text("stop", encoding="ascii")
            child.wait()
        if close_handler is not None:
            ctypes.windll.kernel32.SetConsoleCtrlHandler(close_handler, False)
    print()
    if child is not None and child.returncode == 0 and complete:
        print("采样结束，电机已失能，原参数已恢复。")
    elif child is not None and child.returncode == 0:
        print("采样已提前停止，电机已失能，原参数已恢复；本轮未完成。")
    else:
        print("本轮未完成，请查看以下原因；不要继续启动其他遥操作。")
        print((logs / "guided_collection.log").read_text(encoding="utf-8")[-2400:])
    input("按回车关闭窗口。")
    return child.returncode if child is not None else 1


if __name__ == "__main__":
    try:
        result = main()
    except Exception as exc:
        print(f"\n未启动或已退出：{exc}")
        input("按回车关闭窗口。")
        result = 1
    raise SystemExit(result)
