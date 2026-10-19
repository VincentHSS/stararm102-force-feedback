"""额外负载展示入口；操作者准备完成后才检查和启动。"""

import argparse
import ctypes
import subprocess
import sys
from pathlib import Path

APP = Path(__file__).resolve().parent
ROOT = APP.parent
sys.path.insert(0, str(ROOT))


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="B601 extra-load demo; waits for operator readiness"
    )
    parser.add_argument(
        "--self-check", action="store_true", help="validate files without opening devices"
    )
    parser.add_argument("--ready", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument(
        "--check-damping", action="store_true", help="check Leader damping with follower disabled"
    )
    parser.add_argument(
        "--recover-shoulder",
        action="store_true",
        help="allow bounded J2 startup return with unloaded arms",
    )
    args = parser.parse_args(argv)
    if args.check_damping and args.recover_shoulder:
        parser.error("Damping-only check does not move the follower")
    from config import Config
    from extra_load import load_observer
    from launcher import single_instance
    from calibration_guide import refresh_leader_reference

    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    cfg = Config.load(ROOT / ("paired.json" if args.check_damping else "paired_payload.json"))
    if not args.check_damping and cfg.feedback_mode != "extra_load":
        raise ValueError("请先完成额外负载模式的空载标定与验证。")
    if not args.check_damping:
        load_observer(cfg)
    if args.self_check:
        print(
            "OK | config validated | no device opened"
            if args.check_damping
            else "OK | extra-load config / model / unloaded validation | no device opened"
        )
        return 0
    print("B601-DM 额外负载反馈\n")
    if args.check_damping:
        print("本轮只检查主臂 J2/J3 阻尼，从臂保持失能并得到支撑。")
        print("先轻松 5 秒，再增加阻尼 12 秒，最后回到轻松并释放。")
        print("请握住主臂，轻轻来回移动 J2（肩）和 J3（肘）比较手感。")
    else:
        print("空载模型已固定；拿起物体后根据信号增加阻尼，不显示公斤数。")
    print("J2 = 肩部抬落大臂，J3 = 肘部屈伸小臂，J4 = 腕部俯仰。")
    if args.recover_shoulder:
        print("空载启动回收：J2 最多超上界 3° 时先收回到 -5°，最多移动 8°。")
        print("回收会使能七轴，其余轴保持原位；先将双手离开两臂。")
    if not args.ready:
        if args.check_damping:
            print("请放稳并支撑从臂；准备时间不限，按回车开始主臂手感比较。")
        else:
            print("先摆好两臂对应姿态，保持空载，清空从臂活动范围。")
            print("准备时间不限；按回车表示姿态已对应、双手已离开，可以使能跟随。")
        input("准备好后按回车开始；关闭窗口可取消。")
    else:
        print("现场准备已确认，开始设备检查。")
    # 仅重记主臂参考，不改变从臂零位或已训练的重力角度基准。
    if not args.check_damping:
        with single_instance(cfg):
            if args.recover_shoulder:
                refresh_leader_reference(True)
            else:
                refresh_leader_reference()
    child, handler = None, None
    if sys.platform == "win32":

        @ctypes.WINFUNCTYPE(ctypes.c_int, ctypes.c_uint)
        def handler(event):
            if event not in (2, 5, 6):
                return False
            (ROOT / "stop.request").write_text("stop", encoding="ascii")
            if child is not None:
                try:
                    child.wait(timeout=4)
                except subprocess.TimeoutExpired:
                    pass
            return True

        ctypes.windll.kernel32.SetConsoleCtrlHandler(handler, True)
    try:
        print("正在检查并启动；Ctrl+C 或双击“停止.cmd”结束。", flush=True)
        command = [
            sys.executable,
            "-u",
            str(APP / "launcher.py"),
            "--check-damping" if args.check_damping else "--payload",
        ]
        if args.recover_shoulder:
            command.append("--recover-shoulder")
        child = subprocess.Popen(command, cwd=ROOT)
        return child.wait()
    except KeyboardInterrupt:
        print("\n正在停止并恢复参数……", flush=True)
        return 0
    finally:
        if child is not None and child.poll() is None:
            (ROOT / "stop.request").write_text("stop", encoding="ascii")
            child.wait()
        if handler is not None:
            ctypes.windll.kernel32.SetConsoleCtrlHandler(handler, False)


if __name__ == "__main__":
    try:
        code = main()
    except (Exception, KeyboardInterrupt) as exc:
        print(f"\n未启动或已退出：{exc}")
        code = 1
    if sys.stdin.isatty() and "--self-check" not in sys.argv[1:]:
        input("按回车关闭窗口。")
    raise SystemExit(code)
