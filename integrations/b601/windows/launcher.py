"""Windows 展示入口，使用包内运行时与已保存的配对配置。"""

import argparse
import ctypes
import json
import logging
import math
import sys
import time
import traceback
from datetime import datetime
from contextlib import contextmanager
from ctypes import wintypes
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import Config
from response_profile import apply_trial, restore
from teleop import hardware

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "logs"
STOP = ROOT / "stop.request"


@contextmanager
def single_instance(cfg):
    api = ctypes.WinDLL("kernel32", use_last_error=True)
    api.CreateMutexW.argtypes = [ctypes.c_void_p, wintypes.BOOL, wintypes.LPCWSTR]
    api.CreateMutexW.restype = wintypes.HANDLE
    api.ReleaseMutex.argtypes = [wintypes.HANDLE]
    api.CloseHandle.argtypes = [wintypes.HANDLE]
    handle = api.CreateMutexW(
        None, True, f"Local\\StarArmB601_{cfg.leader_port}_{cfg.follower_port}"
    )
    if not handle:
        raise ctypes.WinError(ctypes.get_last_error())
    try:
        if ctypes.get_last_error() == 183:
            raise RuntimeError("程序已在运行，请先使用停止入口结束当前运行")
        yield
    finally:
        api.ReleaseMutex(handle)
        api.CloseHandle(handle)


def sleep(seconds):
    if STOP.exists():
        raise KeyboardInterrupt
    time.sleep(seconds)


def saved_parameters(cfg):
    rows = json.loads((LOGS / "acc_dec_original.json").read_text(encoding="utf-8"))
    expected = {(j.motor_id, rid) for j in cfg.joints[:6] for rid in (4, 5)}
    if len(rows) != 12 or {(row[0], row[1]) for row in rows} != expected:
        raise ValueError("原参数备份无效，请联系维护人员")
    for motor, rid, value in rows:
        if not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("原参数备份无效")
        if (rid == 4 and value <= 0) or (rid == 5 and value >= 0):
            raise ValueError("原参数备份方向无效")
    return rows


def run(cfg, recover=False, collection=None, recover_shoulder=False, guided=False):
    from serial import Serial

    active = LOGS / "profile.active"
    with single_instance(cfg):
        if recover:
            if not active.exists():
                logging.info("没有待恢复的参数。")
                return
            restore(cfg, saved_parameters(cfg), Serial)
            active.unlink(missing_ok=True)
            logging.info("原参数已恢复，电机保持失能。")
            return
        if active.exists():
            raise RuntimeError("上次未正常退出，请先双击“恢复参数.cmd”，确认恢复成功后再启动")
        STOP.unlink(missing_ok=True)
        original = []
        try:
            apply_trial(cfg, original, Serial)
            logging.info("B601-DM | %s / %s | ACC/DEC 4/-4", cfg.leader_port, cfg.follower_port)
            if not collection and (LOGS / "execute.csv").exists():
                (LOGS / "execute.csv").rename(
                    LOGS / f"execute_{datetime.now():%Y%m%d_%H%M%S_%f}.csv"
                )
            hardware(
                cfg,
                execute=True,
                recover_shoulder=recover_shoulder,
                diagnostics=(
                    (ROOT / "data" / (collection + ".csv")) if collection else LOGS / "execute.csv"
                ),
                collect_unloaded=collection is not None,
                guided_collection=guided,
                duration=0 if guided else 60 if collection else 0,
                sleep=sleep,
            )
        finally:
            # 即使启动中途失败，也尝试恢复全部已备份参数；失败时保留恢复标记。
            restore(cfg, original, Serial)
            active.unlink(missing_ok=True)


def main():
    sys.stdout.reconfigure(encoding="utf-8")
    sys.stderr.reconfigure(encoding="utf-8")
    parser = argparse.ArgumentParser()
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--self-check", action="store_true")
    mode.add_argument("--restore", action="store_true")
    mode.add_argument("--collect", choices=("train", "check"))
    mode.add_argument("--fit", action="store_true")
    mode.add_argument("--payload", action="store_true")
    parser.add_argument("--recover-shoulder", action="store_true")
    parser.add_argument("--guided", action="store_true")
    args = parser.parse_args()
    if args.recover_shoulder and args.collect is None:
        parser.error("--recover-shoulder is available only for confirmed unloaded collection")
    if args.guided and args.collect is None:
        parser.error("--guided requires unloaded collection")
    try:
        cfg = Config.load(ROOT / ("paired_payload.json" if args.payload else "paired.json"))
    except Exception as exc:
        print(f"Configuration missing/invalid; finish calibration first: {exc}")
        return 1
    if args.self_check:
        from serial import Serial

        print("OK | bundled Python / pyserial / config | no device opened")
        return 0
    LOGS.mkdir(exist_ok=True)
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(message)s",
        handlers=[
            logging.FileHandler(LOGS / "runtime.log", encoding="utf-8"),
            logging.StreamHandler(),
        ],
    )
    try:
        if args.fit:
            from payload_tools import main as fit_main

            with single_instance(cfg):
                if (LOGS / "profile.active").exists():
                    raise RuntimeError("Restore original motor parameters before fitting")
                stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
                profile_path = ROOT / f"extra_load_profile_{stamp}.json"
                new_config = ROOT / f"paired_payload_{stamp}.json"
                common = ["--config", str(ROOT / "paired.json")]
                code = fit_main(
                    ["fit", "--mode", "extra_load"]
                    + common
                    + [
                        "--model",
                        str(ROOT / "ReBot_Arm_DM.urdf"),
                        "--train",
                        str(ROOT / "data/train.csv"),
                        "--validation",
                        str(ROOT / "data/check.csv"),
                        "--output",
                        str(profile_path),
                    ]
                )
                if code:
                    return code
                code = fit_main(
                    ["enable"]
                    + common
                    + [
                        "--profile",
                        str(profile_path),
                        "--output",
                        str(new_config),
                    ]
                )
                if code:
                    return code
                Config.load(new_config).validate()
                previous = ROOT / "paired_payload.json"
                if previous.exists():
                    references = LOGS / "references"
                    references.mkdir(exist_ok=True)
                    Config.load(previous).save(references / f"payload_before_fit_{stamp}.json")
                new_config.replace(previous)
                print("空载验证通过，额外负载配置已生成；实物反馈仍需测试。")
                return 0
        if args.payload:
            from extra_load import load_observer

            load_observer(cfg)
        run(
            cfg,
            recover=args.restore,
            collection=args.collect,
            recover_shoulder=args.recover_shoulder,
            guided=args.guided,
        )
    except KeyboardInterrupt:
        logging.info("已停止；退出清理完成。")
    except Exception as exc:
        logging.error("运行失败：%s", exc)
        with (LOGS / "runtime.log").open("a", encoding="utf-8") as stream:
            stream.write(traceback.format_exc())
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
