"""给运行日志标注明确空载的时间段；只读日志，不控制电机。"""

from __future__ import annotations

import argparse
import csv
import hashlib
import io
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from config import Config
from payload import signature
from payload_tools import new_json


def snapshot(path, clock=time.time):
    path = Path(path)
    if clock() - path.stat().st_mtime > 6:
        raise ValueError("Following log is stale; start following before recording empty poses")
    data = path.read_bytes()
    data = data[: data.rfind(b"\n") + 1]
    header_end = data.find(b"\n") + 1
    last_start = data.rfind(b"\n", 0, len(data) - 1) + 1
    tail = data[:header_end] + data[last_start:]
    rows = list(csv.DictReader(io.StringIO(tail.decode("utf-8-sig"))))
    if not rows or None in rows[-1] or any(v is None for v in rows[-1].values()):
        raise ValueError("No complete diagnostic rows")
    return data, rows


def begin(path, pending, cfg, clock=time.time):
    data, rows = snapshot(path, clock)
    if rows[-1]["state"] != "feedback" or rows[-1]["valid"] != "1":
        raise ValueError("Following is not healthy; empty capture not started")
    metadata = {
        "confirmed_unloaded": True,
        "config_signature": signature(cfg),
        "sample_selection": "operator_confirmed_empty_interval",
        "capture_complete": False,
        "sample_intervals_s": [[float(rows[-1]["elapsed_s"]) + 2.1, None]],
        "source_csv": str(Path(path).resolve()),
        "source_prefix_sha256": hashlib.sha256(data[:4096]).hexdigest(),
        "started_at_unix": clock(),
    }
    new_json(pending, metadata)
    return metadata


def finish(pending, output, cfg, clock=time.time):
    metadata = json.loads(Path(pending).read_text(encoding="utf-8"))
    if (
        metadata.get("confirmed_unloaded") is not True
        or metadata.get("capture_complete") is not False
        or metadata.get("sample_selection") != "operator_confirmed_empty_interval"
        or metadata.get("config_signature") != signature(cfg)
    ):
        raise ValueError("Invalid or completed empty capture")
    data, rows = snapshot(metadata["source_csv"], clock)
    if hashlib.sha256(data[:4096]).hexdigest() != metadata["source_prefix_sha256"]:
        raise ValueError("Following log rotated; repeat the empty capture in the current run")
    start = metadata["sample_intervals_s"][0][0]
    end = float(rows[-1]["elapsed_s"])
    if end - start < 5:
        raise ValueError("No new empty coverage recorded; following may have exited")
    metadata.update(
        capture_complete=True,
        sample_intervals_s=[[start, end]],
        source_sha256=hashlib.sha256(data).hexdigest(),
        completed_at_unix=clock(),
    )
    output = Path(output)
    sidecar = output.with_suffix(output.suffix + ".meta.json")
    if output.exists() or sidecar.exists():
        raise FileExistsError(output)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("xb") as stream:
        stream.write(data)
    new_json(sidecar, metadata)
    temporary = Path(pending).with_suffix(".completed.tmp")
    new_json(temporary, metadata)
    temporary.replace(pending)
    return metadata


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Label operator-confirmed empty poses; no device opened"
    )
    parser.add_argument("action", choices=("begin", "finish"))
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--pending", required=True, type=Path)
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument("--confirmed-unloaded", action="store_true")
    args = parser.parse_args(argv)
    if not args.confirmed_unloaded:
        parser.error("Operator must confirm empty gripper and no external contact")
    try:
        cfg = Config.load(args.config)
        if args.action == "begin":
            if args.csv is None:
                parser.error("begin requires --csv")
            metadata = begin(args.csv, args.pending, cfg)
        else:
            if args.output is None:
                parser.error("finish requires --output")
            metadata = finish(args.pending, args.output, cfg)
        print(json.dumps(metadata, indent=2))
        return 0
    except (OSError, ValueError, KeyError, TypeError) as exc:
        print(f"Empty capture not accepted: {exc}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
