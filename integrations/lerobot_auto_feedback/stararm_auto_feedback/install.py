from __future__ import annotations

import argparse
import ast
import hashlib
import importlib.util
import json
from pathlib import Path

MARKER = "_stararm_automatic_feedback"


def patch_source(source):
    tree = ast.parse(source)
    functions = [n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == "teleop_loop"]
    if len(functions) != 1:
        raise ValueError("Expected exactly one official teleop_loop function")
    fn = functions[0]
    if MARKER in ast.get_source_segment(source, fn):
        raise ValueError("Automatic feedback hook already installed")
    arguments = {a.arg for a in fn.args.args}
    if not {"teleop", "robot"} <= arguments:
        raise ValueError("Unsupported teleop_loop arguments")
    calls = [
        n
        for n in ast.walk(fn)
        if isinstance(n, ast.Call)
        and isinstance(n.func, ast.Attribute)
        and isinstance(n.func.value, ast.Name)
        and n.func.value.id == "robot"
        and n.func.attr == "send_action"
    ]
    if len(calls) != 1:
        raise ValueError("Expected exactly one robot.send_action call in teleop_loop")
    call = calls[0]
    parents = {child: parent for parent in ast.walk(fn) for child in ast.iter_child_nodes(parent)}
    stmt = parents[call]
    if (
        isinstance(stmt, ast.Assign)
        and len(stmt.targets) == 1
        and isinstance(stmt.targets[0], ast.Name)
    ):
        result_name = stmt.targets[0].id
        replace = None
    elif isinstance(stmt, ast.Expr):
        result_name = "_stararm_sent_action"
        replace = (
            " " * stmt.col_offset
            + result_name
            + " = "
            + ast.get_source_segment(source, call)
            + "\n"
        )
    else:
        raise ValueError("Unsupported send_action statement; source was not modified")
    if any(
        isinstance(n, (ast.FunctionDef, ast.AsyncFunctionDef, ast.Lambda))
        and n is not fn
        and any(c is call for c in ast.walk(n))
        for n in ast.walk(fn)
    ):
        raise ValueError("send_action must be inside the official loop, not a nested helper")
    lines = source.splitlines(keepends=True)
    first = fn.body[0]
    has_doc = (
        isinstance(first, ast.Expr)
        and isinstance(first.value, ast.Constant)
        and isinstance(first.value.value, str)
    )
    start = first.end_lineno if has_doc else first.lineno - 1
    stop = fn.end_lineno
    body = lines[start:stop]
    relative_end = stmt.end_lineno - start
    body[relative_end:relative_end] = [" " * stmt.col_offset + f"{MARKER}.update({result_name})\n"]
    if replace is not None:
        body[stmt.lineno - 1 - start : stmt.end_lineno - start] = [replace]
    prefix = [
        "    from stararm_auto_feedback import AutomaticFeedback\n",
        f"    {MARKER} = AutomaticFeedback(teleop, robot)\n",
        "    try:\n",
    ]
    wrapped = ["    " + line if line.strip() else line for line in body]
    suffix = ["    finally:\n", f"        {MARKER}.close()\n"]
    lines[start:stop] = prefix + wrapped + suffix
    patched = "".join(lines)
    ast.parse(patched)
    return patched


def digest(source):
    return hashlib.sha256(source.encode("utf-8")).hexdigest()


def script_path(explicit=None):
    if explicit is not None:
        result = Path(explicit).resolve()
    else:
        try:
            spec = importlib.util.find_spec("lerobot.scripts.lerobot_teleoperate")
        except (ImportError, ModuleNotFoundError) as exc:
            raise ValueError(
                "Run this installer inside the working LeRobot Python environment"
            ) from exc
        if spec is None or spec.origin is None:
            raise ValueError("Official LeRobot teleoperation module was not found")
        result = Path(spec.origin).resolve()
    if not result.is_file() or result.name != "lerobot_teleoperate.py":
        raise ValueError("Expected the official lerobot_teleoperate.py file")
    return result


def install(path):
    source = path.read_text(encoding="utf-8-sig")
    backup = path.with_name(path.name + ".stararm-feedback.bak")
    record = path.with_name(path.name + ".stararm-feedback.json")
    if backup.exists() or record.exists():
        if backup.exists() and record.exists():
            metadata = json.loads(record.read_text(encoding="utf-8"))
            if digest(source) == metadata.get("patched_sha256"):
                return False
        raise ValueError("Existing backup or changed source detected; inspect before reinstalling")
    patched = patch_source(source)
    backup.write_bytes(path.read_bytes())
    record.write_text(
        json.dumps({"original_sha256": digest(source), "patched_sha256": digest(patched)}, indent=2)
        + "\n",
        encoding="utf-8",
    )
    try:
        path.write_text(patched, encoding="utf-8", newline="\n")
    except Exception:
        # Keep the original source recoverable even if writing the hook fails.
        path.write_bytes(backup.read_bytes())
        raise
    return True


def remove(path):
    backup = path.with_name(path.name + ".stararm-feedback.bak")
    record = path.with_name(path.name + ".stararm-feedback.json")
    if not backup.is_file() or not record.is_file():
        raise ValueError("No complete automatic-feedback backup found")
    metadata = json.loads(record.read_text(encoding="utf-8"))
    if digest(path.read_text(encoding="utf-8-sig")) != metadata["patched_sha256"]:
        raise ValueError("LeRobot source changed after installation; refusing to overwrite it")
    if digest(backup.read_text(encoding="utf-8-sig")) != metadata["original_sha256"]:
        raise ValueError("Original backup hash mismatch")
    path.write_bytes(backup.read_bytes())
    backup.unlink()
    record.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Add automatic feedback to the official LeRobot teleoperation loop"
    )
    parser.add_argument("--script", type=Path, help="Explicit official lerobot_teleoperate.py path")
    parser.add_argument("--remove", action="store_true", help="Restore the original backup")
    args = parser.parse_args(argv)
    try:
        path = script_path(args.script)
        if args.remove:
            remove(path)
            print(f"Restored official teleoperation: {path}")
        else:
            changed = install(path)
            print(f'Automatic feedback {"installed" if changed else "already installed"}: {path}')
        return 0
    except (ValueError, OSError, SyntaxError) as exc:
        parser.exit(1, f"{exc}\n")


if __name__ == "__main__":
    raise SystemExit(main())
