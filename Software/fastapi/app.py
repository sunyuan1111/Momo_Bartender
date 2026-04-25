from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import threading
import time
from copy import deepcopy
from datetime import datetime
from pathlib import Path
from typing import Any

from fastapi import FastAPI, HTTPException, Query
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles


ROOT = Path(__file__).resolve().parents[2]
APP_ROOT = Path(__file__).resolve().parent
STATIC_ROOT = APP_ROOT / "static"
ASSET_ROOT = ROOT / "Software" / "Web" / "pic"
RECORDINGS_ROOT = ROOT / "recordings"
PLAYBACK_SCRIPT = ROOT / "scripts" / "2_playback_trajectory.py"
STATE_SCRIPT = ROOT / "scripts" / "0_robot_get_state.py"

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 1999
SERVER_HOST = os.getenv("MOMOTENDER_FASTAPI_HOST", DEFAULT_HOST)
SERVER_PORT = int(os.getenv("MOMOTENDER_FASTAPI_PORT", DEFAULT_PORT))
OUTPUT_TAIL_CHARS = 6000
ROBOT_STATE_TIMEOUT_SEC = 8.0

ACTION_ORDER = [
    "home",
    "begin",
    "daojiu1",
    "daojiu22",
    "daojiu33",
    "shake1",
    "end",
]

ACTION_LABELS = {
    "home": "回原点",
    "begin": "开始姿态",
    "daojiu1": "倒酒 1",
    "daojiu22": "倒酒 22",
    "daojiu33": "倒酒 33",
    "shake1": "摇壶",
    "end": "收尾",
}

STATE_LOCK = threading.Lock()
RUN_LOCK = threading.Lock()
CURRENT_PROCESS: subprocess.Popen[str] | None = None

STATUS: dict[str, Any] = {
    "busy": False,
    "state": "idle",
    "message": "FastAPI 服务已启动，等待动作",
    "active_action": None,
    "started_at": None,
    "finished_at": None,
    "duration_sec": None,
    "returncode": None,
    "command": None,
    "last_result": None,
    "last_error": None,
    "stdout_tail": "",
    "stderr_tail": "",
    "logs": ["FastAPI 服务已启动，等待动作"],
}

app = FastAPI(
    title="Momo Bartender FastAPI",
    description="Run recorded bartender trajectories from HTTP endpoints.",
    version="0.1.0",
)

app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=False,
    allow_methods=["*"],
    allow_headers=["*"],
)

if STATIC_ROOT.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_ROOT)), name="static")

if ASSET_ROOT.exists():
    app.mount("/assets", StaticFiles(directory=str(ASSET_ROOT)), name="assets")


def now_text() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def tail_text(value: str, limit: int = OUTPUT_TAIL_CHARS) -> str:
    if len(value) <= limit:
        return value
    return value[-limit:]


def update_status(**changes: Any) -> None:
    message = changes.get("message")
    with STATE_LOCK:
        STATUS.update(changes)
        if message:
            STATUS["logs"] = ([f"{now_text()}  {message}"] + STATUS["logs"])[:80]


def status_snapshot() -> dict[str, Any]:
    with STATE_LOCK:
        snapshot = deepcopy(STATUS)
    snapshot["actions"] = list_actions()
    snapshot["server"] = {
        "host": SERVER_HOST,
        "port": SERVER_PORT,
        "ui": "/",
        "state": "/state",
        "api_state": "/api/state",
    }
    return snapshot


def ordered_recording_files() -> list[Path]:
    if not RECORDINGS_ROOT.exists():
        return []

    files_by_name = {
        path.stem: path
        for path in RECORDINGS_ROOT.glob("*.json")
        if path.is_file()
    }
    ordered_names = [name for name in ACTION_ORDER if name in files_by_name]
    ordered_names.extend(
        sorted(name for name in files_by_name if name not in set(ordered_names))
    )
    return [files_by_name[name] for name in ordered_names]


def recording_summary(path: Path) -> dict[str, Any]:
    summary: dict[str, Any] = {
        "name": path.stem,
        "label": ACTION_LABELS.get(path.stem, path.stem),
        "path": str(path.relative_to(ROOT)),
        "bytes": path.stat().st_size,
    }
    try:
        payload = json.loads(path.read_text(encoding="utf-8"))
        frames = payload.get("frames")
        if isinstance(frames, list) and frames:
            first_t = float(dict(frames[0]).get("t", 0.0))
            last_t = float(dict(frames[-1]).get("t", first_t))
            summary["frame_count"] = len(frames)
            summary["duration_sec"] = max(0.0, last_t - first_t)
        joint_names = payload.get("joint_names")
        if isinstance(joint_names, list):
            summary["joint_count"] = len(joint_names)
    except Exception as exc:
        summary["warning"] = str(exc)
    return summary


def list_actions() -> list[dict[str, Any]]:
    return [recording_summary(path) for path in ordered_recording_files()]


def find_action(action_name: str) -> dict[str, Any]:
    normalized = action_name.strip().removesuffix(".json")
    for path in ordered_recording_files():
        if path.stem == normalized:
            return recording_summary(path)
    raise HTTPException(status_code=404, detail=f"未知动作: {action_name}")


def playback_command(recording_path: Path) -> list[str]:
    command = [
        sys.executable,
        str(PLAYBACK_SCRIPT),
        str(recording_path),
    ]

    config_path = os.getenv("MOMOTENDER_FASTAPI_CONFIG")
    if config_path:
        command.extend(["--config", config_path])

    device_port = os.getenv("MOMOTENDER_FASTAPI_DEVICE_PORT")
    if device_port:
        command.extend(["--port", device_port])

    return command


def parse_json_output(stdout: str) -> Any:
    try:
        return json.loads(stdout)
    except json.JSONDecodeError:
        start = stdout.find("{")
        end = stdout.rfind("}")
        if start >= 0 and end > start:
            try:
                return json.loads(stdout[start : end + 1])
            except json.JSONDecodeError:
                return None
    return None


def run_playback_worker(action: dict[str, Any]) -> None:
    global CURRENT_PROCESS

    action_name = str(action["name"])
    label = str(action["label"])
    recording_path = ROOT / str(action["path"])
    command = playback_command(recording_path)
    started = time.monotonic()

    update_status(
        busy=True,
        state="running",
        message=f"开始执行 {label}",
        active_action=action,
        started_at=now_text(),
        finished_at=None,
        duration_sec=None,
        returncode=None,
        command=command,
        last_result=None,
        last_error=None,
        stdout_tail="",
        stderr_tail="",
    )

    try:
        process = subprocess.Popen(
            command,
            cwd=str(ROOT),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        with STATE_LOCK:
            CURRENT_PROCESS = process

        stdout, stderr = process.communicate()
        duration_sec = round(time.monotonic() - started, 3)
        result = parse_json_output(stdout)

        if process.returncode == 0:
            update_status(
                busy=False,
                state="done",
                message=f"{label} 执行完成",
                active_action=None,
                finished_at=now_text(),
                duration_sec=duration_sec,
                returncode=process.returncode,
                last_result=result,
                last_error=None,
                stdout_tail=tail_text(stdout),
                stderr_tail=tail_text(stderr),
            )
            return

        error_text = tail_text(stderr or stdout or f"{action_name} 执行失败")
        update_status(
            busy=False,
            state="error",
            message=f"{label} 执行失败",
            active_action=None,
            finished_at=now_text(),
            duration_sec=duration_sec,
            returncode=process.returncode,
            last_result=result,
            last_error=error_text,
            stdout_tail=tail_text(stdout),
            stderr_tail=tail_text(stderr),
        )
    except Exception as exc:
        update_status(
            busy=False,
            state="error",
            message=f"{label} 执行异常",
            active_action=None,
            finished_at=now_text(),
            duration_sec=round(time.monotonic() - started, 3),
            returncode=None,
            last_error=str(exc),
        )
    finally:
        with STATE_LOCK:
            CURRENT_PROCESS = None
        RUN_LOCK.release()


def start_action(action_name: str) -> dict[str, Any]:
    action = find_action(action_name)
    if not RUN_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="机械臂正在执行上一条动作")

    thread = threading.Thread(target=run_playback_worker, args=(action,), daemon=True)
    thread.start()
    return {
        "ok": True,
        "message": f"{action['label']} 已接收",
        "action": action,
        "state": "/state",
    }


def read_robot_state() -> dict[str, Any]:
    if not RUN_LOCK.acquire(blocking=False):
        raise HTTPException(status_code=409, detail="机械臂正在执行动作，暂不读取硬件状态")

    try:
        command = [sys.executable, str(STATE_SCRIPT)]
        config_path = os.getenv("MOMOTENDER_FASTAPI_CONFIG")
        if config_path:
            command.extend(["--config", config_path])
        device_port = os.getenv("MOMOTENDER_FASTAPI_DEVICE_PORT")
        if device_port:
            command.extend(["--port", device_port])

        try:
            completed = subprocess.run(
                command,
                cwd=str(ROOT),
                capture_output=True,
                text=True,
                timeout=ROBOT_STATE_TIMEOUT_SEC,
                check=False,
            )
        except subprocess.TimeoutExpired as exc:
            raise HTTPException(status_code=504, detail=f"读取机械臂状态超时: {exc}") from exc

        payload = parse_json_output(completed.stdout)
        if completed.returncode != 0:
            raise HTTPException(
                status_code=500,
                detail={
                    "message": "读取机械臂状态失败",
                    "returncode": completed.returncode,
                    "stdout": tail_text(completed.stdout),
                    "stderr": tail_text(completed.stderr),
                },
            )
        return {
            "ok": True,
            "command": command,
            "robot_state": payload,
            "stdout_tail": tail_text(completed.stdout),
            "stderr_tail": tail_text(completed.stderr),
        }
    finally:
        RUN_LOCK.release()


@app.get("/")
def index() -> FileResponse:
    return FileResponse(STATIC_ROOT / "index.html")


@app.get("/api/actions")
def api_actions() -> dict[str, Any]:
    return {"actions": list_actions()}


@app.get("/api/state")
@app.get("/state")
def api_state(robot: bool = Query(False, description="空闲时同时读取机械臂硬件状态")) -> dict[str, Any]:
    snapshot = status_snapshot()
    if robot:
        snapshot["robot"] = read_robot_state()
    return snapshot


@app.get("/api/robot-state")
@app.get("/robot-state")
def api_robot_state() -> dict[str, Any]:
    return read_robot_state()


@app.post("/api/actions/{action_name}", status_code=202)
@app.post("/{action_name}", status_code=202)
def api_run_action(action_name: str) -> dict[str, Any]:
    return start_action(action_name)


def main() -> None:
    global SERVER_HOST, SERVER_PORT

    parser = argparse.ArgumentParser(description="Momo Bartender FastAPI service")
    parser.add_argument("--host", default=SERVER_HOST)
    parser.add_argument("--port", type=int, default=SERVER_PORT)
    args = parser.parse_args()
    SERVER_HOST = args.host
    SERVER_PORT = args.port

    import uvicorn

    uvicorn.run(app, host=args.host, port=args.port)


if __name__ == "__main__":
    main()
