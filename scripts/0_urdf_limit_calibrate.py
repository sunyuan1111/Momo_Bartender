#!/usr/bin/env python3
"""Interactive joint-limit calibrator for the current Physical_Agent project.

Workflow:

1. Connect once with the current `physical_agent` config.
2. Release torque so the arm can be moved by hand.
3. For each selected arm joint, manually place it at the safe lower and upper positions.
4. Save the measured limits into a calibration JSON that can later be applied to the URDF.
"""

from __future__ import annotations

import argparse
import json
import math
import re
import tempfile
import threading
import time
from pathlib import Path
from typing import Any
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from _robot_script_common import DEFAULT_CONFIG, DEFAULT_PORT, make_controller, print_json
from update_urdf_limits import collect_urdf_limits

try:
    import pybullet as p
    import pybullet_data
except ImportError:  # pragma: no cover - optional dependency
    p = None
    pybullet_data = None


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "models" / "arm2" / "urdf" / "arm2.urdf"
DEFAULT_OUTPUT = REPO_ROOT / "runtime" / "urdf_limit_calibration.json"
PYBULLET_JOINT_TYPES = {0, 1}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Measure URDF joint limits by hand-guiding the arm. "
            "The measured joint degrees come from the current physical_agent controller mapping."
        )
    )
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="URDF whose current limits will be read.")
    parser.add_argument(
        "--output",
        type=Path,
        default=DEFAULT_OUTPUT,
        help="Where to save the captured calibration JSON.",
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=None,
        help="Joint name to calibrate. Repeat to limit the run to a subset. Defaults to all arm joints present in the URDF.",
    )
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="Keep torque enabled. By default, selected joints are released for hand-guided calibration.",
    )
    parser.add_argument(
        "--no-gui",
        action="store_true",
        help="Disable the PyBullet live viewer and use terminal-only calibration.",
    )
    parser.add_argument(
        "--gui-poll-hz",
        type=float,
        default=15.0,
        help="Refresh rate for the PyBullet live viewer.",
    )
    return parser


def select_joint_names(arm, urdf_joint_limits: dict[str, dict[str, float]], requested: list[str] | None) -> list[str]:
    available = [joint_name for joint_name in arm.config.arm_joint_names if joint_name in urdf_joint_limits]
    if requested is None:
        return available

    invalid = [joint_name for joint_name in requested if joint_name not in available]
    if invalid:
        raise ValueError(
            f"Unsupported joints for URDF limit calibration: {invalid}. "
            f"Available joints: {available}"
        )

    deduped: list[str] = []
    for joint_name in requested:
        if joint_name not in deduped:
            deduped.append(joint_name)
    return deduped


def existing_limit_summary(limits_rad: dict[str, float]) -> dict[str, float]:
    lower = float(limits_rad["lower"])
    upper = float(limits_rad["upper"])
    return {
        "lower_rad": lower,
        "upper_rad": upper,
        "lower_deg": math.degrees(lower),
        "upper_deg": math.degrees(upper),
    }


def read_joint_positions_deg(arm, joint_names: list[str], io_lock: threading.Lock) -> dict[str, float]:
    with io_lock:
        raw_positions = arm.read_present_raw_positions()
    return {
        joint_name: float(arm.joint_raw_to_position_deg(joint_name, raw_positions[joint_name]))
        for joint_name in joint_names
    }


def capture_joint_snapshot(arm, joint_name: str, io_lock: threading.Lock) -> dict[str, Any]:
    with io_lock:
        raw_positions = arm.read_present_raw_positions()
    joint_deg = float(arm.joint_raw_to_position_deg(joint_name, raw_positions[joint_name]))
    return {
        "joint_deg": joint_deg,
        "joint_rad": math.radians(joint_deg),
        "present_raw": int(raw_positions[joint_name]),
        "timestamp": time.time(),
    }


def print_snapshot(joint_name: str, label: str, snapshot: dict[str, Any]) -> None:
    print(
        f"[limit-cal] {joint_name} {label}: "
        f"joint_deg={snapshot['joint_deg']:.3f}, "
        f"joint_rad={snapshot['joint_rad']:.5f}, "
        f"present_raw={snapshot['present_raw']}"
    )


def prompt_capture(arm, joint_name: str, label: str, io_lock: threading.Lock) -> dict[str, Any] | None:
    prompt = (
        f"\n[limit-cal] Move {joint_name} to the safe {label} position."
        "\nPress Enter to capture, type s to skip this joint, or q to quit."
    )
    while True:
        user_input = input(prompt + "\n> ").strip().lower()
        if user_input in {"q", "quit", "exit"}:
            raise KeyboardInterrupt
        if user_input in {"s", "skip"}:
            return None
        if user_input not in {"", "c", "capture"}:
            print("[limit-cal] Press Enter to capture, s to skip, or q to quit.")
            continue

        snapshot = capture_joint_snapshot(arm, joint_name, io_lock)
        print_snapshot(joint_name, label, snapshot)

        confirm = input("Press Enter to accept, r to recapture, s to skip, q to quit.\n> ").strip().lower()
        if confirm in {"", "y", "yes", "ok"}:
            return snapshot
        if confirm in {"q", "quit", "exit"}:
            raise KeyboardInterrupt
        if confirm in {"s", "skip"}:
            return None


class PyBulletLimitViewer:
    def __init__(
        self,
        *,
        urdf_path: Path,
        arm_joint_names: list[str],
        read_positions_deg,
        poll_hz: float,
    ) -> None:
        self.urdf_path = urdf_path
        self.arm_joint_names = list(arm_joint_names)
        self.read_positions_deg = read_positions_deg
        self.poll_hz = max(1.0, float(poll_hz))

        self.client_id: int | None = None
        self.robot_id: int | None = None
        self._joint_index_by_name: dict[str, int] = {}
        self._status_text_id: int | None = None
        self._current_joint: str | None = None
        self._current_label: str | None = None
        self._temp_urdf_path: Path | None = None
        self._stop_event = threading.Event()
        self._thread: threading.Thread | None = None

    @property
    def is_available(self) -> bool:
        return p is not None and pybullet_data is not None

    def start(self) -> bool:
        if not self.is_available:
            print("[limit-cal] PyBullet is not installed. Continuing without GUI.")
            return False

        try:
            visual_urdf = self._prepare_visual_urdf()
            self.client_id = p.connect(p.GUI)
            if self.client_id < 0:
                raise RuntimeError("pybullet GUI connection failed")

            p.setAdditionalSearchPath(pybullet_data.getDataPath(), physicsClientId=self.client_id)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0, physicsClientId=self.client_id)
            p.resetDebugVisualizerCamera(
                cameraDistance=0.6,
                cameraYaw=45,
                cameraPitch=-25,
                cameraTargetPosition=[0.0, 0.08, 0.12],
                physicsClientId=self.client_id,
            )
            p.loadURDF("plane.urdf", physicsClientId=self.client_id)
            self.robot_id = p.loadURDF(
                str(visual_urdf),
                useFixedBase=True,
                flags=p.URDF_USE_INERTIA_FROM_FILE,
                physicsClientId=self.client_id,
            )
            self._joint_index_by_name = self._collect_joint_indices()
            self._update_status_text()

            self._thread = threading.Thread(target=self._run, name="urdf-limit-viewer", daemon=True)
            self._thread.start()
            return True
        except Exception as exc:
            self.stop()
            print(f"[limit-cal] Failed to start PyBullet viewer: {exc}")
            return False

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None

        if self.client_id is not None:
            try:
                if p is not None and p.isConnected(self.client_id):
                    p.disconnect(physicsClientId=self.client_id)
            except Exception:
                pass
            self.client_id = None

        if self._temp_urdf_path is not None and self._temp_urdf_path.exists():
            self._temp_urdf_path.unlink(missing_ok=True)
            self._temp_urdf_path = None

    def set_focus(self, joint_name: str | None, label: str | None = None) -> None:
        self._current_joint = joint_name
        self._current_label = label
        self._update_status_text()

    def _prepare_visual_urdf(self) -> Path:
        text = self.urdf_path.read_text(encoding="utf-8")
        model_root = self.urdf_path.parent.parent

        def replace_package(match: re.Match[str]) -> str:
            package_name = match.group(1)
            if package_name == model_root.name:
                return f"{model_root.as_posix()}/"
            return match.group(0)

        patched = re.sub(r"package://([^/]+)/", replace_package, text)
        if patched == text:
            return self.urdf_path

        temp_file = tempfile.NamedTemporaryFile(
            mode="w",
            suffix=".urdf",
            prefix=f"{self.urdf_path.stem}_pybullet_",
            delete=False,
        )
        with temp_file:
            temp_file.write(patched)
        self._temp_urdf_path = Path(temp_file.name)
        return self._temp_urdf_path

    def _collect_joint_indices(self) -> dict[str, int]:
        assert self.robot_id is not None
        assert self.client_id is not None

        joint_index_by_name: dict[str, int] = {}
        joint_count = p.getNumJoints(self.robot_id, physicsClientId=self.client_id)
        for joint_index in range(joint_count):
            joint_info = p.getJointInfo(self.robot_id, joint_index, physicsClientId=self.client_id)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = int(joint_info[2])
            if joint_type not in PYBULLET_JOINT_TYPES:
                continue
            joint_index_by_name[joint_name] = joint_index
        return joint_index_by_name

    def _update_status_text(self) -> None:
        if self.client_id is None or self.robot_id is None:
            return
        if p is None or not p.isConnected(self.client_id):
            return

        if self._status_text_id is not None:
            p.removeUserDebugItem(self._status_text_id, physicsClientId=self.client_id)
            self._status_text_id = None

        if self._current_joint is None:
            message = "URDF limit calibration"
        elif self._current_label is None:
            message = f"Current joint: {self._current_joint}"
        else:
            message = f"Move {self._current_joint} to {self._current_label}"

        self._status_text_id = p.addUserDebugText(
            message,
            textPosition=[0.0, -0.15, 0.30],
            textColorRGB=[1.0, 0.2, 0.2],
            textSize=1.3,
            physicsClientId=self.client_id,
        )

    def _run(self) -> None:
        assert self.client_id is not None
        assert self.robot_id is not None

        sleep_sec = 1.0 / self.poll_hz
        while not self._stop_event.is_set():
            try:
                if p is None or not p.isConnected(self.client_id):
                    break
                positions_deg = self.read_positions_deg()
                for joint_name, joint_deg in positions_deg.items():
                    joint_index = self._joint_index_by_name.get(joint_name)
                    if joint_index is None:
                        continue
                    p.resetJointState(
                        self.robot_id,
                        joint_index,
                        targetValue=math.radians(float(joint_deg)),
                        physicsClientId=self.client_id,
                    )
                p.stepSimulation(physicsClientId=self.client_id)
            except Exception:
                break
            time.sleep(sleep_sec)


def summarize_joint_capture(
    joint_name: str,
    existing_limits_rad: dict[str, float],
    first_snapshot: dict[str, Any],
    second_snapshot: dict[str, Any],
) -> dict[str, Any]:
    ordered = sorted(
        [("first_capture", dict(first_snapshot)), ("second_capture", dict(second_snapshot))],
        key=lambda item: float(item[1]["joint_deg"]),
    )
    lower_label, lower_capture = ordered[0]
    upper_label, upper_capture = ordered[1]

    lower_deg = float(lower_capture["joint_deg"])
    upper_deg = float(upper_capture["joint_deg"])
    lower_rad = math.radians(lower_deg)
    upper_rad = math.radians(upper_deg)

    return {
        "joint_name": joint_name,
        "existing_urdf_limit": existing_limit_summary(existing_limits_rad),
        "measured_limit_deg": {
            "lower_deg": lower_deg,
            "upper_deg": upper_deg,
            "span_deg": upper_deg - lower_deg,
            "mid_deg": 0.5 * (upper_deg + lower_deg),
        },
        "measured_limit_rad": {
            "lower_rad": lower_rad,
            "upper_rad": upper_rad,
            "span_rad": upper_rad - lower_rad,
            "mid_rad": 0.5 * (upper_rad + lower_rad),
        },
        "captures": {
            lower_label: lower_capture,
            upper_label: upper_capture,
        },
    }


def build_joint_limits_payload(
    defaults: dict[str, float],
    urdf_joint_limits: dict[str, dict[str, float]],
    captured: dict[str, Any],
) -> dict[str, Any]:
    joint_limits = {
        joint_name: {"lower": float(limits["lower"]), "upper": float(limits["upper"])}
        for joint_name, limits in urdf_joint_limits.items()
    }
    for joint_name, summary in captured.items():
        measured = dict(summary["measured_limit_rad"])
        joint_limits[joint_name] = {
            "lower": float(measured["lower_rad"]),
            "upper": float(measured["upper_rad"]),
        }
    return {
        "defaults": {
            "effort": float(defaults["effort"]),
            "velocity": float(defaults["velocity"]),
        },
        "joint_limits": joint_limits,
    }


def main() -> None:
    args = build_parser().parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    output_path = args.output.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    defaults, urdf_joint_limits = collect_urdf_limits(urdf_path)

    with make_controller(args.config, args.port) as arm:
        io_lock = threading.Lock()
        joint_names = select_joint_names(arm, urdf_joint_limits, args.joint)
        if not joint_names:
            raise ValueError("No arm joints were selected for calibration.")

        if not args.keep_torque:
            assert arm.bus is not None
            arm.bus.disable_torque(joint_names)
            time.sleep(0.05)
            print(f"[limit-cal] Teaching mode: torque released for joints {joint_names}")

        print(
            "[limit-cal] Instructions:\n"
            "1. Move each joint by hand to its safe lower and upper limits.\n"
            "2. Capture both positions when prompted.\n"
            "3. The measured degrees come from the current controller mapping.\n"
            "4. This script only measures limits. It does not change zero_position_raw."
        )

        viewer = None
        if not args.no_gui:
            viewer = PyBulletLimitViewer(
                urdf_path=urdf_path,
                arm_joint_names=joint_names,
                read_positions_deg=lambda: read_joint_positions_deg(arm, joint_names, io_lock),
                poll_hz=args.gui_poll_hz,
            )
            if viewer.start():
                print("[limit-cal] PyBullet viewer started.")
            else:
                viewer = None

        try:
            captured: dict[str, Any] = {}
            for joint_name in joint_names:
                print(
                    f"\n[limit-cal] ===== {joint_name} =====\n"
                    f"[limit-cal] Existing URDF limit: {existing_limit_summary(urdf_joint_limits[joint_name])}"
                )
                if viewer is not None:
                    viewer.set_focus(joint_name, "lower limit")
                first_snapshot = prompt_capture(arm, joint_name, "lower", io_lock)
                if first_snapshot is None:
                    print(f"[limit-cal] Skipped {joint_name}")
                    continue

                if viewer is not None:
                    viewer.set_focus(joint_name, "upper limit")
                second_snapshot = prompt_capture(arm, joint_name, "upper", io_lock)
                if second_snapshot is None:
                    print(f"[limit-cal] Skipped {joint_name}")
                    continue

                summary = summarize_joint_capture(
                    joint_name,
                    urdf_joint_limits[joint_name],
                    first_snapshot,
                    second_snapshot,
                )
                captured[joint_name] = summary
                print(
                    f"[limit-cal] {joint_name} measured lower/upper(deg)="
                    f"{summary['measured_limit_deg']['lower_deg']:.3f} / "
                    f"{summary['measured_limit_deg']['upper_deg']:.3f}"
                )
        finally:
            if viewer is not None:
                viewer.stop()

    if not captured:
        raise RuntimeError("No joint limits were captured.")

    payload = {
        "version": 1,
        "mode": "joint_limit_calibration",
        "captured_at_unix_s": time.time(),
        "config_path": str(args.config.expanduser().resolve()),
        "port": str(args.port),
        "urdf_source_path": str(urdf_path),
        "torque_mode": "enabled" if args.keep_torque else "disabled",
        "selected_joints": joint_names,
        "joint_results": captured,
        "joint_limits_payload": build_joint_limits_payload(defaults, urdf_joint_limits, captured),
        "note": (
            "Measured limits are expressed in current controller joint-space degrees/radians. "
            "If zero_position_raw or joint direction/gear_ratio is wrong, recalibrate after fixing them."
        ),
    }

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print_json(
        {
            "output": str(output_path),
            "captured_joint_count": len(captured),
            "captured_joints": sorted(captured),
        }
    )


if __name__ == "__main__":
    main()
