from __future__ import annotations

import argparse
import json
import time
from datetime import datetime, timezone
from pathlib import Path
from threading import Event, Thread

from _robot_script_common import add_common_arguments, make_controller, print_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_PATH = REPO_ROOT / "recordings" / "latest_trajectory.json"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record present joint angles into a JSON trajectory file."
    )
    add_common_arguments(parser)
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        default=DEFAULT_TRAJECTORY_PATH,
        help=f"Output JSON trajectory path. Defaults to {DEFAULT_TRAJECTORY_PATH}.",
    )
    parser.add_argument(
        "--duration-sec",
        type=float,
        default=None,
        help="Recording duration in seconds. If omitted, recording stops when you press Enter.",
    )
    parser.add_argument(
        "--sample-hz",
        type=float,
        default=20.0,
        help="Sampling frequency in Hz.",
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=None,
        help="Joint name to record. Repeat to limit the recording to a subset. Defaults to all joints.",
    )
    parser.add_argument(
        "--keep-torque",
        action="store_true",
        help="Keep torque enabled while recording. By default, recorded joints are released for hand-guided teaching.",
    )
    return parser.parse_args()


def select_joint_names(arm, requested: list[str] | None) -> list[str]:
    all_joint_names = [joint.name for joint in arm.config.joints]
    if not requested:
        return all_joint_names

    invalid = [joint_name for joint_name in requested if joint_name not in all_joint_names]
    if invalid:
        raise ValueError(f"Unknown joints requested for recording: {invalid}")

    ordered: list[str] = []
    seen: set[str] = set()
    for joint_name in requested:
        if joint_name not in seen:
            ordered.append(joint_name)
            seen.add(joint_name)
    return ordered


def capture_frame(arm, joint_names: list[str], start_time: float) -> dict[str, object]:
    positions_deg = arm.read_present_positions_deg()
    return {
        "t": round(time.monotonic() - start_time, 6),
        "positions_deg": {joint_name: float(positions_deg[joint_name]) for joint_name in joint_names},
    }


def start_enter_stop_listener(stop_event: Event) -> Thread:
    def wait_for_enter() -> None:
        try:
            input("Recording... press Enter to stop.\n")
        except EOFError:
            pass
        stop_event.set()

    listener = Thread(target=wait_for_enter, daemon=True)
    listener.start()
    return listener


def main() -> None:
    args = parse_args()
    if args.duration_sec is not None and args.duration_sec < 0:
        raise ValueError("--duration-sec must be non-negative.")
    if args.sample_hz <= 0:
        raise ValueError("--sample-hz must be positive.")

    with make_controller(args.config, args.port) as arm:
        joint_names = select_joint_names(arm, args.joint)
        zero_position_raw_by_joint = {
            joint_name: arm.config.require_joint(joint_name).zero_position_raw for joint_name in joint_names
        }
        torque_mode = "enabled" if args.keep_torque else "disabled"
        if not args.keep_torque:
            assert arm.bus is not None
            arm.bus.disable_torque(joint_names)
            time.sleep(0.05)
            print(f"Teaching mode: torque released for joints {joint_names}")

        frames: list[dict[str, object]] = []
        period_sec = 1.0 / args.sample_hz
        start_time = time.monotonic()
        sample_index = 0
        stop_event = Event()
        if args.duration_sec is None:
            start_enter_stop_listener(stop_event)

        while True:
            scheduled_time = sample_index * period_sec
            if args.duration_sec is not None and scheduled_time > args.duration_sec + 1e-9:
                break

            sleep_sec = start_time + scheduled_time - time.monotonic()
            if sleep_sec > 0:
                if stop_event.wait(timeout=sleep_sec):
                    break
            elif stop_event.is_set() and sample_index > 0:
                break

            frames.append(capture_frame(arm, joint_names, start_time))
            sample_index += 1

        actual_duration_sec = float(frames[-1]["t"]) if frames else 0.0
        payload = {
            "version": 1,
            "mode": "joint_positions_deg",
            "recorded_at": datetime.now(timezone.utc).isoformat(),
            "duration_sec": actual_duration_sec,
            "stop_condition": "duration" if args.duration_sec is not None else "enter",
            "sample_hz": float(args.sample_hz),
            "joint_names": joint_names,
            "torque_mode": torque_mode,
            "zero_position_raw_by_joint": zero_position_raw_by_joint,
            "frames": frames,
        }

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    print_json(
        {
            "output": str(args.output),
            "frame_count": len(frames),
            "duration_sec": actual_duration_sec,
            "sample_hz": float(args.sample_hz),
            "joint_names": joint_names,
            "stop_condition": payload["stop_condition"],
            "torque_mode": torque_mode,
        }
    )


if __name__ == "__main__":
    main()
