from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from _robot_script_common import add_common_arguments, make_controller, print_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OVERRIDE_PATH = REPO_ROOT / "recordings" / "joint23_interpolation_override.json"
DEFAULT_JOINTS = ("joint_2", "joint_3")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Record endpoint positions for joints that should be interpolated during playback."
    )
    add_common_arguments(parser)
    parser.add_argument(
        "output",
        type=Path,
        nargs="?",
        default=DEFAULT_OVERRIDE_PATH,
        help=f"Output interpolation override path. Defaults to {DEFAULT_OVERRIDE_PATH}.",
    )
    parser.add_argument(
        "--joint",
        action="append",
        default=None,
        help="Joint name to include. Repeat for multiple joints. Defaults to joint_2 and joint_3.",
    )
    parser.add_argument(
        "--step-deg",
        type=float,
        default=2.0,
        help="Default nudge step shown in the interactive prompt.",
    )
    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=None,
        help="Optional speed used when nudging or setting endpoint joints.",
    )
    parser.add_argument(
        "--interpolation",
        choices=("smoothstep", "linear"),
        default="smoothstep",
        help="Interpolation curve used by playback. Defaults to smoothstep.",
    )
    parser.add_argument(
        "--start",
        action="append",
        default=None,
        metavar="JOINT=DEG",
        help="Non-interactive start endpoint value. Repeat once per selected joint.",
    )
    parser.add_argument(
        "--end",
        action="append",
        default=None,
        metavar="JOINT=DEG",
        help="Non-interactive end endpoint value. Repeat once per selected joint.",
    )
    return parser.parse_args()


def parse_position_items(items: list[str] | None) -> dict[str, float]:
    positions: dict[str, float] = {}
    for item in items or []:
        if "=" not in item:
            raise ValueError(f"Expected endpoint value as joint_name=degrees, got '{item}'.")
        joint_name, value = item.split("=", 1)
        joint_name = joint_name.strip()
        if not joint_name:
            raise ValueError(f"Endpoint item has an empty joint name: '{item}'.")
        positions[joint_name] = float(value)
    return positions


def select_joint_names(arm, requested: list[str] | None) -> list[str]:
    selected = list(requested or DEFAULT_JOINTS)
    all_joint_names = [joint.name for joint in arm.config.joints]
    online_joint_names = list(arm.online_joint_names)

    invalid = [joint_name for joint_name in selected if joint_name not in all_joint_names]
    if invalid:
        raise ValueError(f"Unknown joints requested for interpolation override: {invalid}")

    offline = [joint_name for joint_name in selected if joint_name not in online_joint_names]
    if offline:
        raise ConnectionError(
            f"Requested joints are not available on the bus: {offline}. "
            f"Online joints: {online_joint_names}"
        )

    ordered: list[str] = []
    seen: set[str] = set()
    for joint_name in selected:
        if joint_name not in seen:
            ordered.append(joint_name)
            seen.add(joint_name)
    return ordered


def capture_endpoint(arm, joint_names: list[str]) -> tuple[dict[str, float], dict[str, int]]:
    raw_positions = arm.read_present_raw_positions()
    endpoint_raw = {joint_name: int(raw_positions[joint_name]) for joint_name in joint_names}
    endpoint_deg = {
        joint_name: float(arm.joint_raw_to_position_deg(joint_name, endpoint_raw[joint_name]))
        for joint_name in joint_names
    }
    return endpoint_deg, endpoint_raw


def print_current_positions(arm, joint_names: list[str]) -> None:
    positions_deg, raw_positions = capture_endpoint(arm, joint_names)
    print_json(
        {
            "positions_deg": positions_deg,
            "raw_positions": raw_positions,
        }
    )


def validate_endpoint_positions(arm, joint_names: list[str], positions_deg: dict[str, float], label: str) -> None:
    missing = [joint_name for joint_name in joint_names if joint_name not in positions_deg]
    extra = [joint_name for joint_name in positions_deg if joint_name not in joint_names]
    if missing:
        raise ValueError(f"{label} endpoint is missing joints: {missing}")
    if extra:
        raise ValueError(f"{label} endpoint contains unselected joints: {extra}")

    for joint_name in joint_names:
        arm.joint_position_deg_to_raw(joint_name, float(positions_deg[joint_name]))


def show_prompt_help(joint_names: list[str], step_deg: float) -> None:
    print(
        "\nCommands:\n"
        "  show                      print current selected joint positions\n"
        "  capture                   capture this endpoint\n"
        "  <joint> <delta_deg>        nudge one joint, e.g. joint_2 +2\n"
        "  <joint> +                  nudge one joint by the default positive step\n"
        "  <joint> -                  nudge one joint by the default negative step\n"
        "  nudge <joint> <delta_deg>  same as above\n"
        "  set <joint> <target_deg>   move one joint to an absolute degree target\n"
        "  help                      show this help\n"
        "  quit                      abort without saving\n"
        f"\nSelected joints: {joint_names}\n"
        f"Default manual step: {step_deg} deg\n"
    )


def parse_target_deg(raw_value: str) -> float | None:
    try:
        return float(raw_value)
    except ValueError:
        print(f"Expected a numeric degree value, got '{raw_value}'.")
        return None


def parse_delta_deg(raw_value: str, step_deg: float) -> float | None:
    if raw_value == "+":
        return float(step_deg)
    if raw_value == "-":
        return -float(step_deg)
    return parse_target_deg(raw_value)


def require_interactive_stdin() -> None:
    if sys.stdin.isatty():
        return
    raise SystemExit(
        "Interactive endpoint recording needs a real terminal stdin.\n"
        "Use one of these forms:\n"
        "  conda activate momo\n"
        "  python scripts/2_record_interpolation_override.py recordings/joint23_interpolation_override.json --speed-deg-s 10\n\n"
        "or keep conda run streaming I/O:\n"
        "  conda run -n momo --no-capture-output python scripts/2_record_interpolation_override.py recordings/joint23_interpolation_override.json --speed-deg-s 10\n\n"
        "For non-interactive use, pass both endpoints explicitly, for example:\n"
        "  conda run -n momo python scripts/2_record_interpolation_override.py recordings/joint23_interpolation_override.json "
        "--start joint_2=0 --start joint_3=0 --end joint_2=20 --end joint_3=10"
    )


def interactive_capture_endpoint(
    arm,
    joint_names: list[str],
    *,
    label: str,
    step_deg: float,
    speed_deg_s: float | None,
) -> tuple[dict[str, float], dict[str, int]]:
    print(f"\nMove selected joints to the {label} endpoint, then run 'capture'.")
    show_prompt_help(joint_names, step_deg)
    print_current_positions(arm, joint_names)

    while True:
        try:
            raw_command = input(f"{label}> ").strip()
        except EOFError as exc:
            raise RuntimeError(f"Input ended before capturing the {label} endpoint.") from exc

        if not raw_command:
            print_current_positions(arm, joint_names)
            continue

        parts = raw_command.split()
        command = parts[0].lower()

        if command in {"q", "quit", "exit"}:
            raise KeyboardInterrupt
        if command in {"h", "help", "?"}:
            show_prompt_help(joint_names, step_deg)
            continue
        if command == "show":
            print_current_positions(arm, joint_names)
            continue
        if command == "capture":
            endpoint_deg, endpoint_raw = capture_endpoint(arm, joint_names)
            print_json(
                {
                    "captured": label,
                    "positions_deg": endpoint_deg,
                    "raw_positions": endpoint_raw,
                }
            )
            return endpoint_deg, endpoint_raw

        if command in {"set", "move"}:
            if len(parts) != 3:
                print("Usage: set <joint> <target_deg>")
                continue
            joint_name = parts[1]
            if joint_name not in joint_names:
                print(f"{joint_name} is not selected. Selected joints: {joint_names}")
                continue
            target_deg = parse_target_deg(parts[2])
            if target_deg is None:
                continue
            arm.move_joint(joint_name, target_deg, speed_deg_s=speed_deg_s)
            time.sleep(0.05)
            print_current_positions(arm, joint_names)
            continue

        if command == "nudge" or command in joint_names:
            if command == "nudge":
                if len(parts) != 3:
                    print("Usage: nudge <joint> <delta_deg>")
                    continue
                joint_name = parts[1]
                delta_deg = parse_delta_deg(parts[2], step_deg)
            else:
                if len(parts) != 2:
                    print(f"Usage: {command} <delta_deg>")
                    continue
                joint_name = command
                delta_deg = parse_delta_deg(parts[1], step_deg)

            if joint_name not in joint_names:
                print(f"{joint_name} is not selected. Selected joints: {joint_names}")
                continue
            if delta_deg is None:
                continue
            arm.nudge_joint(joint_name, delta_deg, speed_deg_s=speed_deg_s)
            time.sleep(0.05)
            print_current_positions(arm, joint_names)
            continue

        print(f"Unknown command: {raw_command}. Run 'help' for available commands.")


def build_payload(
    arm,
    *,
    joint_names: list[str],
    interpolation: str,
    start_positions_deg: dict[str, float],
    end_positions_deg: dict[str, float],
    start_raw_positions: dict[str, int] | None,
    end_raw_positions: dict[str, int] | None,
    recording_method: str,
) -> dict[str, object]:
    zero_position_raw_by_joint = {
        joint_name: arm.config.require_joint(joint_name).zero_position_raw for joint_name in joint_names
    }
    waypoints: list[dict[str, object]] = [
        {
            "label": "start",
            "phase": 0.0,
            "positions_deg": {joint_name: float(start_positions_deg[joint_name]) for joint_name in joint_names},
        },
        {
            "label": "end",
            "phase": 1.0,
            "positions_deg": {joint_name: float(end_positions_deg[joint_name]) for joint_name in joint_names},
        },
    ]
    if start_raw_positions is not None:
        waypoints[0]["raw_positions"] = {joint_name: int(start_raw_positions[joint_name]) for joint_name in joint_names}
    if end_raw_positions is not None:
        waypoints[1]["raw_positions"] = {joint_name: int(end_raw_positions[joint_name]) for joint_name in joint_names}

    return {
        "version": 1,
        "mode": "joint_interpolation_override",
        "recorded_at": datetime.now(timezone.utc).isoformat(),
        "recording_method": recording_method,
        "joint_names": joint_names,
        "interpolation": {
            "type": interpolation,
        },
        "timing": {
            "policy": "match_base_trajectory",
            "start_phase": 0.0,
            "end_phase": 1.0,
        },
        "zero_position_raw_by_joint": zero_position_raw_by_joint,
        "waypoints": waypoints,
    }


def main() -> None:
    args = parse_args()
    if args.step_deg <= 0:
        raise ValueError("--step-deg must be positive.")
    if args.speed_deg_s is not None and args.speed_deg_s <= 0:
        raise ValueError("--speed-deg-s must be positive.")

    start_positions_deg = parse_position_items(args.start)
    end_positions_deg = parse_position_items(args.end)
    has_manual_endpoints = bool(start_positions_deg or end_positions_deg)
    if has_manual_endpoints and not (start_positions_deg and end_positions_deg):
        raise ValueError("--start and --end must be provided together.")
    if not has_manual_endpoints:
        require_interactive_stdin()

    with make_controller(args.config, args.port) as arm:
        joint_names = select_joint_names(arm, args.joint)
        if not joint_names:
            raise RuntimeError("No joints were selected for interpolation override recording.")

        if has_manual_endpoints:
            validate_endpoint_positions(arm, joint_names, start_positions_deg, "start")
            validate_endpoint_positions(arm, joint_names, end_positions_deg, "end")
            start_raw_positions = None
            end_raw_positions = None
            recording_method = "manual_args"
        else:
            start_positions_deg, start_raw_positions = interactive_capture_endpoint(
                arm,
                joint_names,
                label="start",
                step_deg=float(args.step_deg),
                speed_deg_s=args.speed_deg_s,
            )
            end_positions_deg, end_raw_positions = interactive_capture_endpoint(
                arm,
                joint_names,
                label="end",
                step_deg=float(args.step_deg),
                speed_deg_s=args.speed_deg_s,
            )
            recording_method = "interactive"

        payload = build_payload(
            arm,
            joint_names=joint_names,
            interpolation=args.interpolation,
            start_positions_deg=start_positions_deg,
            end_positions_deg=end_positions_deg,
            start_raw_positions=start_raw_positions,
            end_raw_positions=end_raw_positions,
            recording_method=recording_method,
        )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(payload, indent=2, ensure_ascii=False) + "\n")

    print_json(
        {
            "output": str(args.output),
            "joint_names": payload["joint_names"],
            "interpolation": payload["interpolation"],
            "timing": payload["timing"],
            "recording_method": payload["recording_method"],
        }
    )


if __name__ == "__main__":
    main()
