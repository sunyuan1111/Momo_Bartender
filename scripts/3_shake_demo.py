#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from _robot_script_common import add_common_arguments, make_controller, print_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_PATH = REPO_ROOT / "recordings" / "latest_trajectory.json"
SHAKE_JOINTS = ("joint_4", "joint_5", "joint_6")
POSITION_EPS_DEG = 1e-6


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Build a simple shaking motion from the recorded min/max of joint_4/joint_5/joint_6."
        )
    )
    add_common_arguments(parser)
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=DEFAULT_TRAJECTORY_PATH,
        help=f"Input trajectory path. Defaults to {DEFAULT_TRAJECTORY_PATH}.",
    )
    parser.add_argument(
        "--mode",
        choices=("sequential", "parallel"),
        default="sequential",
        help="Shake pattern. sequential moves 4->5->6, parallel moves 4/5/6 together.",
    )
    parser.add_argument(
        "--cycles",
        type=int,
        default=1,
        help="How many shake cycles to run.",
    )
    parser.add_argument(
        "--amplitude-scale",
        type=float,
        default=1.0,
        help=(
            "Scale recorded min/max around the recorded start pose. "
            "0 keeps the start pose, 1 uses the full recorded range."
        ),
    )
    parser.add_argument(
        "--approach-sec",
        type=float,
        default=1.5,
        help="Seconds used to approach the recorded start pose before shaking.",
    )
    parser.add_argument(
        "--segment-sec",
        type=float,
        default=0.45,
        help="Seconds used for each shake segment.",
    )
    parser.add_argument(
        "--settle-sec",
        type=float,
        default=0.05,
        help="Extra wait time after each commanded segment.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print the extracted shake profile without moving the arm.",
    )
    return parser


def load_trajectory(path: Path) -> dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(f"Trajectory file not found: {path}")
    payload = json.loads(path.read_text())
    if payload.get("version") != 1:
        raise ValueError(f"Unsupported trajectory version: {payload.get('version')}")
    if payload.get("mode") != "joint_positions_deg":
        raise ValueError(f"Unsupported trajectory mode: {payload.get('mode')}")
    frames = payload.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("Trajectory must contain a non-empty 'frames' list.")
    return payload


def extract_shake_profile(payload: dict[str, object], amplitude_scale: float) -> dict[str, object]:
    joint_names = [str(joint_name) for joint_name in payload.get("joint_names", [])]
    missing = [joint_name for joint_name in SHAKE_JOINTS if joint_name not in joint_names]
    if missing:
        raise ValueError(f"Trajectory is missing required shake joints: {missing}")

    frames = payload["frames"]
    first_positions = dict(frames[0]["positions_deg"])
    start_pose = {joint_name: float(first_positions[joint_name]) for joint_name in SHAKE_JOINTS}

    joint_ranges: dict[str, dict[str, float]] = {}
    for joint_name in SHAKE_JOINTS:
        values = [float(dict(frame["positions_deg"])[joint_name]) for frame in frames]
        recorded_min = min(values)
        recorded_max = max(values)
        scaled_min = start_pose[joint_name] + amplitude_scale * (recorded_min - start_pose[joint_name])
        scaled_max = start_pose[joint_name] + amplitude_scale * (recorded_max - start_pose[joint_name])
        joint_ranges[joint_name] = {
            "start_deg": start_pose[joint_name],
            "recorded_min_deg": recorded_min,
            "recorded_max_deg": recorded_max,
            "scaled_min_deg": scaled_min,
            "scaled_max_deg": scaled_max,
            "recorded_span_deg": recorded_max - recorded_min,
            "scaled_span_deg": scaled_max - scaled_min,
        }

    return {
        "input": str(payload.get("input_path", "")),
        "shake_joints": list(SHAKE_JOINTS),
        "start_pose_deg": start_pose,
        "joint_ranges_deg": joint_ranges,
    }


def build_sequential_waypoints(profile: dict[str, object], cycles: int) -> list[dict[str, object]]:
    start_pose = dict(profile["start_pose_deg"])
    joint_ranges = dict(profile["joint_ranges_deg"])
    waypoints: list[dict[str, object]] = []

    for cycle_index in range(cycles):
        for joint_name in SHAKE_JOINTS:
            low_pose = dict(start_pose)
            low_pose[joint_name] = float(dict(joint_ranges[joint_name])["scaled_min_deg"])
            high_pose = dict(start_pose)
            high_pose[joint_name] = float(dict(joint_ranges[joint_name])["scaled_max_deg"])
            waypoints.extend(
                [
                    {"label": f"cycle_{cycle_index + 1}_{joint_name}_min", "pose_deg": low_pose},
                    {"label": f"cycle_{cycle_index + 1}_{joint_name}_max", "pose_deg": high_pose},
                    {"label": f"cycle_{cycle_index + 1}_{joint_name}_start", "pose_deg": dict(start_pose)},
                ]
            )
    return waypoints


def build_parallel_waypoints(profile: dict[str, object], cycles: int) -> list[dict[str, object]]:
    start_pose = dict(profile["start_pose_deg"])
    joint_ranges = dict(profile["joint_ranges_deg"])
    all_min_pose = {
        joint_name: float(dict(joint_ranges[joint_name])["scaled_min_deg"])
        for joint_name in SHAKE_JOINTS
    }
    all_max_pose = {
        joint_name: float(dict(joint_ranges[joint_name])["scaled_max_deg"])
        for joint_name in SHAKE_JOINTS
    }

    waypoints: list[dict[str, object]] = []
    for cycle_index in range(cycles):
        waypoints.extend(
            [
                {"label": f"cycle_{cycle_index + 1}_all_min", "pose_deg": dict(all_min_pose)},
                {"label": f"cycle_{cycle_index + 1}_all_max", "pose_deg": dict(all_max_pose)},
                {"label": f"cycle_{cycle_index + 1}_start", "pose_deg": dict(start_pose)},
            ]
        )
    return waypoints


def read_joint_positions_deg(arm, joint_names: tuple[str, ...]) -> dict[str, float]:
    positions_deg = arm.read_present_positions_deg()
    return {joint_name: float(positions_deg[joint_name]) for joint_name in joint_names}


def compute_speed_map(
    current_pose_deg: dict[str, float],
    target_pose_deg: dict[str, float],
    duration_sec: float,
) -> tuple[dict[str, float], dict[str, float]]:
    changed_targets = {
        joint_name: float(target_deg)
        for joint_name, target_deg in target_pose_deg.items()
        if abs(float(target_deg) - current_pose_deg[joint_name]) > POSITION_EPS_DEG
    }
    if not changed_targets:
        return {}, {}

    speed_map = {
        joint_name: abs(changed_targets[joint_name] - current_pose_deg[joint_name]) / duration_sec
        for joint_name in changed_targets
    }
    return changed_targets, speed_map


def move_pose(
    arm,
    *,
    label: str,
    target_pose_deg: dict[str, float],
    duration_sec: float,
    settle_sec: float,
) -> dict[str, float]:
    current_pose_deg = read_joint_positions_deg(arm, SHAKE_JOINTS)
    changed_targets, speed_map = compute_speed_map(current_pose_deg, target_pose_deg, duration_sec)
    if not changed_targets:
        print(f"[shake] {label}: already at target")
        return current_pose_deg

    print(
        f"[shake] {label}: "
        f"targets={{{', '.join(f'{k}: {v:.3f}' for k, v in changed_targets.items())}}}, "
        f"speed_deg_s={{{', '.join(f'{k}: {v:.3f}' for k, v in speed_map.items())}}}"
    )
    arm.move_joints(changed_targets, speed_deg_s=speed_map)
    time.sleep(duration_sec + settle_sec)
    return read_joint_positions_deg(arm, SHAKE_JOINTS)


def main() -> None:
    args = build_parser().parse_args()
    if args.cycles <= 0:
        raise ValueError("--cycles must be positive.")
    if not 0.0 <= args.amplitude_scale <= 1.0:
        raise ValueError("--amplitude-scale must be within [0, 1].")
    if args.approach_sec <= 0:
        raise ValueError("--approach-sec must be positive.")
    if args.segment_sec <= 0:
        raise ValueError("--segment-sec must be positive.")
    if args.settle_sec < 0:
        raise ValueError("--settle-sec must be non-negative.")

    payload = load_trajectory(args.input)
    payload["input_path"] = str(args.input.resolve())
    profile = extract_shake_profile(payload, args.amplitude_scale)
    if args.mode == "sequential":
        waypoints = build_sequential_waypoints(profile, args.cycles)
    else:
        waypoints = build_parallel_waypoints(profile, args.cycles)

    summary = {
        "input": str(args.input.resolve()),
        "mode": args.mode,
        "cycles": int(args.cycles),
        "amplitude_scale": float(args.amplitude_scale),
        "approach_sec": float(args.approach_sec),
        "segment_sec": float(args.segment_sec),
        "settle_sec": float(args.settle_sec),
        "start_pose_deg": profile["start_pose_deg"],
        "joint_ranges_deg": profile["joint_ranges_deg"],
        "waypoint_count": len(waypoints),
        "waypoint_labels": [str(waypoint["label"]) for waypoint in waypoints],
    }
    print_json(summary)

    if args.dry_run:
        return

    with make_controller(args.config, args.port) as arm:
        move_pose(
            arm,
            label="approach_start_pose",
            target_pose_deg=dict(profile["start_pose_deg"]),
            duration_sec=float(args.approach_sec),
            settle_sec=float(args.settle_sec),
        )
        for waypoint in waypoints:
            move_pose(
                arm,
                label=str(waypoint["label"]),
                target_pose_deg=dict(waypoint["pose_deg"]),
                duration_sec=float(args.segment_sec),
                settle_sec=float(args.settle_sec),
            )

        result = {
            "final_pose_deg": read_joint_positions_deg(arm, SHAKE_JOINTS),
            "start_pose_deg": dict(profile["start_pose_deg"]),
            "mode": args.mode,
            "cycles": int(args.cycles),
        }
        print_json(result)


if __name__ == "__main__":
    main()
