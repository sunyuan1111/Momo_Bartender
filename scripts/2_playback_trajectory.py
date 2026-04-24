from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from _robot_script_common import add_common_arguments, make_controller, print_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_PATH = REPO_ROOT / "recordings" / "latest_trajectory.json"
POSITION_EPS_DEG = 1e-6


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Replay a recorded JSON trajectory in one process."
    )
    add_common_arguments(parser)
    parser.add_argument(
        "input",
        type=Path,
        nargs="?",
        default=DEFAULT_TRAJECTORY_PATH,
        help=f"Input JSON trajectory path. Defaults to {DEFAULT_TRAJECTORY_PATH}.",
    )
    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=None,
        help="Optional fixed playback speed in deg/s for all joints. Defaults to automatic per-segment speeds.",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=0.5,
        help="Seconds to wait at the end of playback before disconnecting.",
    )
    parser.add_argument(
        "--allow-zero-mismatch",
        action="store_true",
        help="Allow playback even if the file zero references do not match the active config.",
    )
    parser.add_argument(
        "--approach-sec",
        type=float,
        default=2.0,
        help="Seconds used to interpolate from the current pose to the first recorded pose before playback.",
    )
    parser.add_argument(
        "--playback-hz",
        type=float,
        default=30.0,
        help="Interpolation frequency in Hz during playback.",
    )
    return parser.parse_args()


def load_trajectory(path: Path) -> dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(
            f"Trajectory file not found: {path}. Record one first or pass an explicit input path."
        )
    payload = json.loads(path.read_text())
    if payload.get("version") != 1:
        raise ValueError(f"Unsupported trajectory version: {payload.get('version')}")
    if payload.get("mode") != "joint_positions_deg":
        raise ValueError(f"Unsupported trajectory mode: {payload.get('mode')}")
    if not isinstance(payload.get("frames"), list) or not payload["frames"]:
        raise ValueError("Trajectory must contain a non-empty 'frames' list.")
    if not isinstance(payload.get("joint_names"), list) or not payload["joint_names"]:
        raise ValueError("Trajectory must contain a non-empty 'joint_names' list.")
    return payload


def validate_trajectory(arm, payload: dict[str, object], *, allow_zero_mismatch: bool) -> list[str]:
    joint_names = [str(joint_name) for joint_name in payload["joint_names"]]
    invalid = [joint_name for joint_name in joint_names if joint_name not in arm.config.joint_map]
    if invalid:
        raise ValueError(f"Trajectory contains joints missing from the active config: {invalid}")

    file_zero_map = {
        str(joint_name): int(raw_value)
        for joint_name, raw_value in dict(payload.get("zero_position_raw_by_joint", {})).items()
    }
    mismatches = {
        joint_name: {
            "file": file_zero_map[joint_name],
            "config": arm.config.require_joint(joint_name).zero_position_raw,
        }
        for joint_name in joint_names
        if joint_name in file_zero_map
        and file_zero_map[joint_name] != arm.config.require_joint(joint_name).zero_position_raw
    }
    if mismatches and not allow_zero_mismatch:
        raise ValueError(f"Trajectory zero references do not match the active config: {mismatches}")

    previous_t = None
    for index, frame in enumerate(payload["frames"]):
        if not isinstance(frame, dict):
            raise ValueError(f"Frame {index} must be a JSON object.")
        if "t" not in frame or "positions_deg" not in frame:
            raise ValueError(f"Frame {index} must contain 't' and 'positions_deg'.")
        frame_t = float(frame["t"])
        if frame_t < 0:
            raise ValueError(f"Frame {index} has a negative timestamp: {frame_t}")
        if previous_t is not None and frame_t < previous_t:
            raise ValueError(f"Frame timestamps must be non-decreasing. Frame {index} went backwards.")
        previous_t = frame_t

        positions_deg = dict(frame["positions_deg"])
        missing = [joint_name for joint_name in joint_names if joint_name not in positions_deg]
        if missing:
            raise ValueError(f"Frame {index} is missing joint targets for: {missing}")

    return joint_names


def final_positions(arm, joint_names: list[str]) -> dict[str, float]:
    positions_deg = arm.read_present_positions_deg()
    return {joint_name: float(positions_deg[joint_name]) for joint_name in joint_names}


def frame_positions(frame: dict[str, object], joint_names: list[str]) -> dict[str, float]:
    positions_deg = dict(frame["positions_deg"])
    return {joint_name: float(positions_deg[joint_name]) for joint_name in joint_names}


def normalize_frames(raw_frames: list[dict[str, object]], joint_names: list[str]) -> list[dict[str, object]]:
    first_t = float(raw_frames[0]["t"])
    return [
        {
            "t": float(frame["t"]) - first_t,
            "positions_deg": frame_positions(frame, joint_names),
        }
        for frame in raw_frames
    ]


def blend_positions(
    start_positions_deg: dict[str, float],
    end_positions_deg: dict[str, float],
    alpha: float,
) -> dict[str, float]:
    return {
        joint_name: (1.0 - alpha) * start_positions_deg[joint_name] + alpha * end_positions_deg[joint_name]
        for joint_name in start_positions_deg
    }


def interpolate_segment(
    start_t: float,
    start_positions_deg: dict[str, float],
    end_t: float,
    end_positions_deg: dict[str, float],
    playback_hz: float,
    *,
    include_start: bool,
) -> list[dict[str, object]]:
    if end_t < start_t:
        raise ValueError(f"Segment end_t {end_t} is smaller than start_t {start_t}.")
    if end_t == start_t:
        return [] if not include_start else [{"t": start_t, "positions_deg": dict(start_positions_deg)}]

    step_sec = 1.0 / playback_hz
    segment: list[dict[str, object]] = []
    sample_index = 0 if include_start else 1
    while True:
        sample_t = start_t + sample_index * step_sec
        if sample_t >= end_t:
            break
        alpha = (sample_t - start_t) / (end_t - start_t)
        segment.append(
            {
                "t": sample_t,
                "positions_deg": blend_positions(start_positions_deg, end_positions_deg, alpha),
            }
        )
        sample_index += 1

    segment.append({"t": end_t, "positions_deg": dict(end_positions_deg)})
    return segment


def build_playback_samples(
    current_positions_deg: dict[str, float],
    frames: list[dict[str, object]],
    playback_hz: float,
    approach_sec: float,
) -> list[dict[str, object]]:
    samples: list[dict[str, object]] = []
    first_positions_deg = dict(frames[0]["positions_deg"])
    if approach_sec > 0:
        samples.extend(
            interpolate_segment(
                0.0,
                current_positions_deg,
                approach_sec,
                first_positions_deg,
                playback_hz,
                include_start=True,
            )
        )
    else:
        samples.append({"t": 0.0, "positions_deg": first_positions_deg})

    offset_sec = samples[-1]["t"]
    for index, frame in enumerate(frames):
        frame_t = offset_sec + float(frame["t"])
        positions_deg = dict(frame["positions_deg"])
        if index == 0:
            if frame_t > samples[-1]["t"]:
                samples.extend(
                    interpolate_segment(
                        samples[-1]["t"],
                        dict(samples[-1]["positions_deg"]),
                        frame_t,
                        positions_deg,
                        playback_hz,
                        include_start=False,
                    )
                )
            else:
                samples[-1] = {"t": frame_t, "positions_deg": positions_deg}
            continue

        previous = frames[index - 1]
        previous_t = offset_sec + float(previous["t"])
        previous_positions_deg = dict(previous["positions_deg"])
        samples.extend(
            interpolate_segment(
                previous_t,
                previous_positions_deg,
                frame_t,
                positions_deg,
                playback_hz,
                include_start=False,
            )
        )

    return samples


def main() -> None:
    args = parse_args()
    if args.hold_sec < 0:
        raise ValueError("--hold-sec must be non-negative.")
    if args.approach_sec < 0:
        raise ValueError("--approach-sec must be non-negative.")
    if args.speed_deg_s is not None and args.speed_deg_s <= 0:
        raise ValueError("--speed-deg-s must be positive.")
    if args.playback_hz is not None and args.playback_hz <= 0:
        raise ValueError("--playback-hz must be positive.")

    payload = load_trajectory(args.input)

    with make_controller(args.config, args.port) as arm:
        joint_names = validate_trajectory(
            arm,
            payload,
            allow_zero_mismatch=args.allow_zero_mismatch,
        )
        frames = normalize_frames(payload["frames"], joint_names)
        playback_hz = float(args.playback_hz)
        current_positions_deg = final_positions(arm, joint_names)
        playback_samples = build_playback_samples(
            current_positions_deg,
            frames,
            playback_hz,
            args.approach_sec,
        )
        start_time = time.monotonic()
        previous_t = 0.0
        previous_positions_deg = dict(current_positions_deg)

        for frame in playback_samples:
            frame_t = float(frame["t"])
            sleep_sec = start_time + frame_t - time.monotonic()
            if sleep_sec > 0:
                time.sleep(sleep_sec)

            target_positions_deg = {
                joint_name: float(dict(frame["positions_deg"])[joint_name])
                for joint_name in joint_names
            }
            changed_positions_deg = {
                joint_name: target_deg
                for joint_name, target_deg in target_positions_deg.items()
                if abs(target_deg - previous_positions_deg[joint_name]) > POSITION_EPS_DEG
            }
            if changed_positions_deg:
                delta_t = frame_t - previous_t
                speed_override = args.speed_deg_s
                if speed_override is None and delta_t > 0:
                    speed_override = {
                        joint_name: abs(changed_positions_deg[joint_name] - previous_positions_deg[joint_name]) / delta_t
                        for joint_name in changed_positions_deg
                    }
                arm.move_joints(
                    changed_positions_deg,
                    speed_deg_s=speed_override,
                )

            previous_t = frame_t
            previous_positions_deg = target_positions_deg

        time.sleep(args.hold_sec)
        result = {
            "input": str(args.input),
            "frame_count": len(frames),
            "playback_sample_count": len(playback_samples),
            "approach_sec": float(args.approach_sec),
            "playback_hz": float(playback_hz),
            "speed_mode": "fixed" if args.speed_deg_s is not None else "auto",
            "fixed_speed_deg_s": None if args.speed_deg_s is None else float(args.speed_deg_s),
            "duration_sec": float(dict(playback_samples[-1])["t"]) if playback_samples else 0.0,
            "joint_names": joint_names,
            "final_positions_deg": final_positions(arm, joint_names),
        }

    print_json(result)


if __name__ == "__main__":
    main()
