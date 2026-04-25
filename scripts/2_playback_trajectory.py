from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

from _robot_script_common import add_common_arguments, make_controller, print_json


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_TRAJECTORY_PATH = REPO_ROOT / "recordings" / "latest_trajectory.json"
POSITION_EPS_DEG = 1e-6
SUPPORTED_OVERRIDE_INTERPOLATIONS = {"linear", "smoothstep"}
SUPPORTED_OVERRIDE_TIMING_POLICIES = {"match_base_trajectory"}


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
    parser.add_argument(
        "--interpolation-override",
        type=Path,
        default=None,
        help="Optional joint_interpolation_override JSON. Its joints overwrite or extend the base trajectory.",
    )
    parser.add_argument(
        "--max-override-speed-deg-s",
        type=float,
        default=None,
        help="Optional max instantaneous speed allowed for override joints after interpolation.",
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


def load_interpolation_override(path: Path) -> dict[str, object]:
    if not path.exists():
        raise FileNotFoundError(f"Interpolation override file not found: {path}")
    payload = json.loads(path.read_text())
    if payload.get("version") != 1:
        raise ValueError(f"Unsupported interpolation override version: {payload.get('version')}")
    if payload.get("mode") != "joint_interpolation_override":
        raise ValueError(f"Unsupported interpolation override mode: {payload.get('mode')}")
    if not isinstance(payload.get("joint_names"), list) or not payload["joint_names"]:
        raise ValueError("Interpolation override must contain a non-empty 'joint_names' list.")
    return payload


def validate_trajectory(arm, payload: dict[str, object], *, allow_zero_mismatch: bool) -> list[str]:
    joint_names = [str(joint_name) for joint_name in payload["joint_names"]]
    invalid = [joint_name for joint_name in joint_names if joint_name not in arm.config.joint_map]
    if invalid:
        raise ValueError(f"Trajectory contains joints missing from the active config: {invalid}")

    offline = [joint_name for joint_name in joint_names if joint_name not in arm.online_joint_names]
    if offline:
        raise ConnectionError(
            f"Trajectory contains joints not available on the bus: {offline}. "
            f"Online joints: {list(arm.online_joint_names)}"
        )

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


def override_interpolation_type(payload: dict[str, object]) -> str:
    interpolation = payload.get("interpolation", {})
    if isinstance(interpolation, dict):
        interpolation_type = str(interpolation.get("type", "smoothstep"))
    else:
        interpolation_type = str(interpolation)
    if interpolation_type not in SUPPORTED_OVERRIDE_INTERPOLATIONS:
        raise ValueError(
            f"Unsupported interpolation override type: {interpolation_type}. "
            f"Supported: {sorted(SUPPORTED_OVERRIDE_INTERPOLATIONS)}"
        )
    return interpolation_type


def override_timing_policy(payload: dict[str, object]) -> str:
    timing = payload.get("timing", {})
    if not isinstance(timing, dict):
        raise ValueError("Interpolation override 'timing' must be a JSON object.")
    policy = str(timing.get("policy", "match_base_trajectory"))
    if policy not in SUPPORTED_OVERRIDE_TIMING_POLICIES:
        raise ValueError(
            f"Unsupported interpolation override timing policy: {policy}. "
            f"Supported: {sorted(SUPPORTED_OVERRIDE_TIMING_POLICIES)}"
        )
    return policy


def override_waypoints(
    payload: dict[str, object],
    joint_names: list[str],
) -> list[dict[str, object]]:
    raw_waypoints = payload.get("waypoints")
    if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
        raise ValueError("Interpolation override must contain at least two waypoints.")

    waypoints: list[dict[str, object]] = []
    previous_phase = -1.0
    for index, raw_waypoint in enumerate(raw_waypoints):
        if not isinstance(raw_waypoint, dict):
            raise ValueError(f"Override waypoint {index} must be a JSON object.")
        if "phase" not in raw_waypoint or "positions_deg" not in raw_waypoint:
            raise ValueError(f"Override waypoint {index} must contain 'phase' and 'positions_deg'.")

        phase = float(raw_waypoint["phase"])
        if phase < 0.0 or phase > 1.0:
            raise ValueError(f"Override waypoint {index} phase must be within [0, 1], got {phase}.")
        if phase < previous_phase:
            raise ValueError(f"Override waypoint phases must be non-decreasing. Waypoint {index} went backwards.")
        previous_phase = phase

        positions_deg = dict(raw_waypoint["positions_deg"])
        missing = [joint_name for joint_name in joint_names if joint_name not in positions_deg]
        if missing:
            raise ValueError(f"Override waypoint {index} is missing joint positions for: {missing}")
        waypoints.append(
            {
                "phase": phase,
                "positions_deg": {
                    joint_name: float(positions_deg[joint_name])
                    for joint_name in joint_names
                },
            }
        )

    if abs(float(waypoints[0]["phase"]) - 0.0) > 1e-9:
        raise ValueError("Interpolation override first waypoint must have phase=0.0.")
    if abs(float(waypoints[-1]["phase"]) - 1.0) > 1e-9:
        raise ValueError("Interpolation override last waypoint must have phase=1.0.")
    return waypoints


def validate_interpolation_override(
    arm,
    payload: dict[str, object],
    *,
    allow_zero_mismatch: bool,
) -> list[str]:
    joint_names = [str(joint_name) for joint_name in payload["joint_names"]]
    duplicates = sorted({joint_name for joint_name in joint_names if joint_names.count(joint_name) > 1})
    if duplicates:
        raise ValueError(f"Interpolation override contains duplicate joints: {duplicates}")

    invalid = [joint_name for joint_name in joint_names if joint_name not in arm.config.joint_map]
    if invalid:
        raise ValueError(f"Interpolation override contains joints missing from the active config: {invalid}")

    offline = [joint_name for joint_name in joint_names if joint_name not in arm.online_joint_names]
    if offline:
        raise ConnectionError(
            f"Interpolation override contains joints not available on the bus: {offline}. "
            f"Online joints: {list(arm.online_joint_names)}"
        )

    override_interpolation_type(payload)
    override_timing_policy(payload)
    override_waypoints(payload, joint_names)

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
        raise ValueError(f"Interpolation override zero references do not match the active config: {mismatches}")

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


def interpolation_alpha(interpolation_type: str, phase: float) -> float:
    phase = max(0.0, min(1.0, float(phase)))
    if interpolation_type == "linear":
        return phase
    if interpolation_type == "smoothstep":
        return phase * phase * (3.0 - 2.0 * phase)
    raise ValueError(f"Unsupported interpolation type: {interpolation_type}")


def interpolate_override_positions(
    waypoints: list[dict[str, object]],
    joint_names: list[str],
    phase: float,
    interpolation_type: str,
) -> dict[str, float]:
    phase = max(0.0, min(1.0, float(phase)))
    first_positions = dict(waypoints[0]["positions_deg"])
    if phase <= float(waypoints[0]["phase"]):
        return {joint_name: float(first_positions[joint_name]) for joint_name in joint_names}

    last_positions = dict(waypoints[-1]["positions_deg"])
    if phase >= float(waypoints[-1]["phase"]):
        return {joint_name: float(last_positions[joint_name]) for joint_name in joint_names}

    for index in range(1, len(waypoints)):
        start_waypoint = waypoints[index - 1]
        end_waypoint = waypoints[index]
        start_phase = float(start_waypoint["phase"])
        end_phase = float(end_waypoint["phase"])
        if phase > end_phase:
            continue

        phase_span = end_phase - start_phase
        if phase_span <= 0:
            raise ValueError(f"Override waypoints {index - 1} and {index} have the same phase.")

        local_phase = (phase - start_phase) / phase_span
        alpha = interpolation_alpha(interpolation_type, local_phase)
        start_positions = dict(start_waypoint["positions_deg"])
        end_positions = dict(end_waypoint["positions_deg"])
        return {
            joint_name: (1.0 - alpha) * float(start_positions[joint_name]) + alpha * float(end_positions[joint_name])
            for joint_name in joint_names
        }

    return {joint_name: float(last_positions[joint_name]) for joint_name in joint_names}


def required_override_speeds(
    waypoints: list[dict[str, object]],
    joint_names: list[str],
    base_duration_sec: float,
    interpolation_type: str,
) -> dict[str, float]:
    required_speeds = {joint_name: 0.0 for joint_name in joint_names}
    speed_factor = 1.5 if interpolation_type == "smoothstep" else 1.0

    for index in range(1, len(waypoints)):
        start_waypoint = waypoints[index - 1]
        end_waypoint = waypoints[index]
        start_phase = float(start_waypoint["phase"])
        end_phase = float(end_waypoint["phase"])
        phase_duration = end_phase - start_phase
        start_positions = dict(start_waypoint["positions_deg"])
        end_positions = dict(end_waypoint["positions_deg"])

        for joint_name in joint_names:
            delta_deg = abs(float(end_positions[joint_name]) - float(start_positions[joint_name]))
            if delta_deg <= POSITION_EPS_DEG:
                continue
            segment_duration = base_duration_sec * phase_duration
            if segment_duration <= 0:
                raise ValueError(
                    f"Override joint {joint_name} moves {delta_deg:.6f} deg but the base trajectory duration is zero."
                )
            speed_deg_s = speed_factor * delta_deg / segment_duration
            required_speeds[joint_name] = max(required_speeds[joint_name], speed_deg_s)

    return required_speeds


def apply_interpolation_override(
    frames: list[dict[str, object]],
    joint_names: list[str],
    payload: dict[str, object],
    *,
    max_override_speed_deg_s: float | None,
) -> tuple[list[dict[str, object]], list[str], dict[str, object]]:
    override_joint_names = [str(joint_name) for joint_name in payload["joint_names"]]
    interpolation_type = override_interpolation_type(payload)
    timing_policy = override_timing_policy(payload)
    waypoints = override_waypoints(payload, override_joint_names)
    base_duration_sec = float(frames[-1]["t"]) if frames else 0.0
    required_speeds = required_override_speeds(
        waypoints,
        override_joint_names,
        base_duration_sec,
        interpolation_type,
    )

    if max_override_speed_deg_s is not None:
        too_fast = {
            joint_name: speed_deg_s
            for joint_name, speed_deg_s in required_speeds.items()
            if speed_deg_s > max_override_speed_deg_s + POSITION_EPS_DEG
        }
        if too_fast:
            raise ValueError(
                f"Interpolation override exceeds --max-override-speed-deg-s={max_override_speed_deg_s}: "
                f"{too_fast}"
            )

    merged_joint_names = list(joint_names)
    for joint_name in override_joint_names:
        if joint_name not in merged_joint_names:
            merged_joint_names.append(joint_name)

    merged_frames: list[dict[str, object]] = []
    for frame in frames:
        frame_t = float(frame["t"])
        phase = 1.0 if base_duration_sec <= 0 else frame_t / base_duration_sec
        override_positions = interpolate_override_positions(
            waypoints,
            override_joint_names,
            phase,
            interpolation_type,
        )
        positions_deg = dict(frame["positions_deg"])
        positions_deg.update(override_positions)
        merged_frames.append(
            {
                "t": frame_t,
                "positions_deg": positions_deg,
            }
        )

    summary = {
        "joint_names": override_joint_names,
        "interpolation": interpolation_type,
        "timing_policy": timing_policy,
        "base_duration_sec": base_duration_sec,
        "required_speed_deg_s": required_speeds,
        "max_override_speed_deg_s": max_override_speed_deg_s,
    }
    return merged_frames, merged_joint_names, summary


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
    if args.max_override_speed_deg_s is not None and args.max_override_speed_deg_s <= 0:
        raise ValueError("--max-override-speed-deg-s must be positive.")

    payload = load_trajectory(args.input)
    interpolation_override = (
        None
        if args.interpolation_override is None
        else load_interpolation_override(args.interpolation_override)
    )

    with make_controller(args.config, args.port) as arm:
        joint_names = validate_trajectory(
            arm,
            payload,
            allow_zero_mismatch=args.allow_zero_mismatch,
        )
        frames = normalize_frames(payload["frames"], joint_names)
        override_summary = None
        if interpolation_override is not None:
            validate_interpolation_override(
                arm,
                interpolation_override,
                allow_zero_mismatch=args.allow_zero_mismatch,
            )
            frames, joint_names, override_summary = apply_interpolation_override(
                frames,
                joint_names,
                interpolation_override,
                max_override_speed_deg_s=args.max_override_speed_deg_s,
            )
            override_summary["path"] = str(args.interpolation_override)
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
            "interpolation_override": override_summary,
            "final_positions_deg": final_positions(arm, joint_names),
        }

    print_json(result)


if __name__ == "__main__":
    main()
