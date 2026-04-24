from __future__ import annotations

import argparse
import time

from _robot_script_common import add_common_arguments, make_controller, print_json


def read_joint_snapshot(arm, joint_name: str) -> dict[str, float | int]:
    assert arm.bus is not None

    present_raw = int(arm.bus.read("Present_Position", joint_name, normalize=False))
    min_limit_raw = int(arm.bus.read("Min_Position_Limit", joint_name, normalize=False))
    max_limit_raw = int(arm.bus.read("Max_Position_Limit", joint_name, normalize=False))
    moving = int(arm.bus.read("Moving", joint_name, normalize=False))
    status = int(arm.bus.read("Status", joint_name, normalize=False))

    return {
        "present_deg": arm.joint_raw_to_position_deg(joint_name, present_raw),
        "present_raw": present_raw,
        "tracked_target_deg": arm.target_positions_deg[joint_name],
        "zero_position_raw": arm.config.require_joint(joint_name).zero_position_raw,
        "min_limit_raw": min_limit_raw,
        "max_limit_raw": max_limit_raw,
        "moving": moving,
        "status": status,
    }


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Same-process mode0 smoke test for one joint using an absolute target."
    )
    add_common_arguments(parser)
    parser.add_argument("--joint", required=True, help="Joint name, for example joint_2.")
    parser.add_argument("--target-deg", type=float, required=True, help="Absolute target angle in degrees.")
    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=None,
        help="Requested joint speed in deg/s. Defaults to the speed configured in the JSON config.",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=1.5,
        help="Seconds to wait after each move before reading back the result.",
    )
    parser.add_argument(
        "--no-return-start",
        action="store_true",
        help="Do not return the joint to the starting angle within the same process.",
    )
    args = parser.parse_args()

    with make_controller(args.config, args.port) as arm:
        before = read_joint_snapshot(arm, args.joint)
        target_raw = arm.joint_position_deg_to_raw(args.joint, args.target_deg)

        if target_raw < before["min_limit_raw"] or target_raw > before["max_limit_raw"]:
            raise ValueError(
                f"{args.joint}: target {args.target_deg} deg maps to raw {target_raw}, "
                f"outside motor limits [{before['min_limit_raw']}, {before['max_limit_raw']}]."
            )

        result = {
            "joint": args.joint,
            "target_deg": float(args.target_deg),
            "target_raw": target_raw,
            "before": before,
            "move_goal_raw": arm.move_joint(
                args.joint,
                args.target_deg,
                speed_deg_s=args.speed_deg_s,
            ),
        }

        time.sleep(max(args.hold_sec, 0.0))
        result["after_move"] = read_joint_snapshot(arm, args.joint)

        if not args.no_return_start:
            start_deg = float(before["present_deg"])
            result["return_start_deg"] = start_deg
            result["return_goal_raw"] = arm.move_joint(
                args.joint,
                start_deg,
                speed_deg_s=args.speed_deg_s,
            )
            time.sleep(max(args.hold_sec, 0.0))
            result["after_return"] = read_joint_snapshot(arm, args.joint)

        print_json(result)


if __name__ == "__main__":
    main()
