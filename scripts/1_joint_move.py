from __future__ import annotations

import argparse
import time

from _robot_script_common import add_common_arguments, make_controller, print_json


def parse_positions(items: list[str]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Expected joint target as joint_name=degrees, got '{item}'.")
        joint_name, value = item.split("=", 1)
        positions[joint_name] = float(value)
    return positions


def main() -> None:
    parser = argparse.ArgumentParser(description="Move one or more joints in one process.")
    add_common_arguments(parser)
    parser.add_argument(
        "--position",
        action="append",
        required=True,
        help="Joint target in the form joint_name=degrees. Repeat for multiple joints.",
    )
    parser.add_argument(
        "--return-zero",
        action="store_true",
        help="After the test move, return the same joints to 0 degrees within the same process.",
    )
    parser.add_argument(
        "--speed-deg-s",
        type=float,
        default=None,
        help="Requested joint speed in deg/s. Defaults to the speed configured in the JSON config.",
    )
    parser.add_argument(
        "--hold-sec",
        type=float,
        default=1.0,
        help="Seconds to wait after the move before exiting or returning to zero.",
    )
    args = parser.parse_args()

    target_positions = parse_positions(args.position)
    with make_controller(args.config, args.port) as arm:
        result = {
            "move": arm.move_joints(
                target_positions,
                speed_deg_s=args.speed_deg_s,
            )
        }
        time.sleep(max(args.hold_sec, 0.0))
        if args.return_zero:
            result["return_zero"] = arm.move_joints(
                {joint_name: 0.0 for joint_name in target_positions},
                speed_deg_s=args.speed_deg_s,
            )
        print_json(result)


if __name__ == "__main__":
    main()
