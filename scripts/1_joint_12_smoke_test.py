from __future__ import annotations

import argparse
import time

from _robot_script_common import add_common_arguments, make_controller, print_json


def main() -> None:
    parser = argparse.ArgumentParser(description="Small motion smoke test for joint_1 and joint_2.")
    add_common_arguments(parser)
    parser.add_argument("--joint-1", type=float, default=0.0, help="Target angle for joint_1 in degrees.")
    parser.add_argument("--joint-2", type=float, default=0.0, help="Target angle for joint_2 in degrees.")
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
        help="Seconds to wait after the move before exiting or returning home.",
    )
    parser.add_argument(
        "--no-return-home",
        action="store_true",
        help="Do not return joint_1 and joint_2 back to the configured zero pose.",
    )
    args = parser.parse_args()

    target_positions = {
        "joint_1": args.joint_1,
        "joint_2": args.joint_2,
    }

    with make_controller(args.config, args.port) as arm:
        result = {
            "move": arm.move_joints(
                target_positions,
                speed_deg_s=args.speed_deg_s,
            )
        }
        time.sleep(max(args.hold_sec, 0.0))
        if not args.no_return_home:
            result["return_home"] = arm.move_joints(
                {"joint_1": 0.0, "joint_2": 0.0},
                speed_deg_s=args.speed_deg_s,
            )
        print_json(result)


if __name__ == "__main__":
    main()
