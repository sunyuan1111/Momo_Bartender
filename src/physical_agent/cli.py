from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import ArmConfig
from .controller import Sts3215ArmController


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="STS3215 arm control helper.")
    parser.add_argument(
        "--config",
        default="configs/arm7_sts3215.example.json",
        help="Path to the JSON arm config file.",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("init", help="Connect and configure every servo for step mode.")
    subparsers.add_parser("state", help="Read current raw state and tracked target state.")
    subparsers.add_parser("cartesian-state", help="Read the current end-effector xyz estimated from URDF kinematics.")

    home_parser = subparsers.add_parser("home", help="Move arm joints back to the startup zero pose.")
    home_parser.add_argument("--speed-deg-s", type=float, default=None)

    move_joints_parser = subparsers.add_parser("move-joints", help="Move one or more joints to target degrees.")
    move_joints_parser.add_argument(
        "--position",
        action="append",
        required=True,
        help="Joint target in the form joint_name=degrees. Repeat for multiple joints.",
    )
    move_joints_parser.add_argument("--speed-deg-s", type=float, default=None)

    nudge_joint_parser = subparsers.add_parser("nudge-joint", help="Move a single joint by a delta in degrees.")
    nudge_joint_parser.add_argument("--joint", required=True)
    nudge_joint_parser.add_argument("--delta", type=float, required=True)
    nudge_joint_parser.add_argument("--speed-deg-s", type=float, default=None)

    move_gripper_parser = subparsers.add_parser("move-gripper", help="Move gripper to a target degree value.")
    move_gripper_parser.add_argument("--position", type=float, required=True)
    move_gripper_parser.add_argument("--speed-deg-s", type=float, default=None)

    nudge_gripper_parser = subparsers.add_parser("nudge-gripper", help="Move gripper by a delta in degrees.")
    nudge_gripper_parser.add_argument("--delta", type=float, required=True)
    nudge_gripper_parser.add_argument("--speed-deg-s", type=float, default=None)

    solve_cartesian_parser = subparsers.add_parser("solve-cartesian", help="Solve xyz to arm joint angles without moving.")
    solve_cartesian_parser.add_argument("--x", type=float, required=True)
    solve_cartesian_parser.add_argument("--y", type=float, required=True)
    solve_cartesian_parser.add_argument("--z", type=float, required=True)

    move_cartesian_parser = subparsers.add_parser("move-cartesian", help="Move the arm to a target xyz position.")
    move_cartesian_parser.add_argument("--x", type=float, required=True)
    move_cartesian_parser.add_argument("--y", type=float, required=True)
    move_cartesian_parser.add_argument("--z", type=float, required=True)
    move_cartesian_parser.add_argument("--speed-deg-s", type=float, default=None)

    nudge_cartesian_parser = subparsers.add_parser("nudge-cartesian", help="Move the arm by a delta xyz.")
    nudge_cartesian_parser.add_argument("--dx", type=float, required=True)
    nudge_cartesian_parser.add_argument("--dy", type=float, required=True)
    nudge_cartesian_parser.add_argument("--dz", type=float, required=True)
    nudge_cartesian_parser.add_argument("--speed-deg-s", type=float, default=None)

    write_default_config_parser = subparsers.add_parser(
        "write-default-config",
        help="Write the built-in default config to a JSON file.",
    )
    write_default_config_parser.add_argument("output", type=Path)

    return parser


def parse_positions(items: list[str]) -> dict[str, float]:
    positions: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"Expected joint target as joint_name=degrees, got '{item}'.")
        name, value = item.split("=", 1)
        positions[name] = float(value)
    return positions


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.command == "write-default-config":
        ArmConfig.default().save_json(args.output)
        print(args.output)
        return

    controller = Sts3215ArmController(ArmConfig.from_json(args.config))
    with controller:
        if args.command == "init":
            print(json.dumps(controller.read_state(), indent=2, sort_keys=True))
            return

        if args.command == "state":
            print(json.dumps(controller.read_state(), indent=2, sort_keys=True))
            return

        if args.command == "cartesian-state":
            print(json.dumps(controller.forward_kinematics(), indent=2, sort_keys=True))
            return

        if args.command == "home":
            result = controller.home(speed_deg_s=args.speed_deg_s)
            print(json.dumps(result, indent=2, sort_keys=True))
            return

        if args.command == "move-joints":
            result = controller.move_joints(
                parse_positions(args.position),
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps(result, indent=2, sort_keys=True))
            return

        if args.command == "nudge-joint":
            result = controller.nudge_joint(
                args.joint,
                args.delta,
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps({args.joint: result}, indent=2, sort_keys=True))
            return

        if args.command == "move-gripper":
            result = controller.move_gripper(
                args.position,
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps({"gripper": result}, indent=2, sort_keys=True))
            return

        if args.command == "nudge-gripper":
            result = controller.nudge_gripper(
                args.delta,
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps({"gripper": result}, indent=2, sort_keys=True))
            return

        if args.command == "solve-cartesian":
            result = controller.solve_cartesian(args.x, args.y, args.z)
            print(json.dumps(result, indent=2, sort_keys=True))
            return

        if args.command == "move-cartesian":
            result = controller.move_cartesian(
                args.x,
                args.y,
                args.z,
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps(result, indent=2, sort_keys=True))
            return

        if args.command == "nudge-cartesian":
            result = controller.nudge_cartesian(
                args.dx,
                args.dy,
                args.dz,
                speed_deg_s=args.speed_deg_s,
            )
            print(json.dumps(result, indent=2, sort_keys=True))
            return

    raise RuntimeError(f"Unhandled command: {args.command}")


if __name__ == "__main__":
    main()
