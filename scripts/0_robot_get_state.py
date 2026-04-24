from __future__ import annotations

import argparse

from _robot_script_common import add_common_arguments, make_controller, print_json


def main() -> None:
    parser = argparse.ArgumentParser(description="Read current robot state from the Feetech bus.")
    add_common_arguments(parser)
    args = parser.parse_args()

    with make_controller(args.config, args.port) as arm:
        print_json(arm.read_state())


if __name__ == "__main__":
    main()
