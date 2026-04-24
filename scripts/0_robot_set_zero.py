from __future__ import annotations

import argparse

from _robot_script_common import add_common_arguments, make_controller, print_json


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Treat the current power-on pose as software zero for this process and print the captured state."
    )
    add_common_arguments(parser)
    args = parser.parse_args()

    with make_controller(args.config, args.port) as arm:
        state = arm.read_state()
        print_json(
            {
                "port": state["port"],
                "startup_raw_positions": state["startup_raw_positions"],
                "target_positions_deg": state["target_positions_deg"],
            }
        )


if __name__ == "__main__":
    main()
