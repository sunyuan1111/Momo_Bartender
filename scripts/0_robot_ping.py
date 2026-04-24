from __future__ import annotations

import argparse
import json

from _robot_script_common import DEFAULT_PORT, ArmConfig
from physical_agent.lerobot_compat import require_lerobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Ping configured Feetech motors or scan a port.")
    parser.add_argument("--config", default="configs/joint12_test.json", help="Arm config JSON path.")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port.")
    parser.add_argument("--baudrate", type=int, default=None, help="Override baudrate.")
    parser.add_argument("--ids", type=int, nargs="*", default=None, help="Motor IDs to ping.")
    parser.add_argument(
        "--scan-port",
        action="store_true",
        help="Scan the port across LeRobot's supported baudrates before pinging.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    config = ArmConfig.from_json(args.config)
    baudrate = int(args.baudrate or config.baudrate)
    ids = args.ids or [joint.motor_id for joint in config.joints]

    FeetechMotorsBus, Motor, MotorNormMode = require_lerobot()

    result: dict[str, object] = {
        "port": args.port,
        "baudrate": baudrate,
        "ids": ids,
    }

    if args.scan_port:
        result["scan_port"] = FeetechMotorsBus.scan_port(args.port)

    motors = {
        f"id_{motor_id}": Motor(id=motor_id, model="sts3215", norm_mode=MotorNormMode.DEGREES)
        for motor_id in ids
    }
    bus = FeetechMotorsBus(port=args.port, motors=motors, protocol_version=config.protocol_version)
    bus.connect(handshake=False)
    try:
        bus.set_baudrate(baudrate)
        result["ping"] = {
            str(motor_id): bus.ping(f"id_{motor_id}", raise_on_error=False)
            for motor_id in ids
        }
        result["broadcast_ping"] = bus.broadcast_ping(raise_on_error=False)
    finally:
        bus.disconnect(disable_torque=False)

    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
