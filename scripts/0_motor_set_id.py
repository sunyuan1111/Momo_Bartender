from __future__ import annotations

import argparse
import json

from _robot_script_common import DEFAULT_PORT
from physical_agent.lerobot_compat import require_lerobot


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read or change the ID register of a single STS3215 motor. Keep only the target motor on the bus while renaming."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port.")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Motor bus baudrate.")
    parser.add_argument("--protocol-version", type=int, default=0, help="Feetech protocol version.")
    parser.add_argument("--model", default="sts3215", help="Motor model name.")
    parser.add_argument("--current-id", type=int, default=None, help="Current motor ID.")
    parser.add_argument("--new-id", type=int, default=None, help="Write a new ID to the motor.")
    parser.add_argument(
        "--scan-port",
        action="store_true",
        help="Scan supported baudrates before doing the requested operation.",
    )
    return parser.parse_args()


def validate_motor_id(value: int | None, *, label: str) -> None:
    if value is None:
        return
    if not 0 <= value <= 253:
        raise ValueError(f"{label} must be in [0, 253], got {value}.")


def make_bus(*, port: str, baudrate: int, protocol_version: int, model: str, motor_id: int):
    FeetechMotorsBus, Motor, MotorNormMode = require_lerobot()
    motors = {
        "target": Motor(
            id=motor_id,
            model=model,
            norm_mode=MotorNormMode.DEGREES,
        )
    }
    bus = FeetechMotorsBus(
        port=port,
        motors=motors,
        protocol_version=protocol_version,
    )
    bus.default_baudrate = baudrate
    bus.connect(handshake=False)
    bus.set_baudrate(baudrate)
    return bus


def scan_current_baudrate(*, port: str, baudrate: int, protocol_version: int) -> dict[int, int] | None:
    FeetechMotorsBus, _, _ = require_lerobot()
    if protocol_version != 0:
        return None

    bus = FeetechMotorsBus(port=port, motors={}, protocol_version=protocol_version)
    bus.connect(handshake=False)
    try:
        bus.set_baudrate(baudrate)
        return bus.broadcast_ping(raise_on_error=False)
    finally:
        bus.disconnect(disable_torque=False)


def read_motor_info(*, port: str, baudrate: int, protocol_version: int, model: str, motor_id: int) -> dict[str, int]:
    bus = make_bus(
        port=port,
        baudrate=baudrate,
        protocol_version=protocol_version,
        model=model,
        motor_id=motor_id,
    )
    try:
        model_number = bus.ping("target", raise_on_error=False)
        if model_number is None:
            raise ConnectionError(f"Motor ID {motor_id} did not respond on {port} at {baudrate} baud.")
        id_register = int(bus.read("ID", "target", normalize=False))
        return {
            "motor_id": motor_id,
            "id_register": id_register,
            "model_number": int(model_number),
        }
    finally:
        bus.disconnect(disable_torque=False)


def rename_motor(
    *,
    port: str,
    baudrate: int,
    protocol_version: int,
    model: str,
    current_id: int,
    new_id: int,
) -> dict[str, dict[str, int]]:
    before = read_motor_info(
        port=port,
        baudrate=baudrate,
        protocol_version=protocol_version,
        model=model,
        motor_id=current_id,
    )

    rename_bus = make_bus(
        port=port,
        baudrate=baudrate,
        protocol_version=protocol_version,
        model=model,
        motor_id=current_id,
    )
    try:
        rename_bus.disable_torque("target")
        rename_bus.write("ID", "target", new_id, normalize=False)
    finally:
        rename_bus.disconnect(disable_torque=False)

    verify_bus = make_bus(
        port=port,
        baudrate=baudrate,
        protocol_version=protocol_version,
        model=model,
        motor_id=new_id,
    )
    try:
        model_number = verify_bus.ping("target", raise_on_error=False)
        if model_number is None:
            raise ConnectionError(f"Motor did not respond after renaming {current_id} -> {new_id}.")
        after = {
            "motor_id": new_id,
            "id_register": int(verify_bus.read("ID", "target", normalize=False)),
            "model_number": int(model_number),
        }
        verify_bus.write("Lock", "target", 1, normalize=False)
    finally:
        verify_bus.disconnect(disable_torque=False)

    return {
        "before": before,
        "after": after,
    }


def main() -> None:
    args = parse_args()
    validate_motor_id(args.current_id, label="current_id")
    validate_motor_id(args.new_id, label="new_id")

    result: dict[str, object] = {
        "port": args.port,
        "baudrate": args.baudrate,
        "protocol_version": args.protocol_version,
        "model": args.model,
    }

    if args.scan_port:
        FeetechMotorsBus, _, _ = require_lerobot()
        result["scan_port"] = FeetechMotorsBus.scan_port(args.port, protocol_version=args.protocol_version)

    current_baud_scan = scan_current_baudrate(
        port=args.port,
        baudrate=args.baudrate,
        protocol_version=args.protocol_version,
    )
    if current_baud_scan is not None:
        result["broadcast_ping"] = current_baud_scan

    current_id = args.current_id
    if current_id is None and current_baud_scan and len(current_baud_scan) == 1:
        current_id = next(iter(current_baud_scan))

    if args.new_id is None:
        if current_id is not None:
            result["motor"] = read_motor_info(
                port=args.port,
                baudrate=args.baudrate,
                protocol_version=args.protocol_version,
                model=args.model,
                motor_id=current_id,
            )
        print(json.dumps(result, indent=2, sort_keys=True))
        return

    if current_id is None:
        raise ValueError(
            "Changing ID requires --current-id, or exactly one responding motor on the bus at the selected baudrate."
        )

    if current_id == args.new_id:
        raise ValueError("current_id and new_id are the same; nothing to do.")

    result["rename"] = rename_motor(
        port=args.port,
        baudrate=args.baudrate,
        protocol_version=args.protocol_version,
        model=args.model,
        current_id=current_id,
        new_id=args.new_id,
    )
    print(json.dumps(result, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
