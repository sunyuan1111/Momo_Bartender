from __future__ import annotations

import argparse
import json
from dataclasses import dataclass

from _robot_script_common import DEFAULT_PORT
from physical_agent.lerobot_compat import require_lerobot


@dataclass(frozen=True)
class RegisterSpec:
    key: str
    label_cn: str
    address: int
    length: int
    group: str
    unit: str = ""
    sign_bit: int | None = None


DOCUMENTED_REGISTERS: tuple[RegisterSpec, ...] = (
    RegisterSpec("Firmware_Major_Version", "固件主版本号", 0, 1, "version"),
    RegisterSpec("Firmware_Minor_Version", "固件次版本号", 1, 1, "version"),
    RegisterSpec("Endianness_Flag", "END", 2, 1, "version"),
    RegisterSpec("Servo_Major_Version", "舵机主版本号", 3, 1, "version"),
    RegisterSpec("Servo_Minor_Version", "舵机次版本号", 4, 1, "version"),
    RegisterSpec("ID", "舵机ID", 5, 1, "eprom", "id"),
    RegisterSpec("Baud_Rate", "波特率", 6, 1, "eprom"),
    RegisterSpec("Reserved_07", "预留地址", 7, 1, "eprom"),
    RegisterSpec("Response_Status_Level", "应答状态级别", 8, 1, "eprom"),
    RegisterSpec("Min_Position_Limit", "最小角度限制", 9, 2, "eprom"),
    RegisterSpec("Max_Position_Limit", "最大角度限制", 11, 2, "eprom"),
    RegisterSpec("Max_Temperature_Limit", "最高温度上限", 13, 1, "eprom", "C"),
    RegisterSpec("Max_Voltage_Limit", "最高输入电压", 14, 1, "eprom", "0.1V"),
    RegisterSpec("Min_Voltage_Limit", "最低输入电压", 15, 1, "eprom", "0.1V"),
    RegisterSpec("Max_Torque_Limit", "最大扭矩", 16, 2, "eprom", "0.1%"),
    RegisterSpec("Phase", "相位", 18, 1, "eprom"),
    RegisterSpec("Unloading_Condition", "卸载条件", 19, 1, "eprom"),
    RegisterSpec("LED_Alarm_Condition", "LED报警条件", 20, 1, "eprom"),
    RegisterSpec("Position_P_Coefficient", "位置环P比例系数", 21, 1, "eprom"),
    RegisterSpec("Position_D_Coefficient", "位置环D微分系数", 22, 1, "eprom"),
    RegisterSpec("Position_I_Coefficient", "位置环I积分系数", 23, 1, "eprom"),
    RegisterSpec("Minimum_Startup_Force", "最小启动力", 24, 1, "eprom", "0.1%"),
    RegisterSpec("Integral_Limit", "积分限制值", 25, 1, "eprom"),
    RegisterSpec("CW_Dead_Zone", "正向不灵敏区", 26, 1, "eprom", "0.087deg"),
    RegisterSpec("CCW_Dead_Zone", "负向不灵敏区", 27, 1, "eprom", "0.087deg"),
    RegisterSpec("Protection_Current", "保护电流", 28, 2, "eprom", "6.5mA"),
    RegisterSpec("Angular_Resolution", "角度分辨率", 30, 1, "eprom"),
    RegisterSpec("Position_Offset", "位置偏移", 31, 2, "eprom"),
    RegisterSpec("Operating_Mode", "运行模式", 33, 1, "eprom"),
    RegisterSpec("Protective_Torque", "保持扭矩", 34, 1, "eprom", "1%"),
    RegisterSpec("Protection_Time", "保护时间", 35, 1, "eprom", "10ms"),
    RegisterSpec("Overload_Torque", "过载扭矩", 36, 1, "eprom", "1%"),
    RegisterSpec("Velocity_Loop_P_Coefficient", "速度闭环P比例系数", 37, 1, "eprom"),
    RegisterSpec("Over_Current_Protection_Time", "过流保护时间", 38, 1, "eprom", "10ms"),
    RegisterSpec("Velocity_Loop_I_Coefficient", "速度闭环I积分系数", 39, 1, "eprom"),
    RegisterSpec("Torque_Enable", "扭矩开关", 40, 1, "sram_control"),
    RegisterSpec("Acceleration", "加速度", 41, 1, "sram_control", "8.7deg/s^2"),
    RegisterSpec("Goal_Position", "目标位置", 42, 2, "sram_control", "0.087deg", sign_bit=15),
    RegisterSpec("PWM_Open_Loop_Speed", "PWM开环速度", 44, 2, "sram_control", "0.1%", sign_bit=10),
    RegisterSpec("Goal_Velocity", "运行速度", 46, 2, "sram_control", "0.732RPM/0.0146RPM", sign_bit=15),
    RegisterSpec("Torque_Limit", "转矩限制", 48, 2, "sram_control", "0.1%"),
    RegisterSpec("Lock", "锁标志", 55, 1, "sram_control"),
    RegisterSpec("Present_Position", "当前位置", 56, 2, "sram_feedback", "0.087deg", sign_bit=15),
    RegisterSpec("Present_Velocity", "当前速度", 58, 2, "sram_feedback", "0.732RPM/0.0146RPM", sign_bit=15),
    RegisterSpec("Present_Load", "当前负载", 60, 2, "sram_feedback", "0.1%", sign_bit=10),
    RegisterSpec("Present_Voltage", "当前电压", 62, 1, "sram_feedback", "0.1V"),
    RegisterSpec("Present_Temperature", "当前温度", 63, 1, "sram_feedback", "C"),
    RegisterSpec("Async_Write_Flag", "异步写标志", 64, 1, "sram_feedback"),
    RegisterSpec("Status", "舵机状态", 65, 1, "sram_feedback"),
    RegisterSpec("Moving", "移动标志", 66, 1, "sram_feedback"),
    RegisterSpec("Goal_Position_Echo", "目标位置反馈", 67, 2, "sram_feedback", "0.087deg", sign_bit=15),
    RegisterSpec("Present_Current", "当前电流", 69, 2, "sram_feedback", "6.5mA"),
    RegisterSpec("Moving_Velocity_Threshold", "移动速度阀值", 80, 1, "factory"),
    RegisterSpec("DTs", "DTs(ms)", 81, 1, "factory"),
    RegisterSpec("Velocity_Unit_Factor", "速度单位系数", 82, 1, "factory"),
    RegisterSpec("Minimum_Velocity_Limit", "最小速度限制", 83, 1, "factory", "0.732RPM"),
    RegisterSpec("Maximum_Velocity_Limit", "最大速度限制", 84, 1, "factory", "0.732RPM"),
    RegisterSpec("Acceleration_Limit", "加速度限制", 85, 1, "factory"),
    RegisterSpec("Acceleration_Multiplier", "加速度倍数", 86, 1, "factory"),
)

REGISTER_BY_KEY = {spec.key: spec for spec in DOCUMENTED_REGISTERS}
GROUPS = tuple(sorted({spec.group for spec in DOCUMENTED_REGISTERS}))
DEFAULT_KEYS = (
    "ID",
    "Operating_Mode",
    "Acceleration",
    "Goal_Position",
    "Goal_Velocity",
    "Present_Position",
    "Present_Velocity",
    "Status",
    "Moving",
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Read STS3215 registers using the official map from docs/1.txt."
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port.")
    parser.add_argument("--baudrate", type=int, default=1_000_000, help="Motor bus baudrate.")
    parser.add_argument("--protocol-version", type=int, default=0, help="Feetech protocol version.")
    parser.add_argument("--model", default="sts3215", help="Motor model name.")
    parser.add_argument("--id", type=int, default=None, help="Motor ID to read.")
    parser.add_argument("--name", action="append", default=None, help="Documented register key to read. Repeatable.")
    parser.add_argument(
        "--group",
        action="append",
        choices=GROUPS,
        default=None,
        help="Read a whole documented register group.",
    )
    parser.add_argument("--all-documented", action="store_true", help="Read every documented register in the map.")
    parser.add_argument("--list", action="store_true", help="List supported documented register keys and exit.")
    return parser.parse_args()


def decode_sign_magnitude(value: int, sign_bit: int) -> int:
    sign_mask = 1 << sign_bit
    magnitude_mask = sign_mask - 1
    magnitude = value & magnitude_mask
    return -magnitude if value & sign_mask else magnitude


def select_registers(args: argparse.Namespace) -> list[RegisterSpec]:
    if args.all_documented:
        return list(DOCUMENTED_REGISTERS)

    selected_keys: list[str] = []
    if args.group:
        for group in args.group:
            selected_keys.extend(spec.key for spec in DOCUMENTED_REGISTERS if spec.group == group)
    if args.name:
        selected_keys.extend(args.name)

    if not selected_keys:
        selected_keys = list(DEFAULT_KEYS)

    seen: set[str] = set()
    selected_specs: list[RegisterSpec] = []
    for key in selected_keys:
        if key not in REGISTER_BY_KEY:
            valid_keys = ", ".join(spec.key for spec in DOCUMENTED_REGISTERS)
            raise ValueError(f"Unknown register key '{key}'. Valid keys: {valid_keys}")
        if key not in seen:
            selected_specs.append(REGISTER_BY_KEY[key])
            seen.add(key)
    return selected_specs


def make_bus(*, port: str, baudrate: int, protocol_version: int, model: str, motor_id: int):
    FeetechMotorsBus, Motor, MotorNormMode = require_lerobot()
    motors = {"target": Motor(id=motor_id, model=model, norm_mode=MotorNormMode.DEGREES)}
    bus = FeetechMotorsBus(port=port, motors=motors, protocol_version=protocol_version)
    bus.connect(handshake=False)
    bus.set_baudrate(baudrate)
    return bus


def read_register(bus, motor_id: int, spec: RegisterSpec) -> dict[str, object]:
    value, comm, error = bus._read(  # noqa: SLF001
        spec.address,
        spec.length,
        motor_id,
        raise_on_error=False,
    )
    payload: dict[str, object] = {
        "key": spec.key,
        "label_cn": spec.label_cn,
        "address_dec": spec.address,
        "address_hex": f"0x{spec.address:02X}",
        "length": spec.length,
        "group": spec.group,
        "unit": spec.unit,
    }

    if not bus._is_comm_success(comm):  # noqa: SLF001
        payload["comm_error"] = bus.packet_handler.getTxRxResult(comm)
        return payload
    if bus._is_error(error):  # noqa: SLF001
        payload["packet_error"] = bus.packet_handler.getRxPacketError(error)
        return payload

    payload["raw"] = int(value)
    if spec.sign_bit is not None:
        payload["decoded"] = int(decode_sign_magnitude(int(value), spec.sign_bit))
    return payload


def print_register_list() -> None:
    for spec in DOCUMENTED_REGISTERS:
        print(
            f"{spec.key:28s} addr={spec.address:>2d} len={spec.length} "
            f"group={spec.group:12s} label={spec.label_cn}"
        )


def main() -> None:
    args = parse_args()
    if args.list:
        print_register_list()
        return
    if args.id is None:
        raise ValueError("--id is required unless --list is used.")

    specs = select_registers(args)
    bus = make_bus(
        port=args.port,
        baudrate=args.baudrate,
        protocol_version=args.protocol_version,
        model=args.model,
        motor_id=args.id,
    )
    try:
        payload = {
            "port": args.port,
            "baudrate": args.baudrate,
            "protocol_version": args.protocol_version,
            "motor_id": args.id,
            "registers": [read_register(bus, args.id, spec) for spec in specs],
        }
    finally:
        bus.disconnect(disable_torque=False)

    print(json.dumps(payload, indent=2, sort_keys=True, ensure_ascii=False))


if __name__ == "__main__":
    main()
