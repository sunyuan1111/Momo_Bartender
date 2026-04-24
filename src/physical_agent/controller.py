from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

from .config import ArmConfig, JointConfig
from .lerobot_compat import require_lerobot


STS3215_RESOLUTION = 4096
STS3215_PHASE_ANGLE_FEEDBACK_BIT = 0x10


@dataclass(frozen=True)
class JointCommand:
    joint_name: str
    target_deg: float
    delta_deg: float
    delta_raw: int


class Sts3215ArmController:
    """Simple step-mode arm controller built on LeRobot's Feetech bus."""

    def __init__(self, config: ArmConfig):
        self.config = config
        self.bus = None
        self._startup_raw_positions: dict[str, int] = {}
        self._target_positions_deg: dict[str, float] = {joint.name: 0.0 for joint in self.config.joints}

    @classmethod
    def from_json(cls, path: str) -> "Sts3215ArmController":
        return cls(ArmConfig.from_json(path))

    def __enter__(self) -> "Sts3215ArmController":
        self.connect()
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.disconnect()

    @property
    def is_connected(self) -> bool:
        return bool(self.bus and self.bus.is_connected)

    @property
    def startup_raw_positions(self) -> dict[str, int]:
        return dict(self._startup_raw_positions)

    @property
    def target_positions_deg(self) -> dict[str, float]:
        return dict(self._target_positions_deg)

    def connect(self) -> None:
        if self.is_connected:
            return

        FeetechMotorsBus, Motor, MotorNormMode = require_lerobot()
        motors = {
            joint.name: Motor(
                id=joint.motor_id,
                model=joint.model,
                norm_mode=MotorNormMode.DEGREES,
            )
            for joint in self.config.joints
        }

        bus = FeetechMotorsBus(
            port=self.config.port,
            motors=motors,
            protocol_version=self.config.protocol_version,
        )
        bus.connect(handshake=False)
        bus.set_baudrate(self.config.baudrate)
        self.bus = bus

        if self.config.handshake:
            self._ping_all_expected_motors()

        if self.config.configure_motors_on_connect:
            self._configure_servos_for_step_mode()

        self._startup_raw_positions = self.read_present_raw_positions()
        self._target_positions_deg = {joint.name: 0.0 for joint in self.config.joints}

        if self.config.enable_torque_on_connect:
            self.bus.enable_torque()

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        assert self.bus is not None
        self.bus.disconnect(disable_torque=self.config.disable_torque_on_disconnect)
        self.bus = None

    def initialize(self) -> dict[str, int]:
        self.connect()
        return self.read_present_raw_positions()

    def read_present_raw_positions(self) -> dict[str, int]:
        self._require_connected()
        assert self.bus is not None
        return {
            joint.name: int(self.bus.read("Present_Position", joint.name, normalize=False))
            for joint in self.config.joints
        }

    def read_state(self) -> dict:
        return {
            "port": self.config.port,
            "baudrate": self.config.baudrate,
            "operating_mode": self.config.operating_mode,
            "startup_raw_positions": self.startup_raw_positions,
            "present_raw_positions": self.read_present_raw_positions(),
            "target_positions_deg": self.target_positions_deg,
        }

    def home(self, duration_ms: int | None = None) -> dict[str, int]:
        zero_targets = {joint_name: 0.0 for joint_name in self.config.arm_joint_names}
        return self.move_joints(zero_targets, duration_ms=duration_ms)

    def move_joint(self, joint_name: str, target_deg: float, duration_ms: int | None = None) -> int:
        return self.move_joints({joint_name: target_deg}, duration_ms=duration_ms)[joint_name]

    def nudge_joint(self, joint_name: str, delta_deg: float, duration_ms: int | None = None) -> int:
        current_target = self._target_positions_deg[joint_name]
        return self.move_joint(joint_name, current_target + delta_deg, duration_ms=duration_ms)

    def move_joints(
        self,
        target_positions_deg: Mapping[str, float],
        *,
        duration_ms: int | None = None,
    ) -> dict[str, int]:
        self._require_connected()
        assert self.bus is not None

        commands = [self._build_joint_command(joint_name, target_deg) for joint_name, target_deg in target_positions_deg.items()]
        result: dict[str, int] = {}
        for command in commands:
            joint = self.config.require_joint(command.joint_name)
            self._apply_motion_profile(joint, duration_ms=duration_ms)
            if command.delta_raw != 0:
                self.bus.write("Goal_Position", joint.name, int(command.delta_raw), normalize=False)
                self._target_positions_deg[joint.name] = command.target_deg
            result[joint.name] = command.delta_raw
        return result

    def move_arm(self, target_positions_deg: Mapping[str, float], *, duration_ms: int | None = None) -> dict[str, int]:
        invalid = [name for name in target_positions_deg if name not in self.config.arm_joint_names]
        if invalid:
            raise ValueError(f"Non-arm joints passed to move_arm: {invalid}")
        return self.move_joints(target_positions_deg, duration_ms=duration_ms)

    def move_gripper(self, target_deg: float, *, duration_ms: int | None = None) -> int:
        return self.move_joint(self.config.gripper_joint_name, target_deg, duration_ms=duration_ms)

    def nudge_gripper(self, delta_deg: float, *, duration_ms: int | None = None) -> int:
        gripper_name = self.config.gripper_joint_name
        return self.nudge_joint(gripper_name, delta_deg, duration_ms=duration_ms)

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError("Controller is not connected.")

    def _ping_all_expected_motors(self) -> None:
        assert self.bus is not None
        missing: list[str] = []
        for joint in self.config.joints:
            model_number = self.bus.ping(joint.name, raise_on_error=False)
            if model_number is None:
                missing.append(joint.name)
        if missing:
            raise ConnectionError(f"Failed to ping expected motors: {missing}")

    def _configure_servos_for_step_mode(self) -> None:
        assert self.bus is not None
        self.bus.disable_torque()

        if hasattr(self.bus, "configure_motors"):
            self.bus.configure_motors(acceleration=self.config.default_acceleration)

        for joint in self.config.joints:
            if joint.model.lower() == "sts3215":
                self._clear_phase_angle_feedback_bit(joint.name)
            self.bus.write("Operating_Mode", joint.name, self.config.operating_mode, normalize=False)
            self.bus.write(
                "Acceleration",
                joint.name,
                self._joint_acceleration(joint),
                normalize=False,
            )
            if joint.running_time_ms is not None:
                self.bus.write("Running_Time", joint.name, joint.running_time_ms, normalize=False)

    def _clear_phase_angle_feedback_bit(self, joint_name: str) -> None:
        assert self.bus is not None
        phase_value = int(self.bus.read("Phase", joint_name, normalize=False))
        if phase_value & STS3215_PHASE_ANGLE_FEEDBACK_BIT:
            self.bus.write(
                "Phase",
                joint_name,
                phase_value & ~STS3215_PHASE_ANGLE_FEEDBACK_BIT,
                normalize=False,
            )

    def _joint_acceleration(self, joint: JointConfig) -> int:
        return int(joint.acceleration if joint.acceleration is not None else self.config.default_acceleration)

    def _joint_running_time(self, joint: JointConfig, duration_ms: int | None) -> int:
        if duration_ms is not None:
            return int(duration_ms)
        if joint.running_time_ms is not None:
            return int(joint.running_time_ms)
        return int(self.config.default_running_time_ms)

    def _apply_motion_profile(self, joint: JointConfig, *, duration_ms: int | None) -> None:
        assert self.bus is not None
        self.bus.write("Acceleration", joint.name, self._joint_acceleration(joint), normalize=False)
        self.bus.write("Running_Time", joint.name, self._joint_running_time(joint, duration_ms), normalize=False)

    def _build_joint_command(self, joint_name: str, target_deg: float) -> JointCommand:
        joint = self.config.require_joint(joint_name)
        self._validate_joint_limit(joint, target_deg)
        current_target = self._target_positions_deg[joint_name]
        delta_deg = target_deg - current_target
        delta_raw = self._joint_delta_deg_to_raw(joint, delta_deg)
        return JointCommand(
            joint_name=joint_name,
            target_deg=float(target_deg),
            delta_deg=float(delta_deg),
            delta_raw=delta_raw,
        )

    def _validate_joint_limit(self, joint: JointConfig, target_deg: float) -> None:
        if joint.min_position_deg is not None and target_deg < joint.min_position_deg:
            raise ValueError(f"{joint.name}: target {target_deg} deg is below {joint.min_position_deg} deg.")
        if joint.max_position_deg is not None and target_deg > joint.max_position_deg:
            raise ValueError(f"{joint.name}: target {target_deg} deg is above {joint.max_position_deg} deg.")

    def _joint_delta_deg_to_raw(self, joint: JointConfig, delta_deg: float) -> int:
        motor_delta_deg = delta_deg * joint.gear_ratio * joint.direction
        return int(round(motor_delta_deg / 360.0 * STS3215_RESOLUTION))
