from __future__ import annotations

from collections.abc import Mapping as MappingABC
from dataclasses import dataclass
from math import ceil
from typing import Mapping

from .config import ArmConfig, JointConfig
from .kinematics import UrdfArmKinematics
from .lerobot_compat import require_lerobot


STS3215_RESOLUTION = 4096
STS3215_PHASE_ANGLE_FEEDBACK_BIT = 0x10
STS3215_MAX_GOAL_VELOCITY = 32767
JointSpeedOverride = float | Mapping[str, float] | None


@dataclass(frozen=True)
class JointCommand:
    joint_name: str
    target_deg: float
    goal_raw: int


class Sts3215ArmController:
    """Simple position-servo arm controller built on LeRobot's Feetech bus."""

    def __init__(self, config: ArmConfig):
        self.config = config
        self.bus = None
        self._kinematics = (
            UrdfArmKinematics(
                urdf_path=self.config.urdf_path,
                joint_names=self.config.arm_joint_names,
                base_link=self.config.cartesian_base_link,
                tip_link=self.config.cartesian_tip_link,
            )
            if self.config.has_urdf_kinematics
            else None
        )
        self._startup_raw_positions: dict[str, int] = {}
        self._startup_positions_deg: dict[str, float] = {}
        self._target_positions_deg: dict[str, float] = {joint.name: 0.0 for joint in self.config.joints}
        self._online_joint_names: tuple[str, ...] = tuple(joint.name for joint in self.config.joints)
        self._missing_joint_names: tuple[str, ...] = ()

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
    def startup_positions_deg(self) -> dict[str, float]:
        return dict(self._startup_positions_deg)

    @property
    def target_positions_deg(self) -> dict[str, float]:
        return dict(self._target_positions_deg)

    @property
    def online_joint_names(self) -> tuple[str, ...]:
        return self._online_joint_names

    @property
    def missing_joint_names(self) -> tuple[str, ...]:
        return self._missing_joint_names

    @property
    def online_joints(self) -> tuple[JointConfig, ...]:
        return tuple(self.config.require_joint(joint_name) for joint_name in self._online_joint_names)

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
        else:
            self._online_joint_names = tuple(joint.name for joint in self.config.joints)
            self._missing_joint_names = ()

        if self.config.configure_motors_on_connect:
            self._configure_servos_for_position_mode()

        self._startup_raw_positions = self.read_present_raw_positions()
        self._startup_positions_deg = self._raw_positions_to_joint_degrees(self._startup_raw_positions)
        self._target_positions_deg = dict(self._startup_positions_deg)

        if self.config.enable_torque_on_connect:
            self.bus.enable_torque(list(self._online_joint_names))

    def disconnect(self) -> None:
        if not self.is_connected:
            return
        assert self.bus is not None
        try:
            if self.config.disable_torque_on_disconnect and self._online_joint_names:
                self.bus.disable_torque(list(self._online_joint_names), num_retry=5)
        finally:
            self.bus.disconnect(disable_torque=False)
            self.bus = None

    def initialize(self) -> dict[str, int]:
        self.connect()
        return self.read_present_raw_positions()

    def read_present_raw_positions(self) -> dict[str, int]:
        self._require_connected()
        assert self.bus is not None
        return {
            joint.name: int(self.bus.read("Present_Position", joint.name, normalize=False))
            for joint in self.online_joints
        }

    def read_present_positions_deg(self) -> dict[str, float]:
        return self._raw_positions_to_joint_degrees(self.read_present_raw_positions())

    def read_state(self) -> dict:
        present_raw_positions = self.read_present_raw_positions()
        state = {
            "port": self.config.port,
            "baudrate": self.config.baudrate,
            "operating_mode": self.config.operating_mode,
            "online_joint_names": list(self._online_joint_names),
            "missing_joint_names": list(self._missing_joint_names),
            "startup_raw_positions": self.startup_raw_positions,
            "startup_positions_deg": self.startup_positions_deg,
            "present_raw_positions": present_raw_positions,
            "present_positions_deg": self._raw_positions_to_joint_degrees(present_raw_positions),
            "zero_position_raw_by_joint": {
                joint.name: joint.zero_position_raw for joint in self.config.joints
            },
            "target_positions_deg": self.target_positions_deg,
        }
        if self._kinematics is not None and self._all_arm_joints_online():
            state["cartesian_position_m"] = self.forward_kinematics()
        return state

    def home(self, speed_deg_s: float | None = None) -> dict[str, int]:
        zero_targets = {joint_name: 0.0 for joint_name in self.config.arm_joint_names}
        return self.move_joints(zero_targets, speed_deg_s=speed_deg_s)

    def move_joint(
        self,
        joint_name: str,
        target_deg: float,
        speed_deg_s: float | None = None,
    ) -> int:
        return self.move_joints(
            {joint_name: target_deg},
            speed_deg_s=speed_deg_s,
        )[joint_name]

    def joint_position_deg_to_raw(self, joint_name: str, target_deg: float) -> int:
        joint = self.config.require_joint(joint_name)
        self._validate_joint_limit(joint, target_deg)
        return self._joint_position_deg_to_raw(joint, float(target_deg))

    def joint_raw_to_position_deg(self, joint_name: str, raw_position: int) -> float:
        joint = self.config.require_joint(joint_name)
        return self._joint_raw_to_position_deg(joint, int(raw_position))

    def nudge_joint(
        self,
        joint_name: str,
        delta_deg: float,
        speed_deg_s: float | None = None,
    ) -> int:
        self._require_online_joints([joint_name])
        current_target = self.read_present_positions_deg()[joint_name]
        self._target_positions_deg[joint_name] = current_target
        return self.move_joint(
            joint_name,
            current_target + delta_deg,
            speed_deg_s=speed_deg_s,
        )

    def move_joints(
        self,
        target_positions_deg: Mapping[str, float],
        *,
        speed_deg_s: JointSpeedOverride = None,
    ) -> dict[str, int]:
        self._require_connected()
        assert self.bus is not None

        commands = [self._build_joint_command(joint_name, target_deg) for joint_name, target_deg in target_positions_deg.items()]
        result: dict[str, int] = {}
        for command in commands:
            joint = self.config.require_joint(command.joint_name)
            self._apply_motion_profile(
                joint,
                speed_deg_s=self._resolve_joint_speed_deg_s(command.joint_name, speed_deg_s),
            )
            self.bus.write("Goal_Position", joint.name, int(command.goal_raw), normalize=False)
            self._target_positions_deg[joint.name] = command.target_deg
            result[joint.name] = command.goal_raw
        return result

    def move_arm(
        self,
        target_positions_deg: Mapping[str, float],
        *,
        speed_deg_s: JointSpeedOverride = None,
    ) -> dict[str, int]:
        invalid = [name for name in target_positions_deg if name not in self.config.arm_joint_names]
        if invalid:
            raise ValueError(f"Non-arm joints passed to move_arm: {invalid}")
        return self.move_joints(
            target_positions_deg,
            speed_deg_s=speed_deg_s,
        )

    def move_gripper(
        self,
        target_deg: float,
        *,
        speed_deg_s: float | None = None,
    ) -> int:
        return self.move_joint(
            self.config.gripper_joint_name,
            target_deg,
            speed_deg_s=speed_deg_s,
        )

    def nudge_gripper(
        self,
        delta_deg: float,
        *,
        speed_deg_s: float | None = None,
    ) -> int:
        gripper_name = self.config.gripper_joint_name
        return self.nudge_joint(
            gripper_name,
            delta_deg,
            speed_deg_s=speed_deg_s,
        )

    def forward_kinematics(self, joint_positions_deg: Mapping[str, float] | None = None) -> dict[str, float]:
        kinematics = self._require_kinematics()
        arm_positions_deg = self._arm_positions_deg(joint_positions_deg)
        position = kinematics.forward_position(arm_positions_deg)
        return {"x": float(position[0]), "y": float(position[1]), "z": float(position[2])}

    def solve_cartesian(self, x: float, y: float, z: float, joint_positions_deg: Mapping[str, float] | None = None) -> dict[str, float]:
        kinematics = self._require_kinematics()
        arm_positions_deg = self._arm_positions_deg(joint_positions_deg)
        return kinematics.solve_position_ik(
            (x, y, z),
            arm_positions_deg,
            max_iterations=self.config.cartesian_max_iterations,
            position_tolerance_m=self.config.cartesian_position_tolerance_m,
            damping=self.config.cartesian_damping,
            max_step_deg=self.config.cartesian_max_step_deg,
        )

    def move_cartesian(
        self,
        x: float,
        y: float,
        z: float,
        *,
        speed_deg_s: float | None = None,
    ) -> dict[str, int]:
        target_positions_deg = self.solve_cartesian(x, y, z)
        return self.move_arm(
            target_positions_deg,
            speed_deg_s=speed_deg_s,
        )

    def nudge_cartesian(
        self,
        dx: float,
        dy: float,
        dz: float,
        *,
        speed_deg_s: float | None = None,
    ) -> dict[str, int]:
        current_position = self.forward_kinematics()
        return self.move_cartesian(
            current_position["x"] + dx,
            current_position["y"] + dy,
            current_position["z"] + dz,
            speed_deg_s=speed_deg_s,
        )

    def _require_connected(self) -> None:
        if not self.is_connected:
            raise RuntimeError("Controller is not connected.")

    def _require_kinematics(self) -> UrdfArmKinematics:
        if self._kinematics is None:
            raise RuntimeError("Cartesian control requires a URDF path in the arm config.")
        return self._kinematics

    def _arm_positions_deg(self, joint_positions_deg: Mapping[str, float] | None) -> dict[str, float]:
        if joint_positions_deg is None:
            self._require_online_joints(self.config.arm_joint_names)
            present_positions_deg = self.read_present_positions_deg()
            return {joint_name: present_positions_deg[joint_name] for joint_name in self.config.arm_joint_names}
        self._require_online_joints(self.config.arm_joint_names)
        arm_positions_deg = {joint_name: float(joint_positions_deg[joint_name]) for joint_name in self.config.arm_joint_names}
        return arm_positions_deg

    def _ping_all_expected_motors(self) -> None:
        assert self.bus is not None
        missing: list[str] = []
        online: list[str] = []
        for joint in self.config.joints:
            model_number = self.bus.ping(joint.name, raise_on_error=False)
            if model_number is None:
                missing.append(joint.name)
            else:
                online.append(joint.name)
        if not online:
            raise ConnectionError(f"Failed to ping any expected motors: {missing}")
        self._online_joint_names = tuple(online)
        self._missing_joint_names = tuple(missing)

    def _configure_servos_for_position_mode(self) -> None:
        assert self.bus is not None
        self.bus.disable_torque(list(self._online_joint_names))

        for joint in self.online_joints:
            self._configure_servo_defaults(joint)
            if joint.model.lower() == "sts3215":
                self._enable_full_angle_feedback(joint.name)
            self.bus.write("Operating_Mode", joint.name, self.config.operating_mode, normalize=False)
            self.bus.write(
                "Acceleration",
                joint.name,
                self._joint_acceleration(joint),
                normalize=False,
            )
            goal_velocity = self._joint_goal_velocity(joint, speed_deg_s=None)
            if goal_velocity is not None:
                self.bus.write("Goal_Velocity", joint.name, goal_velocity, normalize=False)

    def _configure_servo_defaults(self, joint: JointConfig) -> None:
        assert self.bus is not None
        self.bus.write("Return_Delay_Time", joint.name, 0, normalize=False)
        if self.config.min_position_limit_raw is not None:
            self.bus.write(
                "Min_Position_Limit",
                joint.name,
                int(self.config.min_position_limit_raw),
                normalize=False,
            )
        if self.config.max_position_limit_raw is not None:
            self.bus.write(
                "Max_Position_Limit",
                joint.name,
                int(self.config.max_position_limit_raw),
                normalize=False,
            )
        if self.config.protocol_version == 0:
            self.bus.write("Maximum_Acceleration", joint.name, 254, normalize=False)

    def _enable_full_angle_feedback(self, joint_name: str) -> None:
        assert self.bus is not None
        phase_value = int(self.bus.read("Phase", joint_name, normalize=False))
        updated_phase = phase_value | STS3215_PHASE_ANGLE_FEEDBACK_BIT
        if updated_phase != phase_value:
            self.bus.write(
                "Phase",
                joint_name,
                updated_phase,
                normalize=False,
            )

    def _joint_acceleration(self, joint: JointConfig) -> int:
        return int(joint.acceleration if joint.acceleration is not None else self.config.default_acceleration)

    def _apply_motion_profile(
        self,
        joint: JointConfig,
        *,
        speed_deg_s: float | None,
    ) -> None:
        assert self.bus is not None
        self.bus.write("Acceleration", joint.name, self._joint_acceleration(joint), normalize=False)
        goal_velocity = self._joint_goal_velocity(
            joint,
            speed_deg_s=speed_deg_s,
        )
        if goal_velocity is not None:
            self.bus.write("Goal_Velocity", joint.name, goal_velocity, normalize=False)

    def _joint_goal_velocity(
        self,
        joint: JointConfig,
        *,
        speed_deg_s: float | None,
    ) -> int | None:
        resolved_speed_deg_s = speed_deg_s
        if resolved_speed_deg_s is None:
            resolved_speed_deg_s = joint.speed_deg_s
        if resolved_speed_deg_s is None:
            resolved_speed_deg_s = self.config.default_speed_deg_s
        if resolved_speed_deg_s is None:
            return None
        if resolved_speed_deg_s <= 0:
            raise ValueError(f"{joint.name}: speed_deg_s must be positive.")
        motor_deg_s = resolved_speed_deg_s * joint.gear_ratio
        goal_velocity = ceil(motor_deg_s / 360.0 * STS3215_RESOLUTION)
        return max(1, min(int(goal_velocity), STS3215_MAX_GOAL_VELOCITY))

    def _resolve_joint_speed_deg_s(
        self,
        joint_name: str,
        speed_deg_s: JointSpeedOverride,
    ) -> float | None:
        if speed_deg_s is None:
            return None
        if isinstance(speed_deg_s, MappingABC):
            value = speed_deg_s.get(joint_name)
            return None if value is None else float(value)
        return float(speed_deg_s)

    def _build_joint_command(self, joint_name: str, target_deg: float) -> JointCommand:
        joint = self.config.require_joint(joint_name)
        self._require_online_joints([joint_name])
        self._validate_joint_limit(joint, target_deg)
        goal_raw = self._joint_position_deg_to_raw(joint, target_deg)
        return JointCommand(
            joint_name=joint_name,
            target_deg=float(target_deg),
            goal_raw=goal_raw,
        )

    def _validate_joint_limit(self, joint: JointConfig, target_deg: float) -> None:
        if joint.min_position_deg is not None and target_deg < joint.min_position_deg:
            raise ValueError(f"{joint.name}: target {target_deg} deg is below {joint.min_position_deg} deg.")
        if joint.max_position_deg is not None and target_deg > joint.max_position_deg:
            raise ValueError(f"{joint.name}: target {target_deg} deg is above {joint.max_position_deg} deg.")

    def _joint_position_deg_to_raw(self, joint: JointConfig, position_deg: float) -> int:
        motor_delta_deg = position_deg * joint.gear_ratio * joint.direction
        goal_raw = int(round(joint.zero_position_raw + motor_delta_deg / 360.0 * STS3215_RESOLUTION))
        if not -32767 <= goal_raw <= 32767:
            raise ValueError(
                f"{joint.name}: target {position_deg} deg maps to raw {goal_raw}, outside [-32767, 32767]."
            )
        return goal_raw

    def _joint_raw_to_position_deg(self, joint: JointConfig, raw_position: int) -> float:
        motor_delta_raw = raw_position - joint.zero_position_raw
        return motor_delta_raw * 360.0 / STS3215_RESOLUTION / joint.gear_ratio * joint.direction

    def _raw_positions_to_joint_degrees(self, raw_positions: Mapping[str, int]) -> dict[str, float]:
        return {
            joint.name: self._joint_raw_to_position_deg(joint, int(raw_positions[joint.name]))
            for joint in self.config.joints
            if joint.name in raw_positions
        }

    def _all_arm_joints_online(self) -> bool:
        online = set(self._online_joint_names)
        return all(joint_name in online for joint_name in self.config.arm_joint_names)

    def _require_online_joints(self, joint_names: tuple[str, ...] | list[str]) -> None:
        offline = [joint_name for joint_name in joint_names if joint_name not in self._online_joint_names]
        if offline:
            raise ConnectionError(
                f"Joints are not available on the bus: {offline}. "
                f"Online joints: {list(self._online_joint_names)}"
            )
