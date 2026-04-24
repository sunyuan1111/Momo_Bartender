from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Literal


JointKind = Literal["arm", "gripper"]


@dataclass(frozen=True)
class JointConfig:
    name: str
    motor_id: int
    model: str = "sts3215"
    kind: JointKind = "arm"
    gear_ratio: float = 1.0
    direction: int = 1
    running_time_ms: int | None = None
    acceleration: int | None = None
    min_position_deg: float | None = None
    max_position_deg: float | None = None

    def __post_init__(self) -> None:
        if self.direction not in (-1, 1):
            raise ValueError(f"{self.name}: direction must be -1 or 1.")
        if self.gear_ratio <= 0:
            raise ValueError(f"{self.name}: gear_ratio must be positive.")
        if self.kind not in ("arm", "gripper"):
            raise ValueError(f"{self.name}: unsupported joint kind '{self.kind}'.")


@dataclass(frozen=True)
class ArmConfig:
    port: str = "/dev/ttyUSB0"
    baudrate: int = 1_000_000
    protocol_version: int = 0
    handshake: bool = True
    operating_mode: int = 3
    default_running_time_ms: int = 600
    default_acceleration: int = 254
    configure_motors_on_connect: bool = True
    enable_torque_on_connect: bool = True
    disable_torque_on_disconnect: bool = True
    urdf_path: str | None = None
    cartesian_base_link: str = "base_link"
    cartesian_tip_link: str = "link6"
    cartesian_max_iterations: int = 100
    cartesian_position_tolerance_m: float = 1e-3
    cartesian_damping: float = 2e-2
    cartesian_max_step_deg: float = 8.0
    joints: tuple[JointConfig, ...] = field(default_factory=tuple)

    def __post_init__(self) -> None:
        names = [joint.name for joint in self.joints]
        if len(names) != len(set(names)):
            raise ValueError("Joint names must be unique.")
        ids = [joint.motor_id for joint in self.joints]
        if len(ids) != len(set(ids)):
            raise ValueError("Motor IDs must be unique.")

    @property
    def joint_map(self) -> dict[str, JointConfig]:
        return {joint.name: joint for joint in self.joints}

    @property
    def has_urdf_kinematics(self) -> bool:
        return self.urdf_path is not None

    @property
    def arm_joint_names(self) -> tuple[str, ...]:
        return tuple(joint.name for joint in self.joints if joint.kind == "arm")

    @property
    def gripper_joint_name(self) -> str:
        grippers = [joint.name for joint in self.joints if joint.kind == "gripper"]
        if len(grippers) != 1:
            raise ValueError("Expected exactly one gripper joint in the config.")
        return grippers[0]

    def require_joint(self, joint_name: str) -> JointConfig:
        try:
            return self.joint_map[joint_name]
        except KeyError as exc:
            raise KeyError(f"Unknown joint '{joint_name}'.") from exc

    def to_dict(self) -> dict:
        payload = asdict(self)
        payload["joints"] = [asdict(joint) for joint in self.joints]
        return payload

    @classmethod
    def default(cls) -> "ArmConfig":
        return cls(
            joints=(
                JointConfig(name="joint_1", motor_id=1),
                JointConfig(name="joint_2", motor_id=2, gear_ratio=14.0),
                JointConfig(name="joint_3", motor_id=3, gear_ratio=14.0),
                JointConfig(name="joint_4", motor_id=4),
                JointConfig(name="joint_5", motor_id=5),
                JointConfig(name="joint_6", motor_id=6),
                JointConfig(name="gripper", motor_id=7, kind="gripper"),
            )
        )

    @classmethod
    def from_dict(cls, payload: dict) -> "ArmConfig":
        payload = dict(payload)
        joints_payload = payload.pop("joints", [])
        joints = tuple(JointConfig(**joint_payload) for joint_payload in joints_payload)
        return cls(joints=joints, **payload)

    @classmethod
    def from_json(cls, path: str | Path) -> "ArmConfig":
        path = Path(path)
        payload = json.loads(path.read_text())
        urdf_path = payload.get("urdf_path")
        if urdf_path is not None:
            urdf_path = Path(urdf_path)
            if not urdf_path.is_absolute():
                payload["urdf_path"] = str((path.parent / urdf_path).resolve())
        return cls.from_dict(payload)

    def save_json(self, path: str | Path) -> None:
        Path(path).write_text(json.dumps(self.to_dict(), indent=2) + "\n")
