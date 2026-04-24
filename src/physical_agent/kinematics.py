from __future__ import annotations

import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class UrdfJoint:
    name: str
    joint_type: str
    parent_link: str
    child_link: str
    origin_xyz: tuple[float, float, float]
    origin_rpy: tuple[float, float, float]
    axis_xyz: tuple[float, float, float]
    lower_rad: float | None
    upper_rad: float | None

    @property
    def is_actuated(self) -> bool:
        return self.joint_type in {"revolute", "continuous"}


def _parse_floats(raw: str | None, expected: int, default: float) -> tuple[float, ...]:
    if not raw:
        return tuple(default for _ in range(expected))
    values = tuple(float(value) for value in raw.split())
    if len(values) != expected:
        raise ValueError(f"Expected {expected} values, got {values}")
    return values


def _rotation_x(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rotation_y(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rotation_z(angle: float) -> np.ndarray:
    c = math.cos(angle)
    s = math.sin(angle)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def _rotation_from_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    return _rotation_z(yaw) @ _rotation_y(pitch) @ _rotation_x(roll)


def _rotation_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    x, y, z = axis
    c = math.cos(angle)
    s = math.sin(angle)
    one_minus_c = 1.0 - c
    return np.array(
        [
            [c + x * x * one_minus_c, x * y * one_minus_c - z * s, x * z * one_minus_c + y * s],
            [y * x * one_minus_c + z * s, c + y * y * one_minus_c, y * z * one_minus_c - x * s],
            [z * x * one_minus_c - y * s, z * y * one_minus_c + x * s, c + z * z * one_minus_c],
        ]
    )


def _transform_from_origin(xyz: tuple[float, float, float], rpy: tuple[float, float, float]) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rotation_from_rpy(*rpy)
    transform[:3, 3] = np.asarray(xyz, dtype=float)
    return transform


def _transform_from_axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = _rotation_from_axis_angle(axis, angle)
    return transform


class UrdfArmKinematics:
    def __init__(self, urdf_path: str | Path, joint_names: tuple[str, ...], base_link: str, tip_link: str):
        self.urdf_path = str(urdf_path)
        self.base_link = base_link
        self.tip_link = tip_link
        self.chain = self._build_chain(Path(urdf_path), base_link=base_link, tip_link=tip_link)
        self.joint_names = tuple(joint_names)
        chain_joint_names = tuple(joint.name for joint in self.chain if joint.is_actuated)
        if chain_joint_names != self.joint_names:
            raise ValueError(
                f"URDF joint chain {chain_joint_names} does not match configured arm joints {self.joint_names}."
            )

    @staticmethod
    def _build_chain(urdf_path: Path, *, base_link: str, tip_link: str) -> tuple[UrdfJoint, ...]:
        root = ET.fromstring(urdf_path.read_text())
        child_to_joint: dict[str, UrdfJoint] = {}

        for joint_elem in root.findall("joint"):
            name = joint_elem.attrib["name"]
            joint_type = joint_elem.attrib["type"]
            parent_link = joint_elem.find("parent").attrib["link"]
            child_link = joint_elem.find("child").attrib["link"]
            origin_elem = joint_elem.find("origin")
            axis_elem = joint_elem.find("axis")
            limit_elem = joint_elem.find("limit")
            child_to_joint[child_link] = UrdfJoint(
                name=name,
                joint_type=joint_type,
                parent_link=parent_link,
                child_link=child_link,
                origin_xyz=_parse_floats(origin_elem.attrib.get("xyz") if origin_elem is not None else None, 3, 0.0),
                origin_rpy=_parse_floats(origin_elem.attrib.get("rpy") if origin_elem is not None else None, 3, 0.0),
                axis_xyz=_parse_floats(axis_elem.attrib.get("xyz") if axis_elem is not None else None, 3, 0.0),
                lower_rad=float(limit_elem.attrib["lower"]) if limit_elem is not None and "lower" in limit_elem.attrib else None,
                upper_rad=float(limit_elem.attrib["upper"]) if limit_elem is not None and "upper" in limit_elem.attrib else None,
            )

        chain: list[UrdfJoint] = []
        current_link = tip_link
        while current_link != base_link:
            joint = child_to_joint.get(current_link)
            if joint is None:
                raise ValueError(f"Could not find a URDF chain from {base_link} to {tip_link}.")
            chain.append(joint)
            current_link = joint.parent_link

        chain.reverse()
        return tuple(chain)

    def forward_position(self, joint_positions_deg: Mapping[str, float]) -> np.ndarray:
        transform = self._forward_transform(self._joint_positions_rad(joint_positions_deg))
        return transform[:3, 3].copy()

    def solve_position_ik(
        self,
        target_xyz: tuple[float, float, float],
        initial_positions_deg: Mapping[str, float],
        *,
        max_iterations: int,
        position_tolerance_m: float,
        damping: float,
        max_step_deg: float,
    ) -> dict[str, float]:
        target = np.asarray(target_xyz, dtype=float)
        positions_rad = self._joint_positions_rad(initial_positions_deg)
        max_step_rad = math.radians(max_step_deg)

        for _ in range(max_iterations):
            tip_position, jacobian = self._forward_position_and_jacobian(positions_rad)
            error = target - tip_position
            if np.linalg.norm(error) <= position_tolerance_m:
                return {joint_name: math.degrees(positions_rad[joint_name]) for joint_name in self.joint_names}

            damping_matrix = (damping**2) * np.eye(3)
            try:
                delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping_matrix, error)
            except np.linalg.LinAlgError:
                delta = np.linalg.pinv(jacobian) @ error

            step_norm = float(np.max(np.abs(delta))) if delta.size else 0.0
            if step_norm > max_step_rad:
                delta *= max_step_rad / step_norm

            for index, joint_name in enumerate(self.joint_names):
                positions_rad[joint_name] += float(delta[index])
                joint = next(chain_joint for chain_joint in self.chain if chain_joint.name == joint_name)
                if joint.lower_rad is not None:
                    positions_rad[joint_name] = max(positions_rad[joint_name], joint.lower_rad)
                if joint.upper_rad is not None:
                    positions_rad[joint_name] = min(positions_rad[joint_name], joint.upper_rad)

        raise ValueError(
            f"IK did not converge to {target_xyz} within {max_iterations} iterations."
        )

    def _joint_positions_rad(self, joint_positions_deg: Mapping[str, float]) -> dict[str, float]:
        return {joint_name: math.radians(float(joint_positions_deg[joint_name])) for joint_name in self.joint_names}

    def _forward_transform(self, joint_positions_rad: Mapping[str, float]) -> np.ndarray:
        transform = np.eye(4)
        for joint in self.chain:
            transform = transform @ _transform_from_origin(joint.origin_xyz, joint.origin_rpy)
            if joint.is_actuated:
                transform = transform @ _transform_from_axis_angle(np.asarray(joint.axis_xyz, dtype=float), joint_positions_rad[joint.name])
        return transform

    def _forward_position_and_jacobian(self, joint_positions_rad: Mapping[str, float]) -> tuple[np.ndarray, np.ndarray]:
        transform = np.eye(4)
        joint_origins_world: list[np.ndarray] = []
        joint_axes_world: list[np.ndarray] = []

        for joint in self.chain:
            transform = transform @ _transform_from_origin(joint.origin_xyz, joint.origin_rpy)
            if joint.is_actuated:
                joint_origins_world.append(transform[:3, 3].copy())
                axis_world = transform[:3, :3] @ np.asarray(joint.axis_xyz, dtype=float)
                joint_axes_world.append(axis_world)
                transform = transform @ _transform_from_axis_angle(np.asarray(joint.axis_xyz, dtype=float), joint_positions_rad[joint.name])

        tip_position = transform[:3, 3].copy()
        jacobian = np.zeros((3, len(self.joint_names)))
        for index, (origin_world, axis_world) in enumerate(zip(joint_origins_world, joint_axes_world, strict=True)):
            jacobian[:, index] = np.cross(axis_world, tip_position - origin_world)

        return tip_position, jacobian
