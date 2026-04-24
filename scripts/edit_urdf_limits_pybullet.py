#!/usr/bin/env python3
"""PyBullet-based visual editor for URDF joint limits.

The editor reads limit data from a JSON file, loads the URDF into PyBullet GUI,
and exposes three controls per movable joint:

- pose slider in degrees
- lower limit slider in degrees
- upper limit slider in degrees

It can save the JSON sidecar and optionally write the chosen limits back into
the URDF `<limit>` elements.
"""

from __future__ import annotations

import argparse
import json
import math
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any


DEG_PER_RAD = 180.0 / math.pi
RAD_PER_DEG = math.pi / 180.0
MOVABLE_JOINT_TYPES = {"revolute", "prismatic"}
PYBULLET_MOVABLE_TYPES = {0, 1}  # REVOLUTE, PRISMATIC


@dataclass(frozen=True)
class JointRecord:
    name: str
    urdf_type: str
    lower_rad: float | None
    upper_rad: float | None
    effort: float | None
    velocity: float | None


@dataclass(frozen=True)
class PackageInfo:
    package_name: str | None
    package_root: Path


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Visual URDF joint-limit editor using PyBullet.")
    parser.add_argument("urdf", type=Path, help="Path to the URDF file.")
    parser.add_argument(
        "--limits-file",
        type=Path,
        required=True,
        help="JSON file storing joint limits.",
    )
    parser.add_argument(
        "--slider-min-deg",
        type=float,
        default=-360.0,
        help="Minimum degree value for PyBullet sliders.",
    )
    parser.add_argument(
        "--slider-max-deg",
        type=float,
        default=360.0,
        help="Maximum degree value for PyBullet sliders.",
    )
    parser.add_argument(
        "--slider-resolution-deg",
        type=float,
        default=0.5,
        help="Step size for degree sliders.",
    )
    parser.add_argument(
        "--camera-distance",
        type=float,
        default=0.8,
        help="Initial debug camera distance.",
    )
    parser.add_argument(
        "--camera-yaw",
        type=float,
        default=45.0,
        help="Initial debug camera yaw.",
    )
    parser.add_argument(
        "--camera-pitch",
        type=float,
        default=-25.0,
        help="Initial debug camera pitch.",
    )
    return parser


def import_pybullet() -> tuple[Any, Any]:
    try:
        import pybullet as pb
        import pybullet_data
    except ModuleNotFoundError as exc:
        raise SystemExit(
            "pybullet is not installed. Run ./scripts/setup_momo_env.sh or "
            "install it with `python3 -m pip install pybullet`."
        ) from exc
    return pb, pybullet_data


def parse_optional_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    text = raw.strip()
    if not text:
        return None
    return float(text)


def format_number(value: float) -> str:
    return format(value, ".10g")


def rad_to_deg(value: float | None) -> float:
    if value is None:
        return 0.0
    return value * DEG_PER_RAD


def deg_to_rad(value: float) -> float:
    return value * RAD_PER_DEG


def resolve_package_info(urdf_path: Path) -> PackageInfo:
    package_root = urdf_path.parent.parent
    package_xml = package_root / "package.xml"
    package_name = None
    if package_xml.exists():
        root = ET.parse(package_xml).getroot()
        name_elem = root.find("name")
        if name_elem is not None and name_elem.text:
            package_name = name_elem.text.strip()
    return PackageInfo(package_name=package_name, package_root=package_root)


def resolve_mesh_filename(filename: str, *, urdf_path: Path, package_info: PackageInfo) -> str:
    if filename.startswith("package://"):
        remainder = filename[len("package://") :]
        package_name, rel_path = remainder.split("/", 1)
        if package_info.package_name and package_name != package_info.package_name:
            raise ValueError(
                f"Unsupported package reference '{package_name}' in {urdf_path}. "
                f"Expected '{package_info.package_name}'."
            )
        return str((package_info.package_root / rel_path).resolve())
    return str((urdf_path.parent / filename).resolve())


def materialize_pybullet_urdf(urdf_path: Path) -> tuple[tempfile.TemporaryDirectory[str], Path]:
    package_info = resolve_package_info(urdf_path)
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    for mesh in root.findall(".//mesh"):
        filename = mesh.attrib.get("filename")
        if not filename:
            continue
        mesh.set("filename", resolve_mesh_filename(filename, urdf_path=urdf_path, package_info=package_info))

    temp_dir = tempfile.TemporaryDirectory(prefix="arm2_pybullet_")
    temp_urdf = Path(temp_dir.name) / urdf_path.name
    tree.write(temp_urdf, encoding="utf-8", xml_declaration=True)
    return temp_dir, temp_urdf


def collect_urdf_joints(urdf_path: Path) -> dict[str, JointRecord]:
    root = ET.parse(urdf_path).getroot()
    joints: dict[str, JointRecord] = {}
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type", "")
        if joint_type not in MOVABLE_JOINT_TYPES:
            continue
        name = joint.attrib.get("name")
        if not name:
            raise ValueError(f"{urdf_path}: found movable joint without a name.")
        limit = joint.find("limit")
        joints[name] = JointRecord(
            name=name,
            urdf_type=joint_type,
            lower_rad=parse_optional_number(limit.attrib.get("lower")) if limit is not None else None,
            upper_rad=parse_optional_number(limit.attrib.get("upper")) if limit is not None else None,
            effort=parse_optional_number(limit.attrib.get("effort")) if limit is not None else None,
            velocity=parse_optional_number(limit.attrib.get("velocity")) if limit is not None else None,
        )
    return joints


def build_initial_limits(urdf_path: Path) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    urdf_joints = collect_urdf_joints(urdf_path)
    defaults = {"effort": 1.0, "velocity": 1.0}
    joint_limits: dict[str, dict[str, float]] = {}
    for name, joint in urdf_joints.items():
        joint_limits[name] = {
            "lower": float(joint.lower_rad if joint.lower_rad is not None else -math.pi),
            "upper": float(joint.upper_rad if joint.upper_rad is not None else math.pi),
        }
        if joint.effort is not None:
            defaults["effort"] = float(joint.effort)
        if joint.velocity is not None:
            defaults["velocity"] = float(joint.velocity)
    return defaults, joint_limits


def load_limits_file(path: Path) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: limits file must contain a JSON object.")

    defaults_payload = payload.get("defaults", {})
    if not isinstance(defaults_payload, dict):
        raise ValueError(f"{path}: 'defaults' must be a JSON object.")

    joint_limits_payload = payload.get("joint_limits", {})
    if not isinstance(joint_limits_payload, dict):
        raise ValueError(f"{path}: 'joint_limits' must be a JSON object.")

    defaults = {
        "effort": float(defaults_payload.get("effort", 1.0)),
        "velocity": float(defaults_payload.get("velocity", 1.0)),
    }
    joint_limits: dict[str, dict[str, float]] = {}
    for joint_name, joint_payload in joint_limits_payload.items():
        if not isinstance(joint_payload, dict):
            raise ValueError(f"{path}: joint '{joint_name}' must map to an object.")
        lower = joint_payload.get("lower")
        upper = joint_payload.get("upper")
        if lower is None or upper is None:
            raise ValueError(f"{path}: joint '{joint_name}' must define both lower and upper.")
        joint_limits[joint_name] = {
            "lower": float(lower),
            "upper": float(upper),
        }
    return defaults, joint_limits


def save_limits_file(path: Path, defaults: dict[str, float], joint_limits: dict[str, dict[str, float]]) -> None:
    payload = {
        "defaults": {
            "effort": float(defaults["effort"]),
            "velocity": float(defaults["velocity"]),
        },
        "joint_limits": joint_limits,
    }
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def apply_limits_to_urdf(urdf_path: Path, defaults: dict[str, float], joint_limits: dict[str, dict[str, float]]) -> None:
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    known_joint_names = {joint.attrib.get("name") for joint in root.findall("joint")}
    unknown = sorted(set(joint_limits) - known_joint_names)
    if unknown:
        raise ValueError(f"Unknown joints in limits payload: {unknown}")

    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type", "")
        joint_name = joint.attrib.get("name", "")
        if joint_type not in MOVABLE_JOINT_TYPES or joint_name not in joint_limits:
            continue
        limit = joint.find("limit")
        if limit is None:
            limit = ET.SubElement(joint, "limit")
        lower = float(joint_limits[joint_name]["lower"])
        upper = float(joint_limits[joint_name]["upper"])
        if lower > upper:
            raise ValueError(f"{joint_name}: lower limit {lower} is greater than upper limit {upper}.")
        limit.set("lower", format_number(lower))
        limit.set("upper", format_number(upper))
        limit.set("effort", format_number(defaults["effort"]))
        limit.set("velocity", format_number(defaults["velocity"]))

    ET.indent(tree, space="  ")
    tree.write(urdf_path, encoding="utf-8", xml_declaration=True)


def load_or_seed_limits(urdf_path: Path, limits_path: Path) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    if limits_path.exists():
        return load_limits_file(limits_path)
    return build_initial_limits(urdf_path)


def add_button(pb: Any, label: str) -> int:
    return pb.addUserDebugParameter(label, 1, 0, 0)


def read_button(pb: Any, button_id: int) -> int:
    return int(pb.readUserDebugParameter(button_id))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def print_limit_summary(defaults: dict[str, float], joint_limits: dict[str, dict[str, float]]) -> None:
    print("[pybullet-limit-editor] defaults:")
    print(f"  effort={defaults['effort']:.6g}, velocity={defaults['velocity']:.6g}")
    print("[pybullet-limit-editor] joint limits:")
    for joint_name in sorted(joint_limits):
        lower = float(joint_limits[joint_name]["lower"])
        upper = float(joint_limits[joint_name]["upper"])
        print(
            f"  {joint_name}: "
            f"lower={lower:.6f} rad ({rad_to_deg(lower):.2f} deg), "
            f"upper={upper:.6f} rad ({rad_to_deg(upper):.2f} deg)"
        )


def run_editor(args: argparse.Namespace, pb: Any, pybullet_data: Any) -> None:
    urdf_path = args.urdf.expanduser().resolve()
    limits_path = args.limits_file.expanduser().resolve()

    defaults, joint_limits = load_or_seed_limits(urdf_path, limits_path)
    urdf_joints = collect_urdf_joints(urdf_path)

    temp_dir, pybullet_urdf = materialize_pybullet_urdf(urdf_path)
    with temp_dir:
        client = pb.connect(pb.GUI)
        if client < 0:
            raise SystemExit("Failed to connect to PyBullet GUI.")

        pb.setAdditionalSearchPath(pybullet_data.getDataPath())
        pb.configureDebugVisualizer(pb.COV_ENABLE_GUI, 1)
        pb.resetDebugVisualizerCamera(
            cameraDistance=args.camera_distance,
            cameraYaw=args.camera_yaw,
            cameraPitch=args.camera_pitch,
            cameraTargetPosition=[0.0, 0.0, 0.15],
        )
        pb.setGravity(0, 0, -9.81)
        pb.loadURDF("plane.urdf")
        robot_id = pb.loadURDF(str(pybullet_urdf), useFixedBase=True)

        joint_ui: dict[str, dict[str, int | float]] = {}
        pybullet_joint_indices: dict[str, int] = {}
        for joint_index in range(pb.getNumJoints(robot_id)):
            joint_info = pb.getJointInfo(robot_id, joint_index)
            joint_name = joint_info[1].decode("utf-8")
            joint_type = int(joint_info[2])
            if joint_type not in PYBULLET_MOVABLE_TYPES:
                continue
            pybullet_joint_indices[joint_name] = joint_index

        missing = sorted(set(urdf_joints) - set(pybullet_joint_indices))
        if missing:
            raise SystemExit(f"PyBullet did not expose expected movable joints: {missing}")

        for joint_name in sorted(urdf_joints):
            initial_lower = rad_to_deg(joint_limits[joint_name]["lower"])
            initial_upper = rad_to_deg(joint_limits[joint_name]["upper"])
            lower_id = pb.addUserDebugParameter(
                f"{joint_name} lower (deg)",
                args.slider_min_deg,
                args.slider_max_deg,
                initial_lower,
            )
            upper_id = pb.addUserDebugParameter(
                f"{joint_name} upper (deg)",
                args.slider_min_deg,
                args.slider_max_deg,
                initial_upper,
            )
            pose_id = pb.addUserDebugParameter(
                f"{joint_name} pose (deg)",
                args.slider_min_deg,
                args.slider_max_deg,
                clamp(0.0, min(initial_lower, initial_upper), max(initial_lower, initial_upper)),
            )
            joint_ui[joint_name] = {
                "lower_id": lower_id,
                "upper_id": upper_id,
                "pose_id": pose_id,
            }

        effort_id = pb.addUserDebugParameter("default effort", 0.01, 100.0, float(defaults["effort"]))
        velocity_id = pb.addUserDebugParameter("default velocity", 0.01, 100.0, float(defaults["velocity"]))
        save_json_button = add_button(pb, "Save JSON")
        apply_urdf_button = add_button(pb, "Save JSON + Apply URDF")
        button_state = {
            save_json_button: read_button(pb, save_json_button),
            apply_urdf_button: read_button(pb, apply_urdf_button),
        }

        print("[pybullet-limit-editor] Started.")
        print("[pybullet-limit-editor] Adjust lower/upper/pose sliders in the PyBullet UI.")
        print("[pybullet-limit-editor] Use 'Save JSON' or 'Save JSON + Apply URDF' buttons in the UI.")

        while pb.isConnected():
            current_defaults = {
                "effort": float(pb.readUserDebugParameter(effort_id)),
                "velocity": float(pb.readUserDebugParameter(velocity_id)),
            }
            current_limits: dict[str, dict[str, float]] = {}

            for joint_name in sorted(joint_ui):
                lower_deg = float(pb.readUserDebugParameter(int(joint_ui[joint_name]["lower_id"])))
                upper_deg = float(pb.readUserDebugParameter(int(joint_ui[joint_name]["upper_id"])))
                pose_deg = float(pb.readUserDebugParameter(int(joint_ui[joint_name]["pose_id"])))

                effective_lower_deg = min(lower_deg, upper_deg)
                effective_upper_deg = max(lower_deg, upper_deg)
                effective_pose_deg = clamp(pose_deg, effective_lower_deg, effective_upper_deg)
                effective_pose_rad = deg_to_rad(effective_pose_deg)

                pb.resetJointState(robot_id, pybullet_joint_indices[joint_name], targetValue=effective_pose_rad)

                current_limits[joint_name] = {
                    "lower": deg_to_rad(lower_deg),
                    "upper": deg_to_rad(upper_deg),
                }

            for button_id in list(button_state):
                current_value = read_button(pb, button_id)
                if current_value == button_state[button_id]:
                    continue
                button_state[button_id] = current_value

                try:
                    for joint_name, limits in current_limits.items():
                        if float(limits["lower"]) > float(limits["upper"]):
                            raise ValueError(
                                f"{joint_name}: lower limit is greater than upper limit. "
                                "Adjust the sliders before saving."
                            )

                    save_limits_file(limits_path, current_defaults, current_limits)
                    print_limit_summary(current_defaults, current_limits)
                    print(f"[pybullet-limit-editor] Saved {limits_path}")

                    if button_id == apply_urdf_button:
                        apply_limits_to_urdf(urdf_path, current_defaults, current_limits)
                        print(f"[pybullet-limit-editor] Updated {urdf_path}")
                except Exception as exc:
                    print(f"[pybullet-limit-editor] Save failed: {exc}")

            pb.stepSimulation()
            time.sleep(1.0 / 120.0)


def main() -> None:
    args = build_parser().parse_args()
    pb, pybullet_data = import_pybullet()
    run_editor(args, pb, pybullet_data)


if __name__ == "__main__":
    main()
