#!/usr/bin/env python3
"""Read, edit, and apply URDF joint limits for the current arm models."""

from __future__ import annotations

import argparse
import json
import math
import xml.etree.ElementTree as ET
from pathlib import Path


MOVABLE_JOINT_TYPES = {"revolute", "prismatic"}
DEFAULT_LIMIT_RAD = math.pi
RAD_PER_DEG = math.pi / 180.0
DEG_PER_RAD = 180.0 / math.pi


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Update movable joint limits for a URDF.")
    parser.add_argument("urdf", type=Path, help="Path to the URDF file.")

    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument(
        "--limits-file",
        type=Path,
        help="Load limits from a JSON sidecar and apply them to the URDF.",
    )
    mode_group.add_argument(
        "--write-template",
        type=Path,
        help="Write a JSON sidecar seeded from the URDF's current joint limits.",
    )

    parser.add_argument(
        "--output-urdf",
        type=Path,
        default=None,
        help="Optional output URDF path. Defaults to patching the source URDF in place.",
    )
    parser.add_argument(
        "--set-rad",
        action="append",
        default=[],
        metavar="JOINT=LOWER,UPPER",
        help="Override one joint limit pair in radians. Repeatable.",
    )
    parser.add_argument(
        "--set-deg",
        action="append",
        default=[],
        metavar="JOINT=LOWER,UPPER",
        help="Override one joint limit pair in degrees. Repeatable.",
    )
    parser.add_argument(
        "--effort",
        type=float,
        default=None,
        help="Override the default effort written into the JSON or URDF.",
    )
    parser.add_argument(
        "--velocity",
        type=float,
        default=None,
        help="Override the default velocity written into the JSON or URDF.",
    )
    return parser


def parse_optional_float(raw: str | None) -> float | None:
    if raw is None:
        return None
    text = raw.strip()
    if not text:
        return None
    return float(text)


def format_number(value: float) -> str:
    return format(value, ".10g")


def rad_to_deg(value: float) -> float:
    return value * DEG_PER_RAD


def deg_to_rad(value: float) -> float:
    return value * RAD_PER_DEG


def collect_urdf_limits(urdf_path: Path) -> tuple[dict[str, float], dict[str, dict[str, float]]]:
    root = ET.parse(urdf_path).getroot()
    defaults = {"effort": 1.0, "velocity": 1.0}
    joint_limits: dict[str, dict[str, float]] = {}

    for joint in root.findall("joint"):
        joint_name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "")
        if joint_type not in MOVABLE_JOINT_TYPES:
            continue

        limit_elem = joint.find("limit")
        lower = -DEFAULT_LIMIT_RAD
        upper = DEFAULT_LIMIT_RAD
        if limit_elem is not None:
            lower = float(parse_optional_float(limit_elem.attrib.get("lower")) or -DEFAULT_LIMIT_RAD)
            upper = float(parse_optional_float(limit_elem.attrib.get("upper")) or DEFAULT_LIMIT_RAD)
            effort = parse_optional_float(limit_elem.attrib.get("effort"))
            velocity = parse_optional_float(limit_elem.attrib.get("velocity"))
            if effort is not None:
                defaults["effort"] = float(effort)
            if velocity is not None:
                defaults["velocity"] = float(velocity)

        joint_limits[joint_name] = {"lower": float(lower), "upper": float(upper)}

    if not joint_limits:
        raise ValueError(f"{urdf_path}: no movable joints with limits were found.")
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
        joint_limits[str(joint_name)] = {
            "lower": float(lower),
            "upper": float(upper),
        }
    if not joint_limits:
        raise ValueError(f"{path}: 'joint_limits' is empty.")
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


def parse_joint_override(raw: str, *, unit: str) -> tuple[str, float, float]:
    if "=" not in raw:
        raise ValueError(f"Expected override as JOINT=LOWER,UPPER, got '{raw}'.")
    joint_name, pair = raw.split("=", 1)
    values = [piece.strip() for piece in pair.split(",")]
    if len(values) != 2:
        raise ValueError(f"Expected two values for '{raw}', got '{pair}'.")
    lower = float(values[0])
    upper = float(values[1])
    if unit == "deg":
        lower = deg_to_rad(lower)
        upper = deg_to_rad(upper)
    return joint_name.strip(), lower, upper


def apply_overrides(
    joint_limits: dict[str, dict[str, float]],
    *,
    set_rad: list[str],
    set_deg: list[str],
) -> None:
    for raw in set_rad:
        joint_name, lower, upper = parse_joint_override(raw, unit="rad")
        joint_limits[joint_name] = {"lower": float(lower), "upper": float(upper)}
    for raw in set_deg:
        joint_name, lower, upper = parse_joint_override(raw, unit="deg")
        joint_limits[joint_name] = {"lower": float(lower), "upper": float(upper)}


def validate_joint_limits(
    urdf_joint_names: set[str],
    joint_limits: dict[str, dict[str, float]],
) -> None:
    unknown = sorted(set(joint_limits) - urdf_joint_names)
    if unknown:
        raise ValueError(f"Unknown joints in limits payload: {unknown}")

    for joint_name, limits in joint_limits.items():
        lower = float(limits["lower"])
        upper = float(limits["upper"])
        if lower > upper:
            raise ValueError(f"{joint_name}: lower limit {lower} is greater than upper limit {upper}.")


def apply_limits_to_urdf(
    source_urdf: Path,
    output_urdf: Path,
    defaults: dict[str, float],
    joint_limits: dict[str, dict[str, float]],
) -> None:
    tree = ET.parse(source_urdf)
    root = tree.getroot()
    validate_joint_limits(
        {joint.attrib.get("name", "") for joint in root.findall("joint")},
        joint_limits,
    )

    for joint in root.findall("joint"):
        joint_name = joint.attrib.get("name", "")
        joint_type = joint.attrib.get("type", "")
        if joint_type not in MOVABLE_JOINT_TYPES or joint_name not in joint_limits:
            continue

        limits = joint_limits[joint_name]
        limit_elem = joint.find("limit")
        if limit_elem is None:
            limit_elem = ET.SubElement(joint, "limit")
        limit_elem.set("lower", format_number(float(limits["lower"])))
        limit_elem.set("upper", format_number(float(limits["upper"])))
        limit_elem.set("effort", format_number(float(defaults["effort"])))
        limit_elem.set("velocity", format_number(float(defaults["velocity"])))

    ET.indent(tree, space="  ")
    output_urdf.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_urdf, encoding="utf-8", xml_declaration=True)


def print_limit_summary(defaults: dict[str, float], joint_limits: dict[str, dict[str, float]], *, header: str) -> None:
    print(header)
    print(f"defaults: effort={defaults['effort']:.6g}, velocity={defaults['velocity']:.6g}")
    for joint_name in sorted(joint_limits):
        lower = float(joint_limits[joint_name]["lower"])
        upper = float(joint_limits[joint_name]["upper"])
        print(
            f"{joint_name}: "
            f"lower={lower:.6f} rad ({rad_to_deg(lower):.2f} deg), "
            f"upper={upper:.6f} rad ({rad_to_deg(upper):.2f} deg)"
        )


def main() -> None:
    args = build_parser().parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    source_defaults, source_joint_limits = collect_urdf_limits(urdf_path)
    urdf_joint_names = set(source_joint_limits)

    if args.write_template is not None:
        defaults = dict(source_defaults)
        joint_limits = {joint_name: dict(limits) for joint_name, limits in source_joint_limits.items()}
        apply_overrides(joint_limits, set_rad=args.set_rad, set_deg=args.set_deg)
        if args.effort is not None:
            defaults["effort"] = float(args.effort)
        if args.velocity is not None:
            defaults["velocity"] = float(args.velocity)
        validate_joint_limits(urdf_joint_names, joint_limits)

        output_path = args.write_template.expanduser().resolve()
        save_limits_file(output_path, defaults, joint_limits)
        print_limit_summary(defaults, joint_limits, header=f"Wrote limits template: {output_path}")
        return

    assert args.limits_file is not None
    defaults, joint_limits = load_limits_file(args.limits_file.expanduser().resolve())
    apply_overrides(joint_limits, set_rad=args.set_rad, set_deg=args.set_deg)
    if args.effort is not None:
        defaults["effort"] = float(args.effort)
    if args.velocity is not None:
        defaults["velocity"] = float(args.velocity)
    validate_joint_limits(urdf_joint_names, joint_limits)

    output_urdf = args.output_urdf.expanduser().resolve() if args.output_urdf is not None else urdf_path
    apply_limits_to_urdf(urdf_path, output_urdf, defaults, joint_limits)
    print_limit_summary(defaults, joint_limits, header=f"Updated URDF limits: {output_urdf}")


if __name__ == "__main__":
    main()
