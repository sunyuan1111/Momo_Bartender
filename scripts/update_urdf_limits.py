#!/usr/bin/env python3

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


MOVABLE_JOINT_TYPES = {"revolute", "prismatic"}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Update URDF joint limits from a JSON file.",
    )
    parser.add_argument(
        "urdf",
        type=Path,
        help="Path to the URDF file to inspect or update.",
    )
    parser.add_argument(
        "--limits-file",
        type=Path,
        help="JSON file containing per-joint limits.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional output path. Defaults to overwriting the input URDF.",
    )
    parser.add_argument(
        "--write-template",
        type=Path,
        help="Write a JSON template extracted from the URDF and exit unless --limits-file is also provided.",
    )
    return parser


def parse_urdf(path: Path) -> ET.ElementTree:
    parser = ET.XMLParser(target=ET.TreeBuilder(insert_comments=True))
    return ET.parse(path, parser=parser)


def collect_movable_joints(root: ET.Element) -> dict[str, ET.Element]:
    joints: dict[str, ET.Element] = {}
    for joint in root.findall("joint"):
        joint_type = joint.attrib.get("type")
        if joint_type not in MOVABLE_JOINT_TYPES:
            continue
        name = joint.attrib.get("name")
        if not name:
            raise ValueError("Found a movable joint without a name.")
        joints[name] = joint
    return joints


def parse_optional_number(raw: str | None) -> float | None:
    if raw is None:
        return None
    return float(raw)


def format_number(value: float) -> str:
    return format(value, ".10g")


def write_template(urdf_path: Path, output_path: Path) -> None:
    tree = parse_urdf(urdf_path)
    joints = collect_movable_joints(tree.getroot())

    payload: dict[str, Any] = {
        "defaults": {
            "effort": 1.0,
            "velocity": 1.0,
        },
        "joint_limits": {},
    }

    for joint_name, joint in joints.items():
        limit = joint.find("limit")
        payload["joint_limits"][joint_name] = {
            "lower": parse_optional_number(limit.attrib.get("lower")) if limit is not None else None,
            "upper": parse_optional_number(limit.attrib.get("upper")) if limit is not None else None,
        }

    output_path.write_text(json.dumps(payload, indent=2) + "\n", encoding="utf-8")


def load_limits(path: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"{path}: limits file must contain a JSON object.")

    defaults = payload.get("defaults", {})
    if not isinstance(defaults, dict):
        raise ValueError(f"{path}: 'defaults' must be a JSON object when present.")

    joint_limits = payload.get("joint_limits", payload)
    if not isinstance(joint_limits, dict):
        raise ValueError(f"{path}: 'joint_limits' must be a JSON object.")

    normalized: dict[str, dict[str, Any]] = {}
    for joint_name, joint_payload in joint_limits.items():
        if not isinstance(joint_name, str):
            raise ValueError(f"{path}: joint names must be strings.")
        if not isinstance(joint_payload, dict):
            raise ValueError(f"{path}: joint '{joint_name}' must map to a JSON object.")
        normalized[joint_name] = dict(joint_payload)

    return defaults, normalized


def resolve_number(
    joint_name: str,
    field_name: str,
    joint_payload: dict[str, Any],
    defaults: dict[str, Any],
    existing_limit: ET.Element | None,
) -> float:
    value = joint_payload.get(field_name, defaults.get(field_name))
    if value is None and existing_limit is not None:
        raw_existing = existing_limit.attrib.get(field_name)
        if raw_existing is not None:
            value = float(raw_existing)
    if value is None:
        raise ValueError(
            f"Joint '{joint_name}' is missing '{field_name}'. "
            "Provide it in the limits file or defaults."
        )
    return float(value)


def insert_limit(joint: ET.Element, limit: ET.Element) -> None:
    children = list(joint)
    insert_after = None
    for index, child in enumerate(children):
        if child.tag in {"axis", "child", "parent", "origin"}:
            insert_after = index
    if insert_after is None:
        joint.append(limit)
        return
    joint.insert(insert_after + 1, limit)


def update_limits(urdf_path: Path, limits_path: Path, output_path: Path | None) -> None:
    defaults, joint_limits = load_limits(limits_path)
    tree = parse_urdf(urdf_path)
    root = tree.getroot()
    joints = collect_movable_joints(root)

    unknown_joints = sorted(set(joint_limits) - set(joints))
    if unknown_joints:
        raise ValueError(f"Unknown joints in {limits_path}: {unknown_joints}")

    for joint_name, joint_payload in joint_limits.items():
        joint = joints[joint_name]
        existing_limit = joint.find("limit")

        lower = resolve_number(joint_name, "lower", joint_payload, defaults, existing_limit)
        upper = resolve_number(joint_name, "upper", joint_payload, defaults, existing_limit)
        effort = resolve_number(joint_name, "effort", joint_payload, defaults, existing_limit)
        velocity = resolve_number(joint_name, "velocity", joint_payload, defaults, existing_limit)

        if lower > upper:
            raise ValueError(
                f"Joint '{joint_name}' has lower limit {lower} greater than upper limit {upper}."
            )

        if existing_limit is None:
            existing_limit = ET.Element("limit")
            insert_limit(joint, existing_limit)

        existing_limit.set("lower", format_number(lower))
        existing_limit.set("upper", format_number(upper))
        existing_limit.set("effort", format_number(effort))
        existing_limit.set("velocity", format_number(velocity))

    ET.indent(tree, space="  ")
    target_path = output_path or urdf_path
    tree.write(target_path, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.write_template is None and args.limits_file is None:
        parser.error("Provide at least one of --write-template or --limits-file.")

    if args.write_template is not None:
        write_template(args.urdf, args.write_template)
        print(args.write_template)
        if args.limits_file is None:
            return

    assert args.limits_file is not None
    update_limits(args.urdf, args.limits_file, args.output)
    print(args.output or args.urdf)


if __name__ == "__main__":
    main()
