#!/usr/bin/env python3
"""Apply a measured URDF joint-limit calibration to the current arm model."""

from __future__ import annotations

import argparse
import json
import shutil
from pathlib import Path
from typing import Any
import sys


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from update_urdf_limits import apply_limits_to_urdf, collect_urdf_limits, save_limits_file


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = REPO_ROOT / "models" / "arm2" / "urdf" / "arm2.urdf"
DEFAULT_CALIBRATION_JSON = REPO_ROOT / "runtime" / "urdf_limit_calibration.json"
DEFAULT_LIMITS_JSON = REPO_ROOT / "models" / "arm2" / "config" / "joint_limits.json"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=(
            "Apply a calibration JSON produced by 0_urdf_limit_calibrate.py. "
            "By default this updates models/arm2/config/joint_limits.json and patches the URDF in place."
        )
    )
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="Source URDF path.")
    parser.add_argument(
        "--calibration-json",
        type=Path,
        default=DEFAULT_CALIBRATION_JSON,
        help="Calibration JSON produced by 0_urdf_limit_calibrate.py.",
    )
    parser.add_argument(
        "--limits-output",
        type=Path,
        default=DEFAULT_LIMITS_JSON,
        help="Where to write the sidecar joint_limits.json file.",
    )
    parser.add_argument(
        "--output-urdf",
        type=Path,
        default=None,
        help="Optional output URDF path. Defaults to patching the source URDF in place.",
    )
    parser.add_argument(
        "--no-backup",
        action="store_true",
        help="Do not create a .bak file before patching the source URDF in place.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Validate and print the calibration summary without writing files.",
    )
    return parser


def read_json(path: Path) -> dict[str, Any]:
    if not path.exists():
        raise FileNotFoundError(f"Calibration JSON not found: {path}")
    payload = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(payload, dict):
        raise ValueError(f"Expected a JSON object in {path}")
    return payload


def load_joint_limits_payload(calibration_json: Path, urdf_path: Path) -> tuple[dict[str, float], dict[str, dict[str, float]], dict[str, Any]]:
    payload = read_json(calibration_json)
    if payload.get("mode") != "joint_limit_calibration":
        raise ValueError(
            f"{calibration_json} is not a joint-limit calibration file. "
            f"Expected mode='joint_limit_calibration', got {payload.get('mode')!r}."
        )

    defaults_from_urdf, limits_from_urdf = collect_urdf_limits(urdf_path)
    embedded = payload.get("joint_limits_payload")
    if isinstance(embedded, dict):
        defaults_payload = embedded.get("defaults", {})
        joint_limits_payload = embedded.get("joint_limits", {})
        if isinstance(defaults_payload, dict) and isinstance(joint_limits_payload, dict) and joint_limits_payload:
            defaults = {
                "effort": float(defaults_payload.get("effort", defaults_from_urdf["effort"])),
                "velocity": float(defaults_payload.get("velocity", defaults_from_urdf["velocity"])),
            }
            joint_limits = {
                str(joint_name): {
                    "lower": float(dict(limits)["lower"]),
                    "upper": float(dict(limits)["upper"]),
                }
                for joint_name, limits in joint_limits_payload.items()
            }
            return defaults, joint_limits, payload

    joint_results = payload.get("joint_results")
    if not isinstance(joint_results, dict):
        raise ValueError(f"{calibration_json} does not contain a valid 'joint_results' object.")

    joint_limits = {joint_name: dict(limits) for joint_name, limits in limits_from_urdf.items()}
    for joint_name, entry in joint_results.items():
        if joint_name not in joint_limits or not isinstance(entry, dict):
            continue
        measured = entry.get("measured_limit_rad")
        if not isinstance(measured, dict):
            continue
        joint_limits[joint_name] = {
            "lower": float(measured["lower_rad"]),
            "upper": float(measured["upper_rad"]),
        }

    return defaults_from_urdf, joint_limits, payload


def print_summary(defaults: dict[str, float], joint_limits: dict[str, dict[str, float]], *, header: str) -> None:
    print(header)
    print(f"defaults: effort={defaults['effort']:.6g}, velocity={defaults['velocity']:.6g}")
    for joint_name in sorted(joint_limits):
        lower = float(joint_limits[joint_name]["lower"])
        upper = float(joint_limits[joint_name]["upper"])
        print(f"{joint_name}: lower={lower:.6f} rad, upper={upper:.6f} rad")


def main() -> None:
    args = build_parser().parse_args()
    urdf_path = args.urdf.expanduser().resolve()
    calibration_json = args.calibration_json.expanduser().resolve()
    limits_output = args.limits_output.expanduser().resolve()
    output_urdf = args.output_urdf.expanduser().resolve() if args.output_urdf is not None else urdf_path

    if not urdf_path.exists():
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    defaults, joint_limits, payload = load_joint_limits_payload(calibration_json, urdf_path)
    print_summary(defaults, joint_limits, header="[urdf-apply] Calibration summary")

    if args.dry_run:
        return

    save_limits_file(limits_output, defaults, joint_limits)

    backup_path: Path | None = None
    if output_urdf == urdf_path and not args.no_backup:
        backup_path = urdf_path.with_suffix(urdf_path.suffix + ".bak")
        shutil.copy2(urdf_path, backup_path)

    apply_limits_to_urdf(urdf_path, output_urdf, defaults, joint_limits)

    result = {
        "calibration_json": str(calibration_json),
        "limits_output": str(limits_output),
        "output_urdf": str(output_urdf),
        "backup_path": str(backup_path) if backup_path is not None else None,
        "applied_joint_count": len(joint_limits),
        "selected_joints": payload.get("selected_joints", []),
    }
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
