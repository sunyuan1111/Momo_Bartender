from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[1]
SRC_ROOT = REPO_ROOT / "src"
DEFAULT_CONFIG = REPO_ROOT / "configs" / "arm7_sts3215.example.json"
DEFAULT_PORT = "/dev/ttyACM0"

if str(SRC_ROOT) not in sys.path:
    sys.path.insert(0, str(SRC_ROOT))

from physical_agent.config import ArmConfig
from physical_agent.controller import Sts3215ArmController


def add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--port", default=DEFAULT_PORT)


def make_controller(config_path: Path, port: str) -> Sts3215ArmController:
    config = ArmConfig.from_json(config_path)
    return Sts3215ArmController(replace(config, port=port))


def print_json(payload: object) -> None:
    print(json.dumps(payload, indent=2, sort_keys=True))
