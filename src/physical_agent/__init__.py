from __future__ import annotations

from importlib import import_module
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .config import ArmConfig, JointConfig
    from .controller import Sts3215ArmController
    from .kinematics import UrdfArmKinematics


__all__ = ["ArmConfig", "JointConfig", "Sts3215ArmController", "UrdfArmKinematics"]


def __getattr__(name: str):
    if name in {"ArmConfig", "JointConfig"}:
        module = import_module(".config", __name__)
        return getattr(module, name)
    if name == "Sts3215ArmController":
        module = import_module(".controller", __name__)
        return getattr(module, name)
    if name == "UrdfArmKinematics":
        module = import_module(".kinematics", __name__)
        return getattr(module, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
