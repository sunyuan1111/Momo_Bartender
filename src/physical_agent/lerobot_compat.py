from __future__ import annotations

from typing import Any


_IMPORT_ERROR: Exception | None = None

try:
    from lerobot.motors.feetech.feetech import FeetechMotorsBus
    from lerobot.motors.motors_bus import Motor, MotorNormMode
except ModuleNotFoundError as exc:
    FeetechMotorsBus = None  # type: ignore[assignment]
    Motor = None  # type: ignore[assignment]
    MotorNormMode = None  # type: ignore[assignment]
    _IMPORT_ERROR = exc


def require_lerobot() -> tuple[Any, Any, Any]:
    if FeetechMotorsBus is None or Motor is None or MotorNormMode is None:
        raise ModuleNotFoundError(
            "LeRobot Feetech support is not installed. Run `pip install -e .` or "
            "`pip install \"lerobot[feetech]\"` before using this controller."
        ) from _IMPORT_ERROR
    return FeetechMotorsBus, Motor, MotorNormMode
