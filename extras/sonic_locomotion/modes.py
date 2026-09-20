"""LocomotionMode values for the SONIC kinematic planner."""

from __future__ import annotations

from enum import IntEnum


class LocomotionMode(IntEnum):
    IDLE = 0
    SLOW_WALK = 1
    WALK = 2
    RUN = 3


SPEED_LIMITS: dict[LocomotionMode, tuple[float, float]] = {
    LocomotionMode.SLOW_WALK: (0.2, 0.8),
    LocomotionMode.WALK: (0.8, 2.5),
    LocomotionMode.RUN: (1.5, 3.0),
}


def mode_for_speed(speed: float) -> LocomotionMode:
    """Pick a standing gait from linear speed magnitude (m/s)."""
    if speed < 0.05:
        return LocomotionMode.IDLE
    if speed < 0.8:
        return LocomotionMode.SLOW_WALK
    if speed < 2.5:
        return LocomotionMode.WALK
    return LocomotionMode.RUN


def clamp_speed(mode: LocomotionMode, speed: float) -> float:
    if speed <= 0.0:
        return -1.0
    lo, hi = SPEED_LIMITS.get(mode, (0.2, 1.0))
    return max(lo, min(hi, speed))
