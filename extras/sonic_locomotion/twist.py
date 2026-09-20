"""Convert ``cmd_vel``-style twists into SONIC planner fields."""

from __future__ import annotations

import math
from dataclasses import dataclass

from .modes import LocomotionMode, clamp_speed, mode_for_speed


@dataclass(frozen=True)
class PlannerCommand:
    mode: int
    movement: tuple[float, float, float]
    facing: tuple[float, float, float]
    speed: float
    height: float = -1.0


@dataclass
class TwistIntegrator:
    """Integrate angular velocity and hold the last movement command."""

    facing_angle: float = 0.0
    mode: LocomotionMode = LocomotionMode.IDLE

    def update(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float,
        *,
        dt: float = 0.02,
        mode_override: LocomotionMode | None = None,
    ) -> PlannerCommand:
        self.facing_angle += angular_z * dt
        self.facing_angle = math.atan2(
            math.sin(self.facing_angle),
            math.cos(self.facing_angle),
        )

        speed = math.hypot(linear_x, linear_y)
        if speed > 1e-6:
            movement = (linear_x / speed, linear_y / speed, 0.0)
            mode = mode_override or mode_for_speed(speed)
            self.mode = mode
            planner_speed = clamp_speed(mode, speed)
        else:
            movement = (0.0, 0.0, 0.0)
            mode = LocomotionMode.IDLE
            self.mode = mode
            planner_speed = -1.0

        facing = (
            math.cos(self.facing_angle),
            math.sin(self.facing_angle),
            0.0,
        )
        return PlannerCommand(
            mode=int(mode),
            movement=movement,
            facing=facing,
            speed=planner_speed,
        )

    def idle(self) -> PlannerCommand:
        self.mode = LocomotionMode.IDLE
        return PlannerCommand(
            mode=int(LocomotionMode.IDLE),
            movement=(0.0, 0.0, 0.0),
            facing=(
                math.cos(self.facing_angle),
                math.sin(self.facing_angle),
                0.0,
            ),
            speed=-1.0,
        )
