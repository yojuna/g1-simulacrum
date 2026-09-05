"""Abstract base class for all sensors."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import mujoco
import numpy as np


class Sensor(ABC):
    """Base interface every sensor in the stack implements.

    A sensor is attached to a MuJoCo model/data pair and produces readings
    at a configured rate. The simulation loop calls ``should_read`` each step
    and ``read`` only when a new sample is due.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        rate_hz: float,
        *,
        name: str = "",
    ) -> None:
        self._model = model
        self._data = data
        self._rate_hz = rate_hz
        self._dt = 1.0 / rate_hz
        self._last_read_time: float = -float("inf")
        self.name = name or self.__class__.__name__

    @property
    def rate_hz(self) -> float:
        return self._rate_hz

    def should_read(self, sim_time: float) -> bool:
        """Return True if enough sim time has elapsed for a new reading."""
        return (sim_time - self._last_read_time) >= self._dt - 1e-9

    @abstractmethod
    def read(self, sim_time: float) -> Any:
        """Capture a reading from the current simulation state.

        Implementations must return the appropriate dataclass from
        ``data_types`` (PointCloud, DepthFrame, ImuReading).
        """
        ...

    def step(self, sim_time: float) -> Any | None:
        """Convenience: read only if the sensor's rate demands it."""
        if self.should_read(sim_time):
            self._last_read_time = sim_time
            return self.read(sim_time)
        return None

    @abstractmethod
    def get_mount_xml(self) -> str:
        """Return MJCF XML snippet that defines this sensor's bodies/sites.

        Used by the model composer to inject sensor mount points into the
        robot model before compilation.
        """
        ...

    def reset(self) -> None:
        """Reset internal state (e.g. IMU bias accumulator)."""
        self._last_read_time = -float("inf")
