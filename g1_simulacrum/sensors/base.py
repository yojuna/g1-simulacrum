"""Abstract base class for all sensors."""

from __future__ import annotations

from abc import ABC, abstractmethod
from typing import Any

import mujoco


class Sensor(ABC):
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
        return (sim_time - self._last_read_time) >= self._dt - 1e-9

    @abstractmethod
    def read(self, sim_time: float) -> Any:
        ...

    def step(self, sim_time: float) -> Any | None:
        if self.should_read(sim_time):
            self._last_read_time = sim_time
            return self.read(sim_time)
        return None

    def reset(self) -> None:
        self._last_read_time = -float("inf")
