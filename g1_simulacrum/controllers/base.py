"""Abstract controller interface."""

from __future__ import annotations

from abc import ABC, abstractmethod

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..config import ControllerConfig


class Controller(ABC):
    """Base interface for G1 whole-body controllers."""

    NUM_JOINTS = 29

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: ControllerConfig,
    ) -> None:
        self._model = model
        self._data = data
        self._config = config

    @abstractmethod
    def step(self, sim_time: float) -> None:
        """Run one control cycle and write to ``data.ctrl``."""
        ...

    @abstractmethod
    def compute_torques(self) -> NDArray[np.float64]:
        """Compute actuator torques for the current state."""
        ...

    @abstractmethod
    def reset(self) -> None:
        """Reset controller state."""
        ...

    @abstractmethod
    def set_targets(self, q_target: NDArray[np.float64]) -> None:
        """Set desired joint positions (29,)."""
        ...
