"""Abstract controller interface."""

from __future__ import annotations

from abc import ABC, abstractmethod

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..config import ControllerConfig
from ..model.joints import BODY_JOINT_NAMES, NUM_BODY_JOINTS
from ..model.loader import CompiledModel
from .gains import FINGER_HOLD_KD, FINGER_HOLD_KP


class Controller(ABC):
    """Body controller: writes named body actuators only, then holds Dex3."""

    NUM_JOINTS = NUM_BODY_JOINTS

    def __init__(
        self,
        compiled: CompiledModel,
        data: mujoco.MjData,
        config: ControllerConfig,
    ) -> None:
        self._model = compiled.model
        self._data = data
        self._config = config
        self._compiled = compiled
        self._q_target = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
        self._hand_q_hold = {
            name: float(data.qpos[adr])
            for name, adr in compiled.hand_qposadr.items()
        }

    def set_targets(self, q_target: NDArray[np.float64]) -> None:
        q = np.asarray(q_target, dtype=np.float64)
        if q.shape != (NUM_BODY_JOINTS,):
            raise ValueError(f"expected action shape ({NUM_BODY_JOINTS},), got {q.shape}")
        self._q_target = q.copy()

    def hold_hands(self) -> None:
        """PD-hold Dex3 named actuators at reset qpos. Not a HandController."""
        for name, aid in self._compiled.hand_actuator_ids.items():
            q = self._data.qpos[self._compiled.hand_qposadr[name]]
            dq = self._data.qvel[self._compiled.hand_dofadr[name]]
            q_des = self._hand_q_hold[name]
            self._data.ctrl[aid] = FINGER_HOLD_KP * (q_des - q) - FINGER_HOLD_KD * dq

    def reset(self) -> None:
        self._q_target = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
        self._hand_q_hold = {
            name: float(self._data.qpos[adr])
            for name, adr in self._compiled.hand_qposadr.items()
        }

    def body_qpos(self) -> NDArray[np.float64]:
        out = np.empty(NUM_BODY_JOINTS, dtype=np.float64)
        for i, name in enumerate(BODY_JOINT_NAMES):
            out[i] = self._data.qpos[self._compiled.body_qposadr[name]]
        return out

    def body_qvel(self) -> NDArray[np.float64]:
        out = np.empty(NUM_BODY_JOINTS, dtype=np.float64)
        for i, name in enumerate(BODY_JOINT_NAMES):
            out[i] = self._data.qvel[self._compiled.body_dofadr[name]]
        return out

    def body_tau(self) -> NDArray[np.float64]:
        out = np.empty(NUM_BODY_JOINTS, dtype=np.float64)
        for i, name in enumerate(BODY_JOINT_NAMES):
            aid = self._compiled.body_actuator_ids[name]
            out[i] = self._data.actuator_force[aid]
        return out

    def hand_qpos(self) -> NDArray[np.float64] | None:
        if not self._compiled.hand_qposadr:
            return None
        names = list(self._compiled.hand_qposadr)
        out = np.empty(len(names), dtype=np.float64)
        for i, name in enumerate(names):
            out[i] = self._data.qpos[self._compiled.hand_qposadr[name]]
        return out

    @abstractmethod
    def step(self, sim_time: float) -> None:
        """Write body ``data.ctrl`` then hold hands."""
        ...
