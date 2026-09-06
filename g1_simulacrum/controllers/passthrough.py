"""Torque passthrough onto named body actuators."""

from __future__ import annotations

from ..model.joints import BODY_JOINT_NAMES
from .base import Controller


class PassthroughController(Controller):
    def step(self, sim_time: float) -> None:
        del sim_time
        if self.body_passive:
            self.zero_body_ctrl()
            self.hold_hands()
            return
        for i, name in enumerate(BODY_JOINT_NAMES):
            aid = self._compiled.body_actuator_ids[name]
            self._data.ctrl[aid] = self._q_target[i]
        self.hold_hands()
