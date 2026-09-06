"""Named-actuator PD for the 29 body joints."""

from __future__ import annotations

from ..model.joints import BODY_JOINT_NAMES
from .base import Controller
from .gains import G1_PD_GAINS


class PDController(Controller):
    def step(self, sim_time: float) -> None:
        del sim_time
        if self.body_passive:
            self.zero_body_ctrl()
            self.hold_hands()
            return
        q = self.body_qpos()
        dq = self.body_qvel()
        for i, name in enumerate(BODY_JOINT_NAMES):
            gains = G1_PD_GAINS[name]
            kp = gains["kp"] * self._config.kp_scale.get(name, 1.0)
            kd = gains["kd"] * self._config.kd_scale.get(name, 1.0)
            tau = kp * (self._q_target[i] - q[i]) - kd * dq[i]
            aid = self._compiled.body_actuator_ids[name]
            self._data.ctrl[aid] = tau
        self.hold_hands()
