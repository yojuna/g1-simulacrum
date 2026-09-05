"""GEAR-SONIC whole-body controller bridge.

Communicates with the ``gear_sonic_deploy`` process over DDS (Unitree SDK2),
exactly matching the real hardware deployment architecture:

    g1-simulacrum (MuJoCo 200Hz)  ◄──── rt/lowcmd ────  gear_sonic_deploy (50Hz)
                                 ────► rt/lowstate ───►
                                 ────► rt/odostate ───►
                                 ────► rt/secondary_imu►

The SONIC policy doesn't know whether it's talking to a real G1 or to MuJoCo.
"""

from __future__ import annotations

import logging
from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..config import ControllerConfig
from .base import Controller
from .gains import G1_PD_GAINS

logger = logging.getLogger(__name__)

# Try importing Unitree SDK2 — it's optional
try:
    from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
    from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_
    HAS_SDK2 = True
except ImportError:
    HAS_SDK2 = False
    LowCmd_ = Any
    LowState_ = Any


class SonicBridge(Controller):
    """Bridge between MuJoCo simulation and GEAR-SONIC deployment.

    Publishes robot state over DDS at ``control_hz`` and applies incoming
    joint position commands via PD control at ``control_hz``.

    Architecture matches ``gear_sonic/scripts/run_sim_loop.py`` from the
    official GR00T-WholeBodyControl repo.
    """

    NUM_JOINTS = 29

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: ControllerConfig,
    ) -> None:
        super().__init__(model, data, config)

        if not HAS_SDK2:
            raise ImportError(
                "unitree_sdk2py is required for SONIC bridge. "
                "Install with: pip install 'g1-simulacrum[sonic]'"
            )

        self._control_dt = 1.0 / config.control_hz
        self._physics_dt = 1.0 / config.physics_hz
        self._substeps = int(config.control_hz / config.physics_hz)

        # PD gains from SONIC config
        self._kp = np.array([G1_PD_GAINS[i]["kp"] for i in range(self.NUM_JOINTS)])
        self._kd = np.array([G1_PD_GAINS[i]["kd"] for i in range(self.NUM_JOINTS)])

        # Apply gain scaling from config
        for joint_idx, scale in config.sonic.motor_kp_scale.items():
            self._kp[joint_idx] *= scale
        for joint_idx, scale in config.sonic.motor_kd_scale.items():
            self._kd[joint_idx] *= scale

        # Target joint positions (updated by DDS subscriber)
        self._q_target = np.zeros(self.NUM_JOINTS)
        self._last_cmd_time: float = 0.0

        # Joint index mapping: MuJoCo qpos/qvel indices for the 29 actuated joints
        # (first 7 are freejoint: 3 pos + 4 quat for qpos, 6 for qvel)
        self._qpos_indices = np.arange(7, 7 + self.NUM_JOINTS)
        self._qvel_indices = np.arange(6, 6 + self.NUM_JOINTS)

        # DDS channels
        self._setup_dds(config.sonic.dds_domain)

    def _setup_dds(self, domain_id: int) -> None:
        """Initialize DDS publishers and subscribers."""
        # Publisher: robot state → SONIC
        self._state_pub = ChannelPublisher("rt/lowstate", LowState_)
        self._state_pub.Init()

        # Subscriber: SONIC → joint commands
        self._cmd_sub = ChannelSubscriber("rt/lowcmd", LowCmd_)
        self._cmd_sub.Init(self._on_low_cmd)

        logger.info("SONIC DDS bridge initialized on domain %d", domain_id)

    def _on_low_cmd(self, msg: LowCmd_) -> None:
        """DDS callback: receive joint position targets from SONIC."""
        for i in range(self.NUM_JOINTS):
            self._q_target[i] = msg.motor_cmd[i].q
        self._last_cmd_time = self._data.time

    def compute_torques(self) -> NDArray[np.float64]:
        """Compute PD torques from current state and SONIC targets.

        τ = Kp * (q_target - q) + Kd * (0 - q̇)
        """
        q = self._data.qpos[self._qpos_indices]
        qd = self._data.qvel[self._qvel_indices]

        torques = self._kp * (self._q_target - q) - self._kd * qd
        return torques

    def step(self, sim_time: float) -> None:
        """Run one control step: publish state, apply PD torques."""
        # Publish current state for SONIC to read
        self._publish_state(sim_time)

        # Compute and apply PD control
        torques = self.compute_torques()
        self._data.ctrl[:self.NUM_JOINTS] = torques

    def _publish_state(self, sim_time: float) -> None:
        """Pack current MuJoCo state into a LowState_ message and publish."""
        msg = LowState_()

        q = self._data.qpos[self._qpos_indices]
        qd = self._data.qvel[self._qvel_indices]
        tau = self._data.actuator_force[:self.NUM_JOINTS]

        for i in range(self.NUM_JOINTS):
            msg.motor_state[i].q = float(q[i])
            msg.motor_state[i].dq = float(qd[i])
            msg.motor_state[i].tau_est = float(tau[i])

        # IMU data (from the base body's accelerometer/gyro)
        # Gravity projection in body frame
        base_quat = self._data.qpos[3:7]  # freejoint quaternion
        gravity_world = np.array([0, 0, -9.81])
        gravity_body = self._quat_rotate_inverse(base_quat, gravity_world)
        msg.imu_state.accelerometer = gravity_body.tolist()

        base_omega = self._data.qvel[3:6]  # freejoint angular velocity
        msg.imu_state.gyroscope = base_omega.tolist()
        msg.imu_state.quaternion = base_quat.tolist()

        self._state_pub.Write(msg)

    @staticmethod
    def _quat_rotate_inverse(q: NDArray, v: NDArray) -> NDArray:
        """Rotate vector v by the inverse of quaternion q (w,x,y,z)."""
        w, x, y, z = q
        # Conjugate quaternion rotation: q* v q
        q_conj = np.array([w, -x, -y, -z])
        # Use quaternion multiplication to rotate
        t = 2.0 * np.cross(q_conj[1:], v)
        return v + q_conj[0] * t + np.cross(q_conj[1:], t)

    def reset(self) -> None:
        """Reset targets to standing pose."""
        self._q_target = np.zeros(self.NUM_JOINTS)
        self._last_cmd_time = 0.0

    def set_targets(self, q_target: NDArray[np.float64]) -> None:
        """Directly set joint position targets (bypassing DDS)."""
        assert len(q_target) == self.NUM_JOINTS
        self._q_target = q_target.copy()
