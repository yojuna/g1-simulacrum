"""LowCmd PD actuation — ported from gear_sonic base_sim compute_*_torques."""

from __future__ import annotations

from typing import Any, Protocol

import numpy as np
from numpy.typing import NDArray

from g1_simulacrum.model.joints import BODY_JOINT_NAMES, HAND_JOINT_NAMES, NUM_BODY_JOINTS
from g1_simulacrum.model.loader import CompiledModel

from .wbc_config import MOTOR_EFFORT_LIMIT_LIST


class _LowCmdBridge(Protocol):
    num_hand_motor: int
    low_cmd_received: bool
    left_hand_cmd_received: bool
    right_hand_cmd_received: bool
    low_cmd: Any
    left_hand_cmd: Any
    right_hand_cmd: Any

    def cmd_received(self) -> bool: ...


def _left_hand_names() -> tuple[str, ...]:
    return tuple(n for n in HAND_JOINT_NAMES if n.startswith("left_hand_"))


def _right_hand_names() -> tuple[str, ...]:
    return tuple(n for n in HAND_JOINT_NAMES if n.startswith("right_hand_"))


def compute_body_torques(
    compiled: CompiledModel,
    data,
    bridge: _LowCmdBridge,
) -> NDArray[np.float64]:
    """tau = tau_ff + kp*(q_des - q) + kd*(dq_des - dq) per SDK2 motor index."""
    torques = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
    if not bridge.cmd_received():
        return torques
    for i, name in enumerate(BODY_JOINT_NAMES):
        cmd = bridge.low_cmd.motor_cmd[i]
        q = data.qpos[compiled.body_qposadr[name]]
        dq = data.qvel[compiled.body_dofadr[name]]
        q_des = float(cmd.q)
        jid = compiled.body_joint_ids[name]
        if compiled.model.jnt_limited[jid]:
            lo, hi = compiled.model.jnt_range[jid]
            q_des = float(np.clip(q_des, lo, hi))
        torques[i] = (
            cmd.tau
            + cmd.kp * (q_des - q)
            + cmd.kd * (cmd.dq - dq)
        )
    return torques


def compute_hand_torques(
    compiled: CompiledModel,
    data,
    bridge: _LowCmdBridge,
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    """Dex3 finger PD from rt/dex3/*/cmd (7 motors per side, IDL order)."""
    n = bridge.num_hand_motor
    left = np.zeros(n, dtype=np.float64)
    right = np.zeros(n, dtype=np.float64)
    if n == 0 or not compiled.hand_actuator_ids:
        return left, right

    left_names = _left_hand_names()
    right_names = _right_hand_names()

    if bridge.left_hand_cmd_received:
        for i, name in enumerate(left_names):
            cmd = bridge.left_hand_cmd.motor_cmd[i]
            q = data.qpos[compiled.hand_qposadr[name]]
            dq = data.qvel[compiled.hand_dofadr[name]]
            left[i] = cmd.tau + cmd.kp * (cmd.q - q) + cmd.kd * (cmd.dq - dq)

    if bridge.right_hand_cmd_received:
        for i, name in enumerate(right_names):
            cmd = bridge.right_hand_cmd.motor_cmd[i]
            q = data.qpos[compiled.hand_qposadr[name]]
            dq = data.qvel[compiled.hand_dofadr[name]]
            right[i] = cmd.tau + cmd.kp * (cmd.q - q) + cmd.kd * (cmd.dq - dq)

    return left, right


def apply_torques(
    compiled: CompiledModel,
    data,
    body_torques: NDArray[np.float64],
    left_hand_torques: NDArray[np.float64],
    right_hand_torques: NDArray[np.float64],
    *,
    torque_limits: tuple[float, ...] | None = None,
) -> None:
    """Write clipped torques to named actuators."""
    limits = np.asarray(torque_limits or MOTOR_EFFORT_LIMIT_LIST[:NUM_BODY_JOINTS])

    for i, name in enumerate(BODY_JOINT_NAMES):
        tau = float(np.clip(body_torques[i], -limits[i], limits[i]))
        data.ctrl[compiled.body_actuator_ids[name]] = tau

    left_names = _left_hand_names()
    right_names = _right_hand_names()
    for i, name in enumerate(left_names):
        if name in compiled.hand_actuator_ids:
            data.ctrl[compiled.hand_actuator_ids[name]] = left_hand_torques[i]
    for i, name in enumerate(right_names):
        if name in compiled.hand_actuator_ids:
            data.ctrl[compiled.hand_actuator_ids[name]] = right_hand_torques[i]
