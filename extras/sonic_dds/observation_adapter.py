"""Build GEAR-style obs dict for UnitreeSdk2Bridge.PublishLowState."""

from __future__ import annotations

from typing import Any

import mujoco
import numpy as np
from numpy.typing import NDArray

from g1_simulacrum.model.joints import BODY_JOINT_NAMES, HAND_JOINT_NAMES, NUM_BODY_JOINTS
from g1_simulacrum.model.loader import CompiledModel


def _torso_body_id(model: mujoco.MjModel) -> int:
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    if bid < 0:
        raise ValueError("missing body torso_link")
    return bid


def _secondary_imu_vel(model: mujoco.MjModel, data: mujoco.MjData, torso_id: int) -> NDArray[np.float64]:
    pose = np.zeros(6, dtype=np.float64)
    mujoco.mj_objectVelocity(
        model, data, mujoco.mjtObj.mjOBJ_BODY, torso_id, pose, 1
    )
    lin_vel = pose[3:6].copy()
    ang_vel = pose[0:3].copy()
    return np.concatenate([lin_vel, ang_vel])


def _hand_q_dq(
    compiled: CompiledModel,
    data: mujoco.MjData,
    names: tuple[str, ...],
) -> tuple[NDArray[np.float64], NDArray[np.float64]]:
    q = np.zeros(len(names), dtype=np.float64)
    dq = np.zeros(len(names), dtype=np.float64)
    for i, name in enumerate(names):
        q[i] = data.qpos[compiled.hand_qposadr[name]]
        dq[i] = data.qvel[compiled.hand_dofadr[name]]
    return q, dq


def _hand_names_for_side(side: str) -> tuple[str, ...]:
    prefix = f"{side}_hand_"
    names = tuple(n for n in HAND_JOINT_NAMES if n.startswith(prefix))
    if len(names) != 7:
        raise ValueError(f"expected 7 {side} hand joints, got {len(names)}")
    return names


def prepare_obs_dict(compiled: CompiledModel, data: mujoco.MjData) -> dict[str, Any]:
    """Mirror gear_sonic DefaultEnv.prepare_obs() using named g1-simulacrum maps."""
    torso_id = _torso_body_id(compiled.model)

    body_q = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
    body_dq = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
    body_ddq = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)
    body_tau_est = np.zeros(NUM_BODY_JOINTS, dtype=np.float64)

    for i, name in enumerate(BODY_JOINT_NAMES):
        body_q[i] = data.qpos[compiled.body_qposadr[name]]
        body_dq[i] = data.qvel[compiled.body_dofadr[name]]
        body_ddq[i] = data.qacc[compiled.body_dofadr[name]]
        body_tau_est[i] = data.actuator_force[compiled.body_actuator_ids[name]]

    if compiled.hand_joint_ids:
        left_names = _hand_names_for_side("left")
        right_names = _hand_names_for_side("right")
        left_hand_q, left_hand_dq = _hand_q_dq(compiled, data, left_names)
        right_hand_q, right_hand_dq = _hand_q_dq(compiled, data, right_names)
    else:
        left_hand_q = np.zeros(7, dtype=np.float64)
        left_hand_dq = np.zeros(7, dtype=np.float64)
        right_hand_q = np.zeros(7, dtype=np.float64)
        right_hand_dq = np.zeros(7, dtype=np.float64)

    return {
        "floating_base_pose": data.qpos[:7].copy(),
        "floating_base_vel": data.qvel[:6].copy(),
        "floating_base_acc": data.qacc[:6].copy(),
        "secondary_imu_quat": data.xquat[torso_id].copy(),
        "secondary_imu_vel": _secondary_imu_vel(compiled.model, data, torso_id),
        "body_q": body_q,
        "body_dq": body_dq,
        "body_ddq": body_ddq,
        "body_tau_est": body_tau_est,
        "left_hand_q": left_hand_q,
        "left_hand_dq": left_hand_dq,
        "right_hand_q": right_hand_q,
        "right_hand_dq": right_hand_dq,
        "time": float(data.time),
    }
