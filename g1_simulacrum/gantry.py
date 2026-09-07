"""Overhead crane: inspect cable, or GEAR-SONIC cartesian hold.

Two force models write ``xfrc_applied`` on one body:

- ``cable`` (inspect viewer): Unitree unilateral spring along hook→body.
  Hook Z is ``HOOK_Z`` (2 m); slack does not push into the floor.
- ``gear`` (SONIC DDS): GEAR ``ElasticBand.Advance`` — cartesian PD to
  ``(spawn_xy, 1)`` with ``kp_pos=10000``, ``kd_pos=1000``, plus attitude
  PD. ``length`` is a vertical offset (keys 7/8), not a cable rest length.

Attitude PD holds ``self.quat`` (spawn yaw, then numpad 7/9). The viewer
key callback must not write ``mjData`` (UI thread vs ``mj_step``).
"""

from __future__ import annotations

from typing import Literal

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation

# Unitree python band uses 200 / 100 with length starting at 0 (big initial yank).
# Slightly stiffer so a taut init holds a ~35 kg G1 near spawn height (mg/k ≈ 0.17 m).
_STIFFNESS = 2000.0
_DAMPING = 200.0
# gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py ElasticBand.
_GEAR_KP_POS = 10000.0
_GEAR_KD_POS = 1000.0
_GEAR_HOLD_Z = 1.0
# GEAR-SONIC ElasticBand attitude PD (identity lock); we track self.quat instead.
_KP_ANG = 1000.0
_KD_ANG = 10.0
HOOK_Z = 2.0
_G1_MASS_KG = 35.0

GantryMode = Literal["cable", "gear"]


def quat_wxyz_from_yaw(yaw_rad: float) -> NDArray[np.float64]:
    """Heading about world +Z. 0 faces +X (MuJoCo identity)."""
    half = 0.5 * float(yaw_rad)
    return np.array([np.cos(half), 0.0, 0.0, np.sin(half)], dtype=np.float64)


def yaw_from_quat_wxyz(quat_wxyz: NDArray[np.float64] | tuple[float, ...]) -> float:
    q = np.asarray(quat_wxyz, dtype=np.float64)
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    return float(rot.as_euler("xyz")[2])


def apply_world_yaw(quat_wxyz: NDArray[np.float64], delta_rad: float) -> NDArray[np.float64]:
    """Rotate a wxyz quaternion about world +Z (does not flatten roll/pitch)."""
    q = np.asarray(quat_wxyz, dtype=np.float64)
    current = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    extra = Rotation.from_euler("z", float(delta_rad))
    out = extra * current
    x, y, z, w = out.as_quat()
    return np.array([w, x, y, z], dtype=np.float64)


def _rot_wxyz(quat_wxyz: NDArray[np.float64] | tuple[float, ...]) -> Rotation:
    q = np.asarray(quat_wxyz, dtype=np.float64)
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


class ElasticBand:
    def __init__(
        self,
        *,
        point: NDArray[np.float64] | None = None,
        length: float | None = None,
        quat_wxyz: NDArray[np.float64] | tuple[float, ...] | None = None,
        stiffness: float = _STIFFNESS,
        damping: float = _DAMPING,
        kp_ang: float = _KP_ANG,
        kd_ang: float = _KD_ANG,
        mode: GantryMode = "cable",
        kp_pos: float = _GEAR_KP_POS,
        kd_pos: float = _GEAR_KD_POS,
    ) -> None:
        self.mode: GantryMode = mode
        self.stiffness = float(stiffness)
        self.damping = float(damping)
        self.kp_pos = float(kp_pos)
        self.kd_pos = float(kd_pos)
        self.kp_ang = float(kp_ang)
        self.kd_ang = float(kd_ang)
        default_z = _GEAR_HOLD_Z if mode == "gear" else HOOK_Z
        self.point = np.array(
            [0.0, 0.0, default_z] if point is None else point, dtype=np.float64
        )
        self.length = 0.0 if length is None else float(length)
        self.quat = np.array(
            [1.0, 0.0, 0.0, 0.0] if quat_wxyz is None else quat_wxyz,
            dtype=np.float64,
        )
        n = np.linalg.norm(self.quat)
        if n < 1e-12:
            raise ValueError("gantry quat is zero")
        self.quat /= n
        self.enable = True

    @classmethod
    def overhead(
        cls,
        attach_pos: NDArray[np.float64],
        *,
        quat_wxyz: NDArray[np.float64] | tuple[float, ...] | None = None,
        hook_z: float = HOOK_Z,
    ) -> ElasticBand:
        """Hook above ``attach_pos``, rest length set so the robot hangs here."""
        hook = np.array([attach_pos[0], attach_pos[1], hook_z], dtype=np.float64)
        dist = float(np.linalg.norm(hook - np.asarray(attach_pos, dtype=np.float64)))
        sag = _G1_MASS_KG * 9.81 / _STIFFNESS
        return cls(
            point=hook,
            length=max(0.0, dist - sag),
            quat_wxyz=quat_wxyz,
            mode="cable",
        )

    @classmethod
    def gear_hold(
        cls,
        attach_pos: NDArray[np.float64],
        *,
        quat_wxyz: NDArray[np.float64] | tuple[float, ...] | None = None,
        hold_z: float = _GEAR_HOLD_Z,
    ) -> ElasticBand:
        """GEAR cartesian setpoint: ``(attach_xy, hold_z)``, ``length=0``."""
        pos = np.asarray(attach_pos, dtype=np.float64)
        return cls(
            point=np.array([pos[0], pos[1], hold_z], dtype=np.float64),
            length=0.0,
            quat_wxyz=quat_wxyz,
            mode="gear",
        )

    @property
    def target(self) -> NDArray[np.float64]:
        """World hook (trolley). Not the attach body."""
        return self.point.copy()

    @property
    def yaw(self) -> float:
        return yaw_from_quat_wxyz(self.quat)

    @property
    def forward_xy(self) -> NDArray[np.float64]:
        yaw = self.yaw
        return np.array([np.cos(yaw), np.sin(yaw)], dtype=np.float64)

    def set_yaw(self, yaw_rad: float) -> None:
        self.quat = quat_wxyz_from_yaw(yaw_rad)

    def nudge_yaw(self, delta_rad: float) -> None:
        self.quat = apply_world_yaw(self.quat, delta_rad)

    def nudge_local(self, *, forward: float = 0.0, left: float = 0.0) -> None:
        """Move the overhead hook in the current heading frame (+X forward)."""
        yaw = self.yaw
        c, s = np.cos(yaw), np.sin(yaw)
        self.point[0] += forward * c - left * s
        self.point[1] += forward * s + left * c

    def advance(self, pose: NDArray[np.float64]) -> NDArray[np.float64]:
        """Cable force + attitude torque. ``pose`` is xpos(3)+xquat(4)+lin(3)+ang(3)."""
        force = self._force(pose[0:3], pose[7:10])
        torque = self._torque(pose[3:7], pose[10:13])
        return np.concatenate([force, torque])

    def _force(self, pos: NDArray[np.float64], lin_vel: NDArray[np.float64]) -> NDArray[np.float64]:
        pos = np.asarray(pos, dtype=np.float64)
        vel = np.asarray(lin_vel, dtype=np.float64)
        if self.mode == "gear":
            # GEAR: f = kp*(point - pos + [0,0,length]) + kd*(0 - v)
            delta = self.point - pos
            delta[2] += self.length
            return self.kp_pos * delta + self.kd_pos * (0.0 - vel)
        # Unitree Advance(x, dx): direction from body to hook, stiffness*(distance - length).
        delta = self.point - pos
        distance = float(np.linalg.norm(delta))
        if distance < 1e-9:
            return np.zeros(3, dtype=np.float64)
        extension = distance - self.length
        if extension <= 0.0:
            return np.zeros(3, dtype=np.float64)
        direction = delta / distance
        v = float(np.dot(vel, direction))
        return (self.stiffness * extension - self.damping * v) * direction

    def _torque(
        self,
        quat_wxyz: NDArray[np.float64],
        ang_vel: NDArray[np.float64],
    ) -> NDArray[np.float64]:
        # GEAR Advance: torque = -kp_ang * rotvec - kd_ang * w, vs identity.
        # Same gains; error vs self.quat so spawn / numpad heading stays locked.
        r_err = _rot_wxyz(quat_wxyz) * _rot_wxyz(self.quat).inv()
        return -self.kp_ang * r_err.as_rotvec() - self.kd_ang * np.asarray(
            ang_vel, dtype=np.float64
        )

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> None:
        if not self.enable:
            data.xfrc_applied[body_id] = 0.0
            return
        vel = np.empty(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0
        )
        # mj_objectVelocity is (ang, lin).
        force = self._force(data.xpos[body_id], vel[3:6])
        torque = self._torque(data.xquat[body_id], vel[0:3])
        data.xfrc_applied[body_id, 0:3] = force
        data.xfrc_applied[body_id, 3:6] = torque
