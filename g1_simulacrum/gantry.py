"""Virtual gantry: elastic-band wrench on a floating-base body.

Same idea as Unitree MuJoCo / GEAR-SONIC ``ElasticBand``
(``gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py``): a spring-damper
to a world point, written to ``data.xfrc_applied``. The freejoint, contacts,
and joint PD still go through ``mj_step``. Not a weld and not a balance policy.
"""

from __future__ import annotations

import mujoco
import numpy as np
from numpy.typing import NDArray
from scipy.spatial.transform import Rotation


class ElasticBand:
    def __init__(
        self,
        *,
        point: NDArray[np.float64] | None = None,
        length: float = 0.0,
        kp_pos: float = 10000.0,
        kd_pos: float = 1000.0,
        kp_ang: float = 1000.0,
        kd_ang: float = 10.0,
    ) -> None:
        self.kp_pos = kp_pos
        self.kd_pos = kd_pos
        self.kp_ang = kp_ang
        self.kd_ang = kd_ang
        self.point = np.array([0.0, 0.0, 1.0] if point is None else point, dtype=np.float64)
        self.length = float(length)
        self.enable = True

    @property
    def target(self) -> NDArray[np.float64]:
        t = self.point.copy()
        t[2] += self.length
        return t

    def advance(self, pose: NDArray[np.float64]) -> NDArray[np.float64]:
        """``pose`` is xpos(3) + xquat wxyz(4) + lin_vel(3) + ang_vel(3)."""
        pos = pose[0:3]
        quat_wxyz = pose[3:7]
        lin_vel = pose[7:10]
        ang_vel = pose[10:13]
        delta = self.target - pos
        force = self.kp_pos * delta + self.kd_pos * (0.0 - lin_vel)
        quat_xyzw = np.array([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])
        rotvec = Rotation.from_quat(quat_xyzw).as_rotvec()
        torque = -self.kp_ang * rotvec - self.kd_ang * ang_vel
        return np.concatenate([force, torque])

    def apply(self, model: mujoco.MjModel, data: mujoco.MjData, body_id: int) -> None:
        if not self.enable:
            data.xfrc_applied[body_id] = 0.0
            return
        pose = np.empty(13, dtype=np.float64)
        pose[0:3] = data.xpos[body_id]
        pose[3:7] = data.xquat[body_id]
        vel = np.empty(6, dtype=np.float64)
        mujoco.mj_objectVelocity(
            model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0
        )
        # mj_objectVelocity is (ang, lin); Advance wants (lin, ang).
        pose[7:10] = vel[3:6]
        pose[10:13] = vel[0:3]
        data.xfrc_applied[body_id] = self.advance(pose)
