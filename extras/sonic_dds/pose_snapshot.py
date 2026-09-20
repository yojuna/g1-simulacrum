"""Keep-last-1 pose snapshot for sensor worker threads (Phase 2).

Motion publishes after each control tick; lidar/depth workers copy the latest
pose into a private ``mjData`` and run ``mj_forward`` before raycast/render.
"""

from __future__ import annotations

import threading
from dataclasses import dataclass

import mujoco
import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class PoseSnapshot:
    """Minimal dynamics state for sensor geometry (sim time stamp included)."""

    time: float
    seq: int
    qpos: NDArray[np.float64]
    qvel: NDArray[np.float64]
    ref_proprio_seq: int = 0


class PoseSnapshotBuffer:
    """Thread-safe keep-last-1 buffer written by motion, read by sensor workers."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._latest: PoseSnapshot | None = None
        self._seq = 0

    @property
    def seq(self) -> int:
        with self._lock:
            return self._seq

    def publish(
        self,
        data: mujoco.MjData,
        *,
        ref_proprio_seq: int = 0,
    ) -> PoseSnapshot:
        with self._lock:
            self._seq += 1
            snap = PoseSnapshot(
                time=float(data.time),
                seq=self._seq,
                qpos=np.asarray(data.qpos, dtype=np.float64).copy(),
                qvel=np.asarray(data.qvel, dtype=np.float64).copy(),
                ref_proprio_seq=int(ref_proprio_seq),
            )
            self._latest = snap
            return snap

    def read(self) -> PoseSnapshot | None:
        with self._lock:
            if self._latest is None:
                return None
            snap = self._latest
            return PoseSnapshot(
                time=snap.time,
                seq=snap.seq,
                qpos=snap.qpos.copy(),
                qvel=snap.qvel.copy(),
                ref_proprio_seq=int(snap.ref_proprio_seq),
            )


def apply_snapshot(
    model: mujoco.MjModel,
    worker_data: mujoco.MjData,
    snap: PoseSnapshot,
) -> None:
    """Apply a pose snapshot to worker ``mjData`` and refresh kinematics."""
    worker_data.qpos[:] = snap.qpos
    worker_data.qvel[:] = snap.qvel
    mujoco.mj_forward(model, worker_data)
