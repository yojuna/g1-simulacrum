"""Dedicated Mid-360 lidar worker thread (Phase 2 PR1)."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import mujoco

from g1_simulacrum.config import Mid360Config
from g1_simulacrum.sensors.data_types import PointCloud
from g1_simulacrum.sensors.mid360 import Mid360Lidar

from .pose_snapshot import PoseSnapshotBuffer, apply_snapshot


@dataclass(frozen=True, slots=True)
class LidarWorkerStats:
    scans: int = 0
    skips: int = 0
    last_seq: int = 0


class LidarWorker:
    """10 Hz (configurable) lidar scans on a private ``mjData``."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config: Mid360Config,
        pose_buffer: PoseSnapshotBuffer,
        *,
        rate_hz: float | None = None,
    ) -> None:
        self._model = model
        self._worker_data = mujoco.MjData(model)
        self._lidar = Mid360Lidar(model, self._worker_data, config)
        self._pose_buffer = pose_buffer
        self._period_s = 1.0 / max(float(rate_hz or config.rate_hz), 1.0)
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._last_cloud: PointCloud | None = None
        self._last_scan_seq = 0
        self._stats = LidarWorkerStats()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run,
            name="sonic-lidar-worker",
            daemon=True,
        )
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 2.0) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, join_timeout_s))
            self._thread = None

    def stats(self) -> LidarWorkerStats:
        with self._lock:
            return self._stats

    def get_last_cloud(self) -> PointCloud | None:
        with self._lock:
            return self._last_cloud

    def last_scan_seq(self) -> int:
        with self._lock:
            return self._last_scan_seq

    def _run(self) -> None:
        next_wake = time.monotonic()
        last_pose_seq = -1
        while self._running:
            snap = self._pose_buffer.read()
            if snap is None:
                time.sleep(0.001)
                continue

            now = time.monotonic()
            if now < next_wake:
                time.sleep(min(0.001, next_wake - now))
                continue

            if snap.seq == last_pose_seq:
                # No new motion sample — still scan at rate with last pose.
                pass
            last_pose_seq = snap.seq

            if now > next_wake + self._period_s:
                with self._lock:
                    self._stats = LidarWorkerStats(
                        scans=self._stats.scans,
                        skips=self._stats.skips + 1,
                        last_seq=self._stats.last_seq,
                    )
                next_wake = now

            apply_snapshot(self._model, self._worker_data, snap)
            cloud = self._lidar.read(snap.time)
            with self._lock:
                self._last_cloud = cloud
                self._last_scan_seq += 1
                self._stats = LidarWorkerStats(
                    scans=self._stats.scans + 1,
                    skips=self._stats.skips,
                    last_seq=self._last_scan_seq,
                )
            next_wake += self._period_s
