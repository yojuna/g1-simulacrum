"""Dedicated D435i depth worker thread (Phase 2 PR2)."""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import mujoco

from g1_simulacrum.config import D435iConfig
from g1_simulacrum.sensors.d435i import D435iCamera
from g1_simulacrum.sensors.data_types import DepthFrame

from .platform_extero import camera_T_world_optical
from .pose_snapshot import PoseSnapshotBuffer, apply_snapshot


@dataclass(frozen=True, slots=True)
class DepthWorkerStats:
    frames: int = 0
    skips: int = 0
    last_seq: int = 0


class DepthWorker:
    """30 Hz (configurable) depth/RGB renders on a private ``mjData`` + ``Renderer``."""

    def __init__(
        self,
        model: mujoco.MjModel,
        config: D435iConfig,
        pose_buffer: PoseSnapshotBuffer,
        *,
        rate_hz: float | None = None,
    ) -> None:
        self._model = model
        self._worker_data = mujoco.MjData(model)
        self._camera = D435iCamera(model, self._worker_data, config)
        self._pose_buffer = pose_buffer
        self._period_s = 1.0 / max(float(rate_hz or config.rate_hz), 1.0)
        self._thread: threading.Thread | None = None
        self._running = False
        self._lock = threading.Lock()
        self._last_depth: DepthFrame | None = None
        self._last_frame_seq = 0
        self._stats = DepthWorkerStats()

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._run,
            name="sonic-depth-worker",
            daemon=True,
        )
        self._thread.start()

    def stop(self, *, join_timeout_s: float = 2.0) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=max(0.0, join_timeout_s))
            self._thread = None

    def stats(self) -> DepthWorkerStats:
        with self._lock:
            return self._stats

    def get_last_depth(self) -> DepthFrame | None:
        with self._lock:
            return self._last_depth

    def last_frame_seq(self) -> int:
        with self._lock:
            return self._last_frame_seq

    def _run(self) -> None:
        next_wake = time.monotonic()
        while self._running:
            snap = self._pose_buffer.read()
            if snap is None:
                time.sleep(0.001)
                continue

            now = time.monotonic()
            if now < next_wake:
                time.sleep(min(0.001, next_wake - now))
                continue

            if now > next_wake + self._period_s:
                with self._lock:
                    self._stats = DepthWorkerStats(
                        frames=self._stats.frames,
                        skips=self._stats.skips + 1,
                        last_seq=self._stats.last_seq,
                    )
                next_wake = now

            apply_snapshot(self._model, self._worker_data, snap)
            raw = self._camera.read(snap.time)
            try:
                T_wc = camera_T_world_optical(
                    self._model,
                    self._worker_data,
                    self._camera._config.rgb_camera,
                )
            except ValueError:
                T_wc = None
            depth = DepthFrame(
                rgb=raw.rgb,
                depth=raw.depth,
                intrinsics=raw.intrinsics,
                timestamp=raw.timestamp,
                frame_id=raw.frame_id,
                ref_proprio_seq=int(snap.ref_proprio_seq),
                T_world_camera=T_wc,
            )
            with self._lock:
                self._last_depth = depth
                self._last_frame_seq += 1
                self._stats = DepthWorkerStats(
                    frames=self._stats.frames + 1,
                    skips=self._stats.skips,
                    last_seq=self._last_frame_seq,
                )
            next_wake += self._period_s
