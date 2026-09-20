"""Dedicated lidar + depth worker orchestration (Phase 2).

Extracted from ``SonicDdsSimLoop`` so sensor threading can be unit-tested without
initializing CycloneDDS / ``UnitreeSdk2Bridge``.
"""

from __future__ import annotations

from dataclasses import dataclass

import mujoco

from g1_simulacrum.config import D435iConfig, Mid360Config
from g1_simulacrum.sensors.data_types import DepthFrame, PointCloud

from .depth_worker import DepthWorker
from .lidar_worker import LidarWorker
from .pose_snapshot import PoseSnapshotBuffer


@dataclass(frozen=True, slots=True)
class SensorWorkerPoll:
    """New sensor products since the previous ``poll()`` call."""

    cloud: PointCloud | None = None
    depth: DepthFrame | None = None
    lidar_frames: int = 0
    depth_frames: int = 0

    @property
    def refreshed(self) -> bool:
        return self.cloud is not None or self.depth is not None


class SensorWorkerGroup:
    """Lidar + depth workers sharing one pose snapshot buffer."""

    def __init__(
        self,
        model: mujoco.MjModel,
        mid360: Mid360Config,
        d435i: D435iConfig,
        *,
        pose_buffer: PoseSnapshotBuffer | None = None,
    ) -> None:
        self.pose_buffer = pose_buffer or PoseSnapshotBuffer()
        self.lidar = LidarWorker(model, mid360, self.pose_buffer)
        self.depth = DepthWorker(model, d435i, self.pose_buffer)
        self._lidar_scan_seq = 0
        self._depth_frame_seq = 0

    @classmethod
    def from_configs(
        cls,
        model: mujoco.MjModel,
        mid360: Mid360Config,
        d435i: D435iConfig,
    ) -> SensorWorkerGroup:
        return cls(model, mid360, d435i)

    def start(self) -> None:
        self.lidar.start()
        self.depth.start()

    def stop(self) -> None:
        self.depth.stop()
        self.lidar.stop()

    def publish(self, data: mujoco.MjData) -> None:
        self.pose_buffer.publish(data)

    def poll(self) -> SensorWorkerPoll:
        cloud: PointCloud | None = None
        depth: DepthFrame | None = None
        lidar_frames = 0
        depth_frames = 0

        lidar_seq = self.lidar.last_scan_seq()
        if lidar_seq != self._lidar_scan_seq:
            self._lidar_scan_seq = lidar_seq
            cloud = self.lidar.get_last_cloud()
            if cloud is not None:
                lidar_frames = 1

        depth_seq = self.depth.last_frame_seq()
        if depth_seq != self._depth_frame_seq:
            self._depth_frame_seq = depth_seq
            depth = self.depth.get_last_depth()
            if depth is not None:
                depth_frames = 1

        return SensorWorkerPoll(
            cloud=cloud,
            depth=depth,
            lidar_frames=lidar_frames,
            depth_frames=depth_frames,
        )
