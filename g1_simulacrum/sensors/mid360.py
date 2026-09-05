"""Livox Mid-360 LiDAR sensor.

Wraps the ``mujoco_lidar`` package with the ``mid360`` preset, adds
realistic noise, and outputs ``PointCloud`` dataclasses.
"""

from __future__ import annotations

import textwrap

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..config import Mid360Config
from .base import Sensor
from .data_types import PointCloud
from .noise import apply_lidar_noise

# Lazy import so the package loads even if mujoco_lidar isn't installed yet.
_LidarSensor = None


def _get_lidar_class():
    global _LidarSensor
    if _LidarSensor is None:
        from mujoco_lidar import LidarSensor as _Cls
        _LidarSensor = _Cls
    return _LidarSensor


class Mid360Lidar(Sensor):
    """Livox Mid-360 360° LiDAR with non-repetitive scan pattern.

    Real sensor specs:
        - FOV: 360° horizontal × 59° vertical (−7° to +52°)
        - Points/sec: 200,000
        - Range: 40 m (90% reflectivity)
        - Weight: 265 g
        - Built-in IMU: ICM-40609-D (handled separately by ImuSensor)
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: Mid360Config,
    ) -> None:
        super().__init__(model, data, config.rate_hz, name="mid360")
        self._config = config

        LidarCls = _get_lidar_class()
        self._lidar = LidarCls(
            model=model,
            data=data,
            sensor_type="mid360",
            body_name=config.mount_body,
            backend=config.backend.value,
        )

    def read(self, sim_time: float) -> PointCloud:
        """Capture a full scan and apply noise model."""
        raw_points = self._lidar.get_point_cloud()  # (N, 3) in sensor frame

        if self._config.noise.range_sigma > 0 or self._config.noise.dropout_rate > 0:
            points, intensities = apply_lidar_noise(
                raw_points,
                range_sigma=self._config.noise.range_sigma,
                dropout_rate=self._config.noise.dropout_rate,
                near_field_rate=self._config.noise.near_field_clutter_rate,
                near_field_max=self._config.noise.near_field_max_range,
            )
        else:
            points = raw_points
            intensities = np.full(len(raw_points), 128.0, dtype=np.float32)

        return PointCloud(
            points=points.astype(np.float32),
            intensities=intensities.astype(np.float32),
            timestamp=sim_time,
            frame_id="mid360_link",
        )

    def get_mount_xml(self) -> str:
        """MJCF snippet for the Mid-360 body and IMU site on the G1."""
        px, py, pz = self._config.mount_pos
        qw, qx, qy, qz = self._config.mount_quat
        return textwrap.dedent(f"""\
            <body name="mid360_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
                <inertial pos="0 0 0" mass="0.265"
                          diaginertia="0.0002 0.0002 0.0002"/>
                <geom type="cylinder" size="0.0325 0.03"
                      rgba="0.15 0.15 0.15 1" contype="0" conaffinity="0"
                      group="1"/>
                <site name="mid360_imu_site" pos="0 0 0"/>
            </body>
        """)

    def reset(self) -> None:
        super().reset()
