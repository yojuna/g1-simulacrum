"""Livox Mid-360 via mujoco-lidar on site ``mid360``."""

from __future__ import annotations

import mujoco
import numpy as np

from ..config import Mid360Config
from .base import Sensor
from .data_types import PointCloud
from .noise import apply_lidar_noise


class Mid360Lidar(Sensor):
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: Mid360Config,
    ) -> None:
        super().__init__(model, data, config.rate_hz, name="mid360")
        self._config = config
        from mujoco_lidar import MjLidarWrapper
        from mujoco_lidar.scan_gen import LivoxGenerator

        exclude = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, config.bodyexclude)
        # Group 4 is cosmetic sensor geoms (Mid-360 cylinder, D435i box).
        # mj_multiRay ignores contype; without this the cloud is the 3 cm shell.
        geomgroup = np.ones(6, dtype=np.uint8)
        geomgroup[4] = 0
        self._lidar = MjLidarWrapper(
            model,
            site_name=config.site_name,
            backend=config.backend.value,
            args={
                "bodyexclude": int(exclude) if exclude >= 0 else -1,
                "geomgroup": geomgroup,
            },
        )
        self._pattern = LivoxGenerator("mid360")

    def read(self, sim_time: float) -> PointCloud:
        theta, phi = self._pattern.sample_ray_angles()
        self._lidar.trace_rays(self._data, theta, phi)
        raw = np.asarray(self._lidar.get_hit_points(), dtype=np.float32)
        if raw.ndim != 2 or raw.shape[1] != 3:
            raw = np.zeros((0, 3), dtype=np.float32)
        if self._config.noise.range_sigma > 0 or self._config.noise.dropout_rate > 0:
            points, intensities = apply_lidar_noise(
                raw,
                range_sigma=self._config.noise.range_sigma,
                dropout_rate=self._config.noise.dropout_rate,
                near_field_rate=self._config.noise.near_field_clutter_rate,
                near_field_max=self._config.noise.near_field_max_range,
            )
        else:
            points = raw
            intensities = np.full(len(raw), 128.0, dtype=np.float32)
        return PointCloud(
            points=points.astype(np.float32),
            intensities=intensities.astype(np.float32),
            timestamp=sim_time,
            frame_id="mid360_link",
        )
