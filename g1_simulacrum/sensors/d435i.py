"""Intel RealSense D435i stand-in: two MuJoCo cameras (depth 58°, RGB 42°).

MuJoCo 3.12 ``Renderer.enable_depth_rendering()`` already converts the OpenGL
depth buffer to **metric metres** in ``Renderer.render``. Do not apply the
older z-buffer formula (znear/zfar) a second time.
"""

from __future__ import annotations

import mujoco
import numpy as np

from ..config import D435iConfig
from .base import Sensor
from .data_types import CameraIntrinsics, DepthFrame
from .noise import apply_depth_noise


class D435iCamera(Sensor):
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: D435iConfig,
    ) -> None:
        super().__init__(model, data, config.rate_hz, name="d435i")
        self._config = config
        w, h = config.resolution
        self._intrinsics = CameraIntrinsics(
            width=w,
            height=h,
            fx=382.613 * (w / 640),
            fy=382.613 * (h / 480),
            cx=w / 2.0,
            cy=h / 2.0,
        )
        self._renderer: mujoco.Renderer | None = None

    def _ensure_renderer(self) -> mujoco.Renderer:
        if self._renderer is None:
            w, h = self._config.resolution
            self._renderer = mujoco.Renderer(self._model, height=h, width=w)
        return self._renderer

    def read(self, sim_time: float) -> DepthFrame:
        renderer = self._ensure_renderer()

        renderer.disable_depth_rendering()
        renderer.update_scene(self._data, camera=self._config.rgb_camera)
        rgb = renderer.render().copy()

        renderer.enable_depth_rendering()
        renderer.update_scene(self._data, camera=self._config.depth_camera)
        metric_depth = renderer.render().copy()  # metres (MuJoCo 3.12)
        renderer.disable_depth_rendering()

        if self._config.noise.edge_erosion or self._config.noise.depth_noise_sigma > 0:
            metric_depth = apply_depth_noise(
                metric_depth,
                edge_erosion=self._config.noise.edge_erosion,
                sigma=self._config.noise.depth_noise_sigma,
                hole_rate=self._config.noise.hole_rate,
                min_range=self._config.noise.min_range,
                max_range=self._config.noise.max_range,
            )

        return DepthFrame(
            rgb=rgb,
            depth=metric_depth.astype(np.float32),
            intrinsics=self._intrinsics,
            timestamp=sim_time,
            frame_id="d435i_color_optical_frame",
        )
