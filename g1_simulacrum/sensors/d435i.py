"""Intel RealSense D435i depth camera sensor.

Uses MuJoCo's native camera rendering for RGB and depth, with a
stereo-matching noise model for realistic sim-to-real transfer.
"""

from __future__ import annotations

import textwrap

import mujoco
import numpy as np
from numpy.typing import NDArray

from ..config import D435iConfig
from .base import Sensor
from .data_types import CameraIntrinsics, DepthFrame
from .noise import apply_depth_noise


class D435iCamera(Sensor):
    """Intel RealSense D435i stereo depth camera.

    Real sensor specs:
        - Depth: 1280×720 @ 30fps (we default to 640×480 for perf)
        - RGB: 1920×1080 @ 30fps
        - H-FOV: 87° (depth), 69° (RGB)
        - Range: 0.105 m – 10 m
        - Baseline: 50 mm
        - Built-in IMU: BMI055 (handled separately by ImuSensor)
    """

    # D435i intrinsics at 640×480 (from factory calibration median)
    _DEFAULT_INTRINSICS = CameraIntrinsics(
        width=640, height=480,
        fx=382.613, fy=382.613,
        cx=320.0, cy=240.0,
    )

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
            width=w, height=h,
            fx=382.613 * (w / 640),
            fy=382.613 * (h / 480),
            cx=w / 2.0,
            cy=h / 2.0,
        )

        # Renderer is created lazily after the model is compiled
        self._renderer: mujoco.Renderer | None = None
        self._camera_name = "d435i_depth"

    def _ensure_renderer(self) -> mujoco.Renderer:
        if self._renderer is None:
            w, h = self._config.resolution
            self._renderer = mujoco.Renderer(self._model, height=h, width=w)
        return self._renderer

    def read(self, sim_time: float) -> DepthFrame:
        """Render RGB and depth from the D435i viewpoint."""
        renderer = self._ensure_renderer()

        # --- RGB ---
        renderer.update_scene(self._data, camera=self._camera_name)
        rgb = renderer.render().copy()  # (H, W, 3) uint8

        # --- Depth ---
        renderer.enable_depth_rendering()
        renderer.update_scene(self._data, camera=self._camera_name)
        raw_depth = renderer.render().copy()  # (H, W) float32, z-buffer [0,1]
        renderer.disable_depth_rendering()

        # Convert z-buffer to metric depth
        metric_depth = self._zbuffer_to_meters(raw_depth)

        # Apply stereo-matching noise model
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

    def _zbuffer_to_meters(self, zbuf: NDArray[np.float32]) -> NDArray[np.float64]:
        """Convert MuJoCo's [0,1] z-buffer to metric depth (meters)."""
        extent = self._model.stat.extent
        near = self._model.vis.map.znear * extent
        far = self._model.vis.map.zfar * extent
        # Avoid division by zero
        denom = 1.0 - zbuf * (1.0 - near / far)
        denom = np.maximum(denom, 1e-10)
        return near / denom

    def get_mount_xml(self) -> str:
        """MJCF snippet for the D435i body, cameras, and IMU site."""
        px, py, pz = self._config.mount_pos
        qw, qx, qy, qz = self._config.mount_quat
        w, h = self._config.resolution

        # MuJoCo fovy = vertical FOV. D435i depth V-FOV ≈ 58°.
        fovy = 58

        return textwrap.dedent(f"""\
            <body name="d435i_link" pos="{px} {py} {pz}" quat="{qw} {qx} {qy} {qz}">
                <inertial pos="0 0 0" mass="0.072"
                          diaginertia="0.00005 0.00005 0.00002"/>
                <geom type="box" size="0.0445 0.0125 0.0125"
                      rgba="0.3 0.3 0.3 1" contype="0" conaffinity="0"
                      group="1"/>
                <camera name="d435i_depth" pos="0 0 0" fovy="{fovy}"
                        resolution="{w} {h}"/>
                <camera name="d435i_rgb" pos="0 0.015 0" fovy="{fovy}"
                        resolution="{w} {h}"/>
                <site name="d435i_imu_site" pos="0 0 0"/>
            </body>
        """)

    def reset(self) -> None:
        super().reset()
        # Renderer state doesn't need resetting
