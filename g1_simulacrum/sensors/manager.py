"""Multi-sensor manager. Rates from config; missing readings are None."""

from __future__ import annotations

import mujoco

from ..config import SensorsConfig
from .d435i import D435iCamera
from .data_types import SensorBundle
from .imu import ImuSensor
from .mid360 import Mid360Lidar

_IMU_NAMES = {
    "pelvis": (
        "imu-pelvis-linear-acceleration",
        "imu-pelvis-angular-velocity",
        "imu_in_pelvis",
    ),
    "torso": (
        "imu-torso-linear-acceleration",
        "imu-torso-angular-velocity",
        "imu_in_torso",
    ),
    "mid360": ("mid360_accel", "mid360_gyro", "mid360_imu_site"),
    "d435i": ("d435i_accel", "d435i_gyro", "d435i_imu_site"),
}


class SensorManager:
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: SensorsConfig,
    ) -> None:
        self._config = config
        self._lidar: Mid360Lidar | None = None
        self._camera: D435iCamera | None = None
        self._imus: dict[str, ImuSensor] = {}

        if config.mid360.enabled:
            self._lidar = Mid360Lidar(model, data, config.mid360)
        if config.d435i.enabled:
            self._camera = D435iCamera(model, data, config.d435i)

        for key, (accel, gyro, frame) in _IMU_NAMES.items():
            cfg = getattr(config.imu, key)
            if cfg.enabled:
                self._imus[key] = ImuSensor(
                    model,
                    data,
                    cfg,
                    accel_sensor_name=accel,
                    gyro_sensor_name=gyro,
                    frame_id=frame,
                )

    def step(self, sim_time: float) -> SensorBundle:
        lidar = self._lidar.step(sim_time) if self._lidar is not None else None
        depth = self._camera.step(sim_time) if self._camera is not None else None
        readings = {
            name: sensor.step(sim_time) for name, sensor in self._imus.items()
        }
        return SensorBundle(
            lidar=lidar,
            depth=depth,
            imu_pelvis=readings.get("pelvis"),
            imu_torso=readings.get("torso"),
            imu_mid360=readings.get("mid360"),
            imu_d435i=readings.get("d435i"),
            timestamp=sim_time,
        )

    def reset(self) -> None:
        if self._lidar is not None:
            self._lidar.reset()
        if self._camera is not None:
            self._camera.reset()
        for sensor in self._imus.values():
            sensor.reset()
