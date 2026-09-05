"""IMU from named MuJoCo accelerometer + gyro sensors."""

from __future__ import annotations

import mujoco
import numpy as np

from ..config import ImuRateConfig
from .base import Sensor
from .data_types import ImuReading


class ImuSensor(Sensor):
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: ImuRateConfig,
        *,
        accel_sensor_name: str,
        gyro_sensor_name: str,
        frame_id: str,
    ) -> None:
        super().__init__(model, data, config.rate_hz, name=f"imu_{frame_id}")
        self._config = config
        self._frame_id = frame_id
        self._accel_adr = self._sensor_adr(accel_sensor_name)
        self._gyro_adr = self._sensor_adr(gyro_sensor_name)
        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)
        self._rng = np.random.default_rng()

    def _sensor_adr(self, name: str) -> int:
        sensor_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            raise ValueError(f"IMU sensor {name!r} not found in compiled MJCF")
        return int(self._model.sensor_adr[sensor_id])

    def read(self, sim_time: float) -> ImuReading:
        accel = self._data.sensordata[self._accel_adr : self._accel_adr + 3].copy()
        gyro = self._data.sensordata[self._gyro_adr : self._gyro_adr + 3].copy()
        noise = self._config.noise
        accel = accel + self._rng.normal(0, noise.accel_sigma, 3)
        gyro = gyro + self._rng.normal(0, noise.gyro_sigma, 3)
        dt = 1.0 / self._rate_hz
        self._accel_bias += self._rng.normal(0, noise.accel_bias_drift * dt, 3)
        self._gyro_bias += self._rng.normal(0, noise.gyro_bias_drift * dt, 3)
        return ImuReading(
            accel=accel + self._accel_bias,
            gyro=gyro + self._gyro_bias,
            timestamp=sim_time,
            frame_id=self._frame_id,
        )

    def reset(self) -> None:
        super().reset()
        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)
