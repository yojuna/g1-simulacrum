"""IMU sensor reading MuJoCo's native accelerometer and gyroscope."""

from __future__ import annotations

import textwrap

import mujoco
import numpy as np

from ..config import ImuConfig
from .base import Sensor
from .data_types import ImuReading


class ImuSensor(Sensor):
    """6-axis IMU (accelerometer + gyroscope).

    Reads from MuJoCo's built-in ``accelerometer`` and ``gyro`` sensor types.
    Adds configurable white noise and slow bias drift to simulate MEMS
    imperfections (ICM-40609-D for Mid-360, BMI055 for D435i).
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: ImuConfig,
        *,
        accel_sensor_name: str = "mid360_accel",
        gyro_sensor_name: str = "mid360_gyro",
        frame_id: str = "mid360_imu_link",
    ) -> None:
        super().__init__(model, data, config.rate_hz, name=f"imu_{frame_id}")
        self._config = config
        self._accel_name = accel_sensor_name
        self._gyro_name = gyro_sensor_name
        self._frame_id = frame_id

        # Resolve sensor indices (will raise if sensors not found in model)
        self._accel_adr = self._find_sensor_adr(accel_sensor_name)
        self._gyro_adr = self._find_sensor_adr(gyro_sensor_name)

        # Bias state (random walk)
        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)
        self._rng = np.random.default_rng()

    def _find_sensor_adr(self, name: str) -> int:
        """Find the sensor data address by name."""
        sensor_id = mujoco.mj_name2id(self._model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            raise ValueError(
                f"Sensor '{name}' not found in model. "
                f"Make sure the sensor mount XML has been included."
            )
        return self._model.sensor_adr[sensor_id]

    def read(self, sim_time: float) -> ImuReading:
        """Read accelerometer and gyro from MuJoCo sensor data."""
        # Raw readings from MuJoCo (body-frame)
        accel_raw = self._data.sensordata[self._accel_adr:self._accel_adr + 3].copy()
        gyro_raw = self._data.sensordata[self._gyro_adr:self._gyro_adr + 3].copy()

        # Add white noise
        noise_cfg = self._config.noise
        accel = accel_raw + self._rng.normal(0, noise_cfg.accel_sigma, 3)
        gyro = gyro_raw + self._rng.normal(0, noise_cfg.gyro_sigma, 3)

        # Add bias drift (random walk)
        dt = 1.0 / self._rate_hz
        self._accel_bias += self._rng.normal(0, noise_cfg.accel_bias_drift * dt, 3)
        self._gyro_bias += self._rng.normal(0, noise_cfg.gyro_bias_drift * dt, 3)
        accel += self._accel_bias
        gyro += self._gyro_bias

        return ImuReading(
            accel=accel,
            gyro=gyro,
            timestamp=sim_time,
            frame_id=self._frame_id,
        )

    def get_mount_xml(self) -> str:
        """MJCF sensor definitions — the body/site is defined by the
        parent sensor (Mid360 or D435i).  This just adds the
        accelerometer and gyro sensor tags.
        """
        site = self._accel_name.replace("_accel", "_imu_site")
        return textwrap.dedent(f"""\
            <accelerometer name="{self._accel_name}" site="{site}"
                           noise="{self._config.noise.accel_sigma}"/>
            <gyro name="{self._gyro_name}" site="{site}"
                  noise="{self._config.noise.gyro_sigma}"/>
        """)

    def reset(self) -> None:
        super().reset()
        self._accel_bias = np.zeros(3)
        self._gyro_bias = np.zeros(3)
