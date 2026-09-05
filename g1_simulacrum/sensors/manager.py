"""Multi-sensor manager that orchestrates readings at configured rates."""

from __future__ import annotations

import mujoco

from ..config import SensorsConfig
from .base import Sensor
from .d435i import D435iCamera
from .data_types import SensorBundle
from .imu import ImuSensor
from .mid360 import Mid360Lidar


class SensorManager:
    """Owns and orchestrates all sensors attached to the G1.

    Each sensor runs at its own configured rate.  The manager's ``step``
    method is called every simulation tick; it checks each sensor's timer
    and collects readings only when due, returning a ``SensorBundle``.
    """

    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        config: SensorsConfig,
    ) -> None:
        self._sensors: dict[str, Sensor] = {}

        if config.mid360.enabled:
            self._sensors["mid360"] = Mid360Lidar(model, data, config.mid360)

        if config.d435i.enabled:
            self._sensors["d435i"] = D435iCamera(model, data, config.d435i)

        if config.imu.enabled:
            # Two IMUs: one co-located with LiDAR, one with camera
            self._sensors["imu_lidar"] = ImuSensor(
                model, data, config.imu,
                accel_sensor_name="mid360_accel",
                gyro_sensor_name="mid360_gyro",
                frame_id="mid360_imu_link",
            )
            if config.d435i.enabled:
                self._sensors["imu_camera"] = ImuSensor(
                    model, data, config.imu,
                    accel_sensor_name="d435i_accel",
                    gyro_sensor_name="d435i_gyro",
                    frame_id="d435i_imu_link",
                )

    @property
    def sensors(self) -> dict[str, Sensor]:
        return dict(self._sensors)

    def step(self, sim_time: float) -> SensorBundle:
        """Collect sensor readings that are due at the current sim time.

        Returns a SensorBundle where fields are None if the corresponding
        sensor did not fire this tick.
        """
        lidar_reading = None
        depth_reading = None
        imu_reading = None

        if "mid360" in self._sensors:
            lidar_reading = self._sensors["mid360"].step(sim_time)

        if "d435i" in self._sensors:
            depth_reading = self._sensors["d435i"].step(sim_time)

        # Primary IMU (LiDAR-mounted) is the one exposed in the bundle
        if "imu_lidar" in self._sensors:
            imu_reading = self._sensors["imu_lidar"].step(sim_time)

        return SensorBundle(
            lidar=lidar_reading,
            depth=depth_reading,
            imu=imu_reading,
            timestamp=sim_time,
        )

    def reset(self) -> None:
        """Reset all sensors (clears timers, bias accumulators, etc.)."""
        for sensor in self._sensors.values():
            sensor.reset()

    def get_mount_xml_snippets(self) -> dict[str, str]:
        """Collect MJCF mount snippets from all sensors.

        Used by the model composer to inject sensor bodies before compilation.
        """
        return {name: s.get_mount_xml() for name, s in self._sensors.items()}
