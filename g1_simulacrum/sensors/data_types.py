"""Typed sensor and observation contracts."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True, slots=True)
class PointCloud:
    """Mid-360 scan in ``mid360_link``."""

    points: NDArray[np.float32]
    intensities: NDArray[np.float32]
    ring_ids: NDArray[np.int32] | None = None
    timestamp: float = 0.0
    frame_id: str = "mid360_link"

    @property
    def num_points(self) -> int:
        return self.points.shape[0]


@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    width: int = 640
    height: int = 480
    fx: float = 382.613
    fy: float = 382.613
    cx: float = 320.0
    cy: float = 240.0

    def as_matrix(self) -> NDArray[np.float64]:
        return np.array(
            [
                [self.fx, 0.0, self.cx],
                [0.0, self.fy, self.cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )


@dataclass(frozen=True, slots=True)
class DepthFrame:
    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]  # metres; 0 = invalid
    intrinsics: CameraIntrinsics = field(default_factory=CameraIntrinsics)
    timestamp: float = 0.0
    frame_id: str = "d435i_color_optical_frame"


@dataclass(frozen=True, slots=True)
class ImuReading:
    accel: NDArray[np.float64]
    gyro: NDArray[np.float64]
    timestamp: float = 0.0
    frame_id: str = "imu_in_pelvis"


@dataclass(frozen=True, slots=True)
class SensorBundle:
    """Any field is None if disabled or not due this control step."""

    lidar: PointCloud | None = None
    depth: DepthFrame | None = None
    imu_pelvis: ImuReading | None = None
    imu_torso: ImuReading | None = None
    imu_mid360: ImuReading | None = None
    imu_d435i: ImuReading | None = None
    timestamp: float = 0.0


@dataclass(frozen=True, slots=True)
class JointState:
    """29 body joints in ``G1JointIndex`` / ``BODY_JOINT_NAMES`` order."""

    position: NDArray[np.float64]
    velocity: NDArray[np.float64]
    torque: NDArray[np.float64]
    timestamp: float = 0.0


@dataclass(frozen=True, slots=True)
class BaseState:
    position: NDArray[np.float64]
    orientation: NDArray[np.float64]
    linear_velocity: NDArray[np.float64]
    angular_velocity: NDArray[np.float64]
    timestamp: float = 0.0


@dataclass(frozen=True, slots=True)
class Observation:
    joint_state: JointState
    base_state: BaseState
    sensors: SensorBundle
    previous_action: NDArray[np.float64] | None = None
    q_hands: NDArray[np.float64] | None = None
    timestamp: float = 0.0
