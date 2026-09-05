"""Sensor data types used across the entire stack.

These dataclasses define the contracts between the sensor layer and all consumers
(interface layer, ROS2 bridge, Gym observations, downstream policies).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from numpy.typing import NDArray


# ---------------------------------------------------------------------------
# LiDAR
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class PointCloud:
    """A single Mid-360 LiDAR scan.

    Attributes:
        points: (N, 3) float32 array of XYZ points in the sensor frame.
        intensities: (N,) float32 array of return intensities [0, 255].
        ring_ids: (N,) int32 array identifying which scan ring produced each point.
        timestamp: Simulation time (seconds) when the scan was captured.
        frame_id: Coordinate frame name for TF lookups.
    """

    points: NDArray[np.float32]
    intensities: NDArray[np.float32]
    ring_ids: NDArray[np.int32] | None = None
    timestamp: float = 0.0
    frame_id: str = "mid360_link"

    @property
    def num_points(self) -> int:
        return self.points.shape[0]


# ---------------------------------------------------------------------------
# Depth camera
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class CameraIntrinsics:
    """Pinhole camera intrinsics matching the real D435i."""

    width: int = 640
    height: int = 480
    fx: float = 382.613  # focal length x (pixels)
    fy: float = 382.613  # focal length y (pixels)
    cx: float = 320.0    # principal point x
    cy: float = 240.0    # principal point y

    def as_matrix(self) -> NDArray[np.float64]:
        """3×3 camera matrix K."""
        return np.array([
            [self.fx, 0.0, self.cx],
            [0.0, self.fy, self.cy],
            [0.0, 0.0, 1.0],
        ], dtype=np.float64)


@dataclass(frozen=True, slots=True)
class DepthFrame:
    """A single D435i depth + RGB capture.

    Attributes:
        rgb: (H, W, 3) uint8 color image.
        depth: (H, W) float32 metric depth in meters. 0 = invalid/missing.
        intrinsics: Camera intrinsic parameters.
        timestamp: Simulation time (seconds) when the frame was captured.
        frame_id: Coordinate frame name for TF lookups.
    """

    rgb: NDArray[np.uint8]
    depth: NDArray[np.float32]
    intrinsics: CameraIntrinsics = field(default_factory=CameraIntrinsics)
    timestamp: float = 0.0
    frame_id: str = "d435i_color_optical_frame"


# ---------------------------------------------------------------------------
# IMU
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class ImuReading:
    """A single IMU measurement.

    All values are in the sensor body frame.

    Attributes:
        accel: (3,) float64 linear acceleration including gravity (m/s²).
        gyro: (3,) float64 angular velocity (rad/s).
        timestamp: Simulation time (seconds).
        frame_id: Coordinate frame name.
    """

    accel: NDArray[np.float64]
    gyro: NDArray[np.float64]
    timestamp: float = 0.0
    frame_id: str = "mid360_imu_link"


# ---------------------------------------------------------------------------
# Composed bundle
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class SensorBundle:
    """All sensor readings from a single simulation step.

    Any field may be None if that sensor is disabled or its rate hasn't
    triggered a new reading this step.
    """

    lidar: PointCloud | None = None
    depth: DepthFrame | None = None
    imu: ImuReading | None = None
    timestamp: float = 0.0


# ---------------------------------------------------------------------------
# Proprioception (joint state)
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class JointState:
    """Proprioceptive state of the G1's 29 actuated joints.

    All arrays are shape (29,) ordered to match GEAR-SONIC convention:
    [left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee,
     left_ankle_pitch, left_ankle_roll,
     right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee,
     right_ankle_pitch, right_ankle_roll,
     waist_yaw, waist_roll, waist_pitch,
     left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,
     left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
     right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,
     right_wrist_roll, right_wrist_pitch, right_wrist_yaw]
    """

    position: NDArray[np.float64]   # radians
    velocity: NDArray[np.float64]   # rad/s
    torque: NDArray[np.float64]     # N·m (measured)
    timestamp: float = 0.0

    NUM_JOINTS: int = field(default=29, init=False, repr=False)


# ---------------------------------------------------------------------------
# Base state (odometry)
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class BaseState:
    """6-DOF base (pelvis) state relative to world frame."""

    position: NDArray[np.float64]      # (3,) xyz in world
    orientation: NDArray[np.float64]   # (4,) quaternion [w, x, y, z]
    linear_velocity: NDArray[np.float64]   # (3,) in body frame
    angular_velocity: NDArray[np.float64]  # (3,) in body frame
    timestamp: float = 0.0


# ---------------------------------------------------------------------------
# Full observation (for Gym / policy consumption)
# ---------------------------------------------------------------------------

@dataclass(frozen=True, slots=True)
class Observation:
    """Complete observation from a simulation step."""

    joint_state: JointState
    base_state: BaseState
    sensors: SensorBundle
    previous_action: NDArray[np.float64] | None = None
    timestamp: float = 0.0
