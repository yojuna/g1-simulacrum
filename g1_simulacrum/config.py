"""Configuration models for the g1-simulacrum.

All configuration is validated by Pydantic and can be loaded from YAML files.
"""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# Sensor configs
# ---------------------------------------------------------------------------

class LidarBackend(str, Enum):
    CPU = "cpu"
    TAICHI = "taichi"
    JAX = "jax"
    WARP = "warp"


class LidarNoiseConfig(BaseModel):
    range_sigma: float = 0.02
    dropout_rate: float = 0.02
    near_field_clutter_rate: float = 0.01
    near_field_max_range: float = 0.3


class Mid360Config(BaseModel):
    enabled: bool = True
    backend: LidarBackend = LidarBackend.CPU
    rate_hz: float = 10.0
    noise: LidarNoiseConfig = Field(default_factory=LidarNoiseConfig)
    mount_body: str = "torso_link"
    mount_pos: tuple[float, float, float] = (0.0, 0.0, 0.05)
    mount_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class DepthNoiseConfig(BaseModel):
    edge_erosion: bool = True
    depth_noise_sigma: float = 0.005
    hole_rate: float = 0.01
    max_range: float = 10.0
    min_range: float = 0.105


class D435iConfig(BaseModel):
    enabled: bool = True
    rate_hz: float = 30.0
    resolution: tuple[int, int] = (640, 480)
    noise: DepthNoiseConfig = Field(default_factory=DepthNoiseConfig)
    mount_body: str = "head_link"
    mount_pos: tuple[float, float, float] = (0.05, 0.0, 0.0)
    mount_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class ImuNoiseConfig(BaseModel):
    accel_sigma: float = 0.01
    gyro_sigma: float = 0.005
    accel_bias_drift: float = 0.0001
    gyro_bias_drift: float = 0.00005


class ImuConfig(BaseModel):
    enabled: bool = True
    rate_hz: float = 200.0
    noise: ImuNoiseConfig = Field(default_factory=ImuNoiseConfig)


class SensorsConfig(BaseModel):
    mid360: Mid360Config = Field(default_factory=Mid360Config)
    d435i: D435iConfig = Field(default_factory=D435iConfig)
    imu: ImuConfig = Field(default_factory=ImuConfig)


# ---------------------------------------------------------------------------
# Controller configs
# ---------------------------------------------------------------------------

class SonicConfig(BaseModel):
    """GEAR-SONIC specific settings."""
    dds_domain: int = 0
    checkpoint: str = "sonic_v1_1"
    observation_history_len: int = 4
    observation_dt: float = 0.02
    motor_kp_scale: dict[int, float] = Field(default_factory=dict)
    motor_kd_scale: dict[int, float] = Field(default_factory=dict)


class ControllerConfig(BaseModel):
    type: Literal["sonic", "pd", "passthrough"] = "sonic"
    physics_hz: float = 1000.0
    control_hz: float = 200.0
    sonic: SonicConfig = Field(default_factory=SonicConfig)


# ---------------------------------------------------------------------------
# Environment configs
# ---------------------------------------------------------------------------

class EnvironmentConfig(BaseModel):
    type: Literal["empty_arena", "robocasa", "custom"] = "empty_arena"
    scene_xml: str | None = None         # path for custom MJCF
    robocasa_scene: str | None = None    # e.g. "kitchen-001"
    robocasa_task: str | None = None     # e.g. "PnPCounterToCab"
    ground_friction: tuple[float, float, float] = (1.0, 0.005, 0.0001)
    spawn_pos: tuple[float, float, float] = (0.0, 0.0, 0.82)
    spawn_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


# ---------------------------------------------------------------------------
# Interface configs
# ---------------------------------------------------------------------------

class GymConfig(BaseModel):
    enabled: bool = True
    obs_keys: list[str] = Field(
        default_factory=lambda: ["proprioception", "lidar", "depth", "imu"]
    )
    reward_type: str = "sparse"
    max_episode_steps: int = 1000


class Ros2Config(BaseModel):
    enabled: bool = False
    namespace: str = "/g1"
    qos_depth: int = 10


class InterfaceConfig(BaseModel):
    gym: GymConfig = Field(default_factory=GymConfig)
    ros2: Ros2Config = Field(default_factory=Ros2Config)


# ---------------------------------------------------------------------------
# Robot config
# ---------------------------------------------------------------------------

class RobotConfig(BaseModel):
    model: str = "g1_29dof"
    initial_height: float = 0.82
    self_collision: bool = True


# ---------------------------------------------------------------------------
# Top-level config
# ---------------------------------------------------------------------------

class G1SimulacrumConfig(BaseModel):
    """Top-level configuration for the entire simulation stack."""

    robot: RobotConfig = Field(default_factory=RobotConfig)
    sensors: SensorsConfig = Field(default_factory=SensorsConfig)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)
    environment: EnvironmentConfig = Field(default_factory=EnvironmentConfig)
    interface: InterfaceConfig = Field(default_factory=InterfaceConfig)
    render: bool = True
    seed: int = 42

    @classmethod
    def from_yaml(cls, path: str | Path) -> G1SimulacrumConfig:
        """Load configuration from a YAML file."""
        with open(path) as f:
            data = yaml.safe_load(f)
        return cls.model_validate(data)

    def to_yaml(self, path: str | Path) -> None:
        """Save configuration to a YAML file."""
        with open(path, "w") as f:
            yaml.dump(
                self.model_dump(mode="json"),
                f,
                default_flow_style=False,
                sort_keys=False,
            )
