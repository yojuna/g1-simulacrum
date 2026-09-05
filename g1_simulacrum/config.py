"""Configuration models for g1-simulacrum (Architecture YAML)."""

from __future__ import annotations

from enum import Enum
from pathlib import Path
from typing import Literal

import yaml
from pydantic import BaseModel, Field


class LidarBackend(str, Enum):
    CPU = "cpu"
    TAICHI = "taichi"
    JAX = "jax"
    WARP = "warp"


class LidarNoiseConfig(BaseModel):
    # Livox Mid-360 datasheet 1σ start (≤2 cm @ 10 m). Dropout/clutter are placeholders.
    range_sigma: float = 0.02
    dropout_rate: float = 0.02
    near_field_clutter_rate: float = 0.01
    near_field_max_range: float = 0.3


class Mid360Config(BaseModel):
    enabled: bool = True
    backend: LidarBackend = LidarBackend.CPU
    rate_hz: float = 10.0
    noise: LidarNoiseConfig = Field(default_factory=LidarNoiseConfig)
    site_name: str = "mid360"
    bodyexclude: str = "torso_link"


class DepthNoiseConfig(BaseModel):
    # Stereo stand-in. Useful range 0.3–3 m (wiki); not a flat 0.105–10 m quality.
    edge_erosion: bool = True
    depth_noise_sigma: float = 0.005
    hole_rate: float = 0.01
    max_range: float = 3.0
    min_range: float = 0.3


class D435iConfig(BaseModel):
    enabled: bool = True
    rate_hz: float = 30.0
    resolution: tuple[int, int] = (640, 480)
    noise: DepthNoiseConfig = Field(default_factory=DepthNoiseConfig)
    depth_camera: str = "d435i_depth"
    rgb_camera: str = "d435i_rgb"


class ImuNoiseConfig(BaseModel):
    # Datasheet 1σ start, not calibrated-from-logs.
    accel_sigma: float = 0.01
    gyro_sigma: float = 0.005
    accel_bias_drift: float = 0.0001
    gyro_bias_drift: float = 0.00005


class ImuRateConfig(BaseModel):
    enabled: bool = True
    rate_hz: float = 500.0
    noise: ImuNoiseConfig = Field(default_factory=ImuNoiseConfig)


class ImuBankConfig(BaseModel):
    pelvis: ImuRateConfig = Field(
        default_factory=lambda: ImuRateConfig(enabled=True, rate_hz=500.0)
    )
    torso: ImuRateConfig = Field(
        default_factory=lambda: ImuRateConfig(enabled=True, rate_hz=500.0)
    )
    mid360: ImuRateConfig = Field(
        default_factory=lambda: ImuRateConfig(enabled=True, rate_hz=200.0)
    )
    d435i: ImuRateConfig = Field(
        default_factory=lambda: ImuRateConfig(enabled=True, rate_hz=200.0)
    )


class SensorsConfig(BaseModel):
    mid360: Mid360Config = Field(default_factory=Mid360Config)
    d435i: D435iConfig = Field(default_factory=D435iConfig)
    imu: ImuBankConfig = Field(default_factory=ImuBankConfig)


class ControllerConfig(BaseModel):
    type: Literal["pd", "passthrough"] = "pd"
    physics_hz: float = 1000.0
    control_hz: float = 500.0
    # Optional per-joint scales, canonical names (not integer indices).
    kp_scale: dict[str, float] = Field(default_factory=dict)
    kd_scale: dict[str, float] = Field(default_factory=dict)


class RobotConfig(BaseModel):
    hands: Literal["dex3", "none"] = "dex3"
    spawn_pos: tuple[float, float, float] = (0.0, 0.0, 0.82)
    spawn_quat: tuple[float, float, float, float] = (1.0, 0.0, 0.0, 0.0)


class G1SimulacrumConfig(BaseModel):
    robot: RobotConfig = Field(default_factory=RobotConfig)
    sensors: SensorsConfig = Field(default_factory=SensorsConfig)
    controller: ControllerConfig = Field(default_factory=ControllerConfig)

    @classmethod
    def from_yaml(cls, path: str | Path) -> G1SimulacrumConfig:
        resolved = _resolve_yaml(path)
        with open(resolved) as f:
            data = yaml.safe_load(f) or {}
        return cls.model_validate(data)

    def to_yaml(self, path: str | Path) -> None:
        with open(path, "w") as f:
            yaml.dump(
                self.model_dump(mode="json"),
                f,
                default_flow_style=False,
                sort_keys=False,
            )


def _resolve_yaml(path: str | Path) -> Path:
    p = Path(path)
    if p.is_file():
        return p
    root = Path(__file__).resolve().parents[1]
    cand = root / path
    if cand.is_file():
        return cand
    raise FileNotFoundError(path)
