"""SONIC DDS bridge configuration."""

from __future__ import annotations

from pathlib import Path
from typing import Any, Literal

import yaml
from pydantic import BaseModel, Field

# First 29 entries from gear_sonic g1_29dof_sonic_model12.yaml motor_effort_limit_list
MOTOR_EFFORT_LIMIT_LIST: tuple[float, ...] = (
    88.0, 88.0, 88.0, 139.0, 50.0, 50.0,
    88.0, 88.0, 88.0, 139.0, 50.0, 50.0,
    88.0, 50.0, 50.0,
    25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
    25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
)

# gear_sonic_deploy policy_parameters.hpp default_angles (MuJoCo / SDK motor order).
DEFAULT_STANDING_Q: tuple[float, ...] = (
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    -0.312, 0.0, 0.0, 0.669, -0.363, 0.0,
    0.0, 0.0, 0.0,
    0.2, 0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
    0.2, -0.2, 0.0, 0.6, 0.0, 0.0, 0.0,
)


class DdsConfig(BaseModel):
    domain_id: int = 0
    interface: str = "lo"
    dds_peer: str | None = None


class BridgeConfig(BaseModel):
    robot_type: str = "g1_29dof"
    num_motors: int = 29
    num_hand_motors: int = 7
    use_sensor: bool = False
    use_joystick: int = 0
    joystick_type: str = "xbox"
    joystick_device: int = 0


class LoopConfig(BaseModel):
    control_hz: float = 200.0
    publish_before_step: bool = True
    wait_for_cmd: bool = False
    fall_height: float = 0.2
    gear_parity: bool = False
    onscreen: bool = True
    viewer_dt: float = 0.02
    cameras: bool = False
    # Fraction of the control slot reserved for LowState + mj_step. Remainder
    # may run lidar/depth; if the tick is already late, those frames drop.
    sensor_budget: float = 0.65


class GantryConfig(BaseModel):
    enabled: bool = True
    attach_body: str = "pelvis"
    mode: Literal["cable", "gear"] = "gear"


class SonicDdsConfig(BaseModel):
    dds: DdsConfig = Field(default_factory=DdsConfig)
    bridge: BridgeConfig = Field(default_factory=BridgeConfig)
    loop: LoopConfig = Field(default_factory=LoopConfig)
    gantry: GantryConfig = Field(default_factory=GantryConfig)
    hands: Literal["dex3", "none"] = "dex3"

    @classmethod
    def from_yaml(cls, path: str | Path) -> SonicDdsConfig:
        p = Path(path)
        if not p.is_file():
            root = Path(__file__).resolve().parents[2]
            p = root / path
        with open(p) as f:
            data = yaml.safe_load(f) or {}
        return cls.model_validate(data)


def bridge_config_dict(cfg: SonicDdsConfig) -> dict[str, Any]:
    """Dict for UnitreeSdk2Bridge and init_channel (GEAR WBC key names)."""
    return {
        "DOMAIN_ID": cfg.dds.domain_id,
        "INTERFACE": cfg.dds.interface,
        "DDS_PEER": cfg.dds.dds_peer,
        "ROBOT_TYPE": cfg.bridge.robot_type,
        "NUM_MOTORS": cfg.bridge.num_motors,
        "NUM_HAND_MOTORS": cfg.bridge.num_hand_motors,
        "USE_SENSOR": cfg.bridge.use_sensor,
        "USE_JOYSTICK": cfg.bridge.use_joystick,
        "JOYSTICK_TYPE": cfg.bridge.joystick_type,
        "JOYSTICK_DEVICE": cfg.bridge.joystick_device,
        "motor_effort_limit_list": list(MOTOR_EFFORT_LIMIT_LIST),
    }
