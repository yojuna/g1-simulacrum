"""Optional SONIC DDS bridge — compose with G1Simulacrum, not core."""

from __future__ import annotations

from typing import TYPE_CHECKING

from .wbc_config import SonicDdsConfig, bridge_config_dict

if TYPE_CHECKING:
    from .sim_loop import SonicDdsSimLoop

__all__ = ["SonicDdsSimLoop", "SonicDdsConfig", "bridge_config_dict"]


def __getattr__(name: str):
    if name == "SonicDdsSimLoop":
        from .sim_loop import SonicDdsSimLoop

        return SonicDdsSimLoop
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
