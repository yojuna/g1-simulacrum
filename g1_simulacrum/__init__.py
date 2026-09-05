"""g1-simulacrum: pinned Unitree G1 in MuJoCo (29-DoF body, Dex3, Mid-360, D435i)."""

from .config import G1SimulacrumConfig
from .simulacrum import G1Simulacrum

__all__ = ["G1SimulacrumConfig", "G1Simulacrum"]
__version__ = "0.1.0"
