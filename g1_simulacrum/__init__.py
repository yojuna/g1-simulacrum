"""g1-simulacrum: Modular MuJoCo simulation for Unitree G1 with full sensor suite."""

from .config import G1SimulacrumConfig
from .simulacrum import G1Simulacrum

__all__ = ["G1SimulacrumConfig", "G1Simulacrum"]
__version__ = "0.1.0"
