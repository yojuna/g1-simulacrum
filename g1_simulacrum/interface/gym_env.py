"""Optional Gymnasium wrapper. Not the core API."""

from __future__ import annotations

from typing import Any

import numpy as np

from ..config import G1SimulacrumConfig
from ..simulacrum import G1Simulacrum
from ..sensors.data_types import Observation

try:
    import gymnasium as gym
    from gymnasium import spaces
    HAS_GYM = True
except ImportError:
    HAS_GYM = False


class G1SimulacrumEnv(gym.Env if HAS_GYM else object):
    metadata = {"render_modes": ["rgb_array"], "render_fps": 30}

    def __init__(
        self,
        config: G1SimulacrumConfig | None = None,
        *,
        max_episode_steps: int = 1000,
        obs_keys: list[str] | None = None,
    ) -> None:
        if not HAS_GYM:
            raise ImportError("pip install 'g1-simulacrum[gym]'")
        super().__init__()
        self._config = config or G1SimulacrumConfig()
        self._max_steps = max_episode_steps
        self._obs_keys = obs_keys or ["proprioception", "imu"]
        self.action_space = spaces.Box(
            low=-3.14, high=3.14, shape=(G1Simulacrum.NUM_JOINTS,), dtype=np.float64
        )
        self.observation_space = spaces.Dict(
            {
                "proprioception": spaces.Box(
                    -np.inf, np.inf, shape=(64,), dtype=np.float64
                )
            }
        )
        self._sim: G1Simulacrum | None = None
        self._step_count = 0

    def reset(self, *, seed: int | None = None, options: dict[str, Any] | None = None):
        super().reset(seed=seed)
        if self._sim is None:
            self._sim = G1Simulacrum(config=self._config)
            self._sim.build()
        obs = self._sim.reset()
        self._step_count = 0
        return self._obs_to_dict(obs), {}

    def step(self, action: np.ndarray):
        assert self._sim is not None
        obs = self._sim.step(action)
        self._step_count += 1
        terminated = bool(obs.base_state.position[2] < 0.3)
        truncated = self._step_count >= self._max_steps
        return self._obs_to_dict(obs), 0.0, terminated, truncated, {"sim_time": obs.timestamp}

    def _obs_to_dict(self, obs: Observation) -> dict[str, np.ndarray]:
        result: dict[str, np.ndarray] = {}
        if "proprioception" in self._obs_keys:
            result["proprioception"] = np.concatenate(
                [
                    obs.joint_state.position,
                    obs.joint_state.velocity,
                    obs.base_state.linear_velocity,
                    obs.base_state.angular_velocity,
                ]
            )
        if "imu" in self._obs_keys:
            imu = obs.sensors.imu_pelvis
            result["imu"] = (
                np.concatenate([imu.accel, imu.gyro])
                if imu is not None
                else np.zeros(6)
            )
        return result
