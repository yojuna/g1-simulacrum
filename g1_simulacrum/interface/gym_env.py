"""Gymnasium-compatible environment wrapping the full G1 sim stack.

This lets any RL framework (Stable-Baselines3, CleanRL, rl_games, RSL-RL, etc.)
train policies against the sensorized G1 with a standard Gym interface.
"""

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


def _make_observation_space(config: G1SimulacrumConfig) -> "spaces.Dict":
    """Build the observation space from config."""
    obs_keys = config.interface.gym.obs_keys
    d: dict[str, spaces.Space] = {}

    if "proprioception" in obs_keys:
        # 29 joint pos + 29 joint vel + 3 base linvel + 3 base angvel = 64
        d["proprioception"] = spaces.Box(
            low=-np.inf, high=np.inf, shape=(64,), dtype=np.float64
        )

    if "lidar" in obs_keys and config.sensors.mid360.enabled:
        # Variable-length point cloud; use a fixed max and pad
        max_points = 25000  # Mid-360 ~20k points/scan
        d["lidar"] = spaces.Box(
            low=-100.0, high=100.0, shape=(max_points, 3), dtype=np.float32
        )

    if "depth" in obs_keys and config.sensors.d435i.enabled:
        h, w = config.sensors.d435i.resolution[1], config.sensors.d435i.resolution[0]
        d["depth"] = spaces.Box(
            low=0.0, high=10.0, shape=(h, w), dtype=np.float32
        )

    if "rgb" in obs_keys and config.sensors.d435i.enabled:
        h, w = config.sensors.d435i.resolution[1], config.sensors.d435i.resolution[0]
        d["rgb"] = spaces.Box(
            low=0, high=255, shape=(h, w, 3), dtype=np.uint8
        )

    if "imu" in obs_keys and config.sensors.imu.enabled:
        d["imu"] = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float64
        )

    return spaces.Dict(d)


class G1SimulacrumEnv(gym.Env if HAS_GYM else object):
    """Gymnasium environment for the sensorized Unitree G1.

    Action space: Box(29,) — joint position targets for PD controller.
    Observation space: Dict with configurable keys.

    Usage:
        env = G1SimulacrumEnv(config=G1SimulacrumConfig.from_yaml("configs/default.yaml"))
        obs, info = env.reset()
        for _ in range(1000):
            action = env.action_space.sample()
            obs, reward, terminated, truncated, info = env.step(action)
    """

    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 30}

    def __init__(
        self,
        config: G1SimulacrumConfig | None = None,
        render_mode: str | None = None,
        scene_xml: str | None = None,
    ) -> None:
        if not HAS_GYM:
            raise ImportError(
                "gymnasium is required for G1SimulacrumEnv. "
                "Install with: pip install 'g1-simulacrum[gym]'"
            )
        super().__init__()

        self._config = config or G1SimulacrumConfig()
        self._scene_xml = scene_xml
        self.render_mode = render_mode

        # Spaces
        self.action_space = spaces.Box(
            low=-3.14, high=3.14,
            shape=(G1Simulacrum.NUM_JOINTS,),
            dtype=np.float64,
        )
        self.observation_space = _make_observation_space(self._config)

        # Sim (initialized on first reset)
        self._sim: G1Simulacrum | None = None
        self._step_count = 0
        self._max_steps = self._config.interface.gym.max_episode_steps

        # Renderer for rgb_array mode
        self._viewer_renderer = None

    def reset(
        self, *, seed: int | None = None, options: dict[str, Any] | None = None
    ) -> tuple[dict, dict]:
        super().reset(seed=seed)
        if self._sim is None:
            self._sim = G1Simulacrum(config=self._config)
            self._sim.build_model(scene_xml=self._scene_xml)

        obs = self._sim.reset()
        self._step_count = 0
        return self._obs_to_dict(obs), {}

    def step(
        self, action: np.ndarray
    ) -> tuple[dict, float, bool, bool, dict]:
        assert self._sim is not None
        obs = self._sim.step(action)
        self._step_count += 1

        reward = self._compute_reward(obs, action)
        terminated = self._check_termination(obs)
        truncated = self._step_count >= self._max_steps

        info = {
            "sim_time": obs.timestamp,
            "base_height": obs.base_state.position[2],
        }

        return self._obs_to_dict(obs), reward, terminated, truncated, info

    def render(self) -> np.ndarray | None:
        if self.render_mode == "rgb_array" and self._sim is not None:
            if self._viewer_renderer is None:
                import mujoco
                self._viewer_renderer = mujoco.Renderer(
                    self._sim.model, height=480, width=640
                )
            self._viewer_renderer.update_scene(self._sim.data)
            return self._viewer_renderer.render()
        return None

    def close(self) -> None:
        if self._viewer_renderer is not None:
            self._viewer_renderer.close()
            self._viewer_renderer = None

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------

    def _obs_to_dict(self, obs: Observation) -> dict[str, np.ndarray]:
        """Convert Observation dataclass to flat dict matching obs space."""
        result: dict[str, np.ndarray] = {}
        keys = self._config.interface.gym.obs_keys

        if "proprioception" in keys:
            result["proprioception"] = np.concatenate([
                obs.joint_state.position,
                obs.joint_state.velocity,
                obs.base_state.linear_velocity,
                obs.base_state.angular_velocity,
            ])

        if "lidar" in keys and obs.sensors.lidar is not None:
            pc = obs.sensors.lidar.points
            max_pts = self.observation_space["lidar"].shape[0]
            padded = np.zeros((max_pts, 3), dtype=np.float32)
            n = min(len(pc), max_pts)
            padded[:n] = pc[:n]
            result["lidar"] = padded

        if "depth" in keys and obs.sensors.depth is not None:
            result["depth"] = obs.sensors.depth.depth

        if "rgb" in keys and obs.sensors.depth is not None:
            result["rgb"] = obs.sensors.depth.rgb

        if "imu" in keys and obs.sensors.imu is not None:
            result["imu"] = np.concatenate([
                obs.sensors.imu.accel,
                obs.sensors.imu.gyro,
            ])

        # Fill any missing optional keys with zeros
        for key in keys:
            if key not in result and key in self.observation_space.spaces:
                result[key] = np.zeros(
                    self.observation_space[key].shape,
                    dtype=self.observation_space[key].dtype,
                )

        return result

    def _compute_reward(self, obs: Observation, action: np.ndarray) -> float:
        """Default reward: stay alive and upright."""
        base_height = obs.base_state.position[2]
        upright_bonus = max(0.0, base_height - 0.3)  # above 0.3m
        action_penalty = -0.001 * np.sum(action ** 2)
        return float(upright_bonus + action_penalty)

    def _check_termination(self, obs: Observation) -> bool:
        """Terminate if robot falls."""
        return obs.base_state.position[2] < 0.3
