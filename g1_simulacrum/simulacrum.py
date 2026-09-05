"""Top-level facade that composes all layers into a single ``G1Simulacrum``.

This is the main entry point for downstream users.

Usage:
    from g1_simulacrum import G1Simulacrum

    sim = G1Simulacrum.from_config("configs/sonic_v1_1.yaml")
    # or
    sim = G1Simulacrum(controller="sonic", sensors=["mid360", "d435i"])
"""

from __future__ import annotations

import logging
from pathlib import Path
from typing import Sequence

import mujoco
import numpy as np
from numpy.typing import NDArray

from .config import G1SimulacrumConfig
from .controllers.base import Controller
from .model.loader import ModelLoader
from .sensors.data_types import (
    BaseState,
    JointState,
    Observation,
    SensorBundle,
)
from .sensors.manager import SensorManager

logger = logging.getLogger(__name__)


class G1Simulacrum:
    """A fully sensorized Unitree G1 ready to drop into a MuJoCo scene.

    Owns:
        - The composed MJCF model (G1 + sensor mounts)
        - A SensorManager for Mid-360, D435i, IMUs
        - A Controller (SONIC bridge, PD, or passthrough)

    Can be attached to any environment via ``inject_into_scene(scene_xml)``.
    """

    NUM_JOINTS = 29

    def __init__(
        self,
        config: G1SimulacrumConfig | None = None,
        *,
        controller: str = "sonic",
        sensors: Sequence[str] = ("mid360", "d435i"),
    ) -> None:
        if config is None:
            config = G1SimulacrumConfig()
            config.controller.type = controller
            config.sensors.mid360.enabled = "mid360" in sensors
            config.sensors.d435i.enabled = "d435i" in sensors

        self._config = config
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._sensor_manager: SensorManager | None = None
        self._controller: Controller | None = None
        self._previous_action: NDArray[np.float64] | None = None

        # Will be initialized when the model is loaded
        self._initialized = False

    @classmethod
    def from_config(cls, config_path: str | Path) -> G1Simulacrum:
        """Create a G1Simulacrum from a YAML config file."""
        config = G1SimulacrumConfig.from_yaml(config_path)
        return cls(config=config)

    # ------------------------------------------------------------------
    # Model composition
    # ------------------------------------------------------------------

    def build_model(self, scene_xml: str | None = None) -> mujoco.MjModel:
        """Compose the G1 model with sensors and compile it.

        Args:
            scene_xml: Optional path to a scene MJCF to inject the robot into.
                       If None, creates a minimal scene with ground plane.

        Returns:
            The compiled MjModel ready for simulation.
        """
        loader = ModelLoader(self._config)
        self._model = loader.build(scene_xml=scene_xml)
        self._data = mujoco.MjData(self._model)
        self._initialize_subsystems()
        return self._model

    def inject_into_scene(self, scene_xml: str) -> mujoco.MjModel:
        """Load an external scene and place the sensorized G1 in it.

        This is the primary method for environment adapters. It:
        1. Parses the scene MJCF
        2. Injects the G1 body tree with sensor mounts
        3. Adds actuators and sensors
        4. Compiles and returns the combined model
        """
        return self.build_model(scene_xml=scene_xml)

    def _initialize_subsystems(self) -> None:
        """Set up sensors and controller after model compilation."""
        assert self._model is not None and self._data is not None

        # Sensors
        self._sensor_manager = SensorManager(
            self._model, self._data, self._config.sensors
        )

        # Controller
        self._controller = self._create_controller()

        # Set initial pose
        self._set_initial_pose()

        self._initialized = True
        logger.info(
            "G1Simulacrum initialized: %d joints, sensors=%s, controller=%s",
            self.NUM_JOINTS,
            [s for s in self._sensor_manager.sensors],
            self._config.controller.type,
        )

    def _create_controller(self) -> Controller:
        """Instantiate the configured controller."""
        ctrl_type = self._config.controller.type
        if ctrl_type == "sonic":
            from .controllers.sonic_bridge import SonicBridge
            return SonicBridge(self._model, self._data, self._config.controller)
        elif ctrl_type == "pd":
            from .controllers.pd_controller import PDController
            return PDController(self._model, self._data, self._config.controller)
        elif ctrl_type == "passthrough":
            from .controllers.passthrough import PassthroughController
            return PassthroughController(self._model, self._data, self._config.controller)
        else:
            raise ValueError(f"Unknown controller type: {ctrl_type}")

    def _set_initial_pose(self) -> None:
        """Place the robot at its configured spawn position."""
        cfg = self._config.environment
        # Freejoint: qpos[0:3] = position, qpos[3:7] = quaternion
        self._data.qpos[0:3] = cfg.spawn_pos
        self._data.qpos[3:7] = cfg.spawn_quat
        mujoco.mj_forward(self._model, self._data)

    # ------------------------------------------------------------------
    # Simulation step
    # ------------------------------------------------------------------

    def step(self, action: NDArray[np.float64] | None = None) -> Observation:
        """Run one control cycle and collect sensor data.

        Args:
            action: (29,) joint position targets. If None, the controller
                    uses its internal target (e.g. from SONIC DDS commands).

        Returns:
            Full observation including proprioception, sensors, and base state.
        """
        assert self._initialized, "Call build_model() first"
        sim_time = self._data.time

        # Apply action
        if action is not None:
            self._controller.set_targets(action)
        self._controller.step(sim_time)
        self._previous_action = action

        # Step physics (multiple substeps per control step)
        substeps = max(1, int(self._config.controller.physics_hz / self._config.controller.control_hz))
        for _ in range(substeps):
            mujoco.mj_step(self._model, self._data)

        # Collect sensor readings
        sensors = self._sensor_manager.step(self._data.time)

        # Build observation
        return self._build_observation(sensors)

    def _build_observation(self, sensors: SensorBundle) -> Observation:
        """Assemble the full observation from current state."""
        # Joint state
        qpos = self._data.qpos[7:7 + self.NUM_JOINTS].copy()
        qvel = self._data.qvel[6:6 + self.NUM_JOINTS].copy()
        qtau = self._data.actuator_force[:self.NUM_JOINTS].copy()
        joint_state = JointState(
            position=qpos, velocity=qvel, torque=qtau,
            timestamp=self._data.time,
        )

        # Base state
        base_pos = self._data.qpos[0:3].copy()
        base_quat = self._data.qpos[3:7].copy()
        base_linvel = self._data.qvel[0:3].copy()
        base_angvel = self._data.qvel[3:6].copy()
        base_state = BaseState(
            position=base_pos,
            orientation=base_quat,
            linear_velocity=base_linvel,
            angular_velocity=base_angvel,
            timestamp=self._data.time,
        )

        return Observation(
            joint_state=joint_state,
            base_state=base_state,
            sensors=sensors,
            previous_action=self._previous_action,
            timestamp=self._data.time,
        )

    # ------------------------------------------------------------------
    # Accessors
    # ------------------------------------------------------------------

    @property
    def model(self) -> mujoco.MjModel:
        assert self._model is not None
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        assert self._data is not None
        return self._data

    @property
    def sensor_manager(self) -> SensorManager:
        assert self._sensor_manager is not None
        return self._sensor_manager

    @property
    def controller(self) -> Controller:
        assert self._controller is not None
        return self._controller

    @property
    def config(self) -> G1SimulacrumConfig:
        return self._config

    def reset(self) -> Observation:
        """Reset the robot to initial state."""
        assert self._initialized
        mujoco.mj_resetData(self._model, self._data)
        self._set_initial_pose()
        self._sensor_manager.reset()
        self._controller.reset()
        self._previous_action = None
        return self._build_observation(SensorBundle(timestamp=0.0))
