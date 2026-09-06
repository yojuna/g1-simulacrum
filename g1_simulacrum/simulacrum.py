"""G1Simulacrum facade: build / reset / step → Observation."""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
from numpy.typing import NDArray

from .config import G1SimulacrumConfig
from .controllers.base import Controller
from .controllers.passthrough import PassthroughController
from .controllers.pd import PDController
from .model.joints import NUM_BODY_JOINTS
from .model.loader import CompiledModel, ModelLoader
from .sensors.data_types import BaseState, JointState, Observation, SensorBundle
from .sensors.manager import SensorManager


class G1Simulacrum:
    NUM_JOINTS = NUM_BODY_JOINTS

    def __init__(self, config: G1SimulacrumConfig | None = None) -> None:
        self._config = config or G1SimulacrumConfig()
        self._compiled: CompiledModel | None = None
        self._model: mujoco.MjModel | None = None
        self._data: mujoco.MjData | None = None
        self._sensor_manager: SensorManager | None = None
        self._controller: Controller | None = None
        self._previous_action: NDArray[np.float64] | None = None
        self._initialized = False

    @classmethod
    def from_config(cls, config_path: str | Path) -> G1Simulacrum:
        return cls(config=G1SimulacrumConfig.from_yaml(config_path))

    def build(self, scene_xml: str | Path | None = None) -> mujoco.MjModel:
        """Compile pinned MJCF. ``scene_xml`` must already ``<include>`` g1_robot.xml."""
        loader = ModelLoader(
            hands=self._config.robot.hands,
            xml_path=scene_xml,
        )
        self._compiled = loader.build()
        self._model = self._compiled.model
        self._data = mujoco.MjData(self._model)
        self._sensor_manager = SensorManager(
            self._model, self._data, self._config.sensors
        )
        self._controller = self._create_controller()
        self._set_spawn()
        self._initialized = True
        return self._model

    def _create_controller(self) -> Controller:
        assert self._compiled is not None and self._data is not None
        ctrl = self._config.controller.type
        if ctrl == "pd":
            return PDController(self._compiled, self._data, self._config.controller)
        if ctrl == "passthrough":
            return PassthroughController(
                self._compiled, self._data, self._config.controller
            )
        raise ValueError(f"unknown controller {ctrl!r}")

    def _set_spawn(self) -> None:
        assert self._data is not None
        robot = self._config.robot
        self._data.qpos[0:3] = robot.spawn_pos
        self._data.qpos[3:7] = robot.spawn_quat
        mujoco.mj_forward(self._model, self._data)
        if self._controller is not None:
            self._controller.reset()

    def step(self, action: NDArray[np.float64] | None = None) -> Observation:
        """One 500 Hz control cycle. ``action`` is (29,) or None to hold last target."""
        assert self._initialized
        prev = self._previous_action
        if action is not None:
            self._controller.set_targets(action)
        self._controller.step(self._data.time)

        physics_hz = self._config.controller.physics_hz
        control_hz = self._config.controller.control_hz
        substeps = max(1, int(round(physics_hz / control_hz)))
        for _ in range(substeps):
            mujoco.mj_step(self._model, self._data)

        sensors = self._sensor_manager.step(self._data.time)
        obs = self._build_observation(sensors, previous_action=prev)
        self._previous_action = (
            np.asarray(action, dtype=np.float64).copy()
            if action is not None
            else prev
        )
        return obs

    def reset(self) -> Observation:
        assert self._initialized
        mujoco.mj_resetData(self._model, self._data)
        self._set_spawn()
        self._sensor_manager.reset()
        self._previous_action = None
        return self._build_observation(
            SensorBundle(timestamp=0.0), previous_action=None
        )

    def _build_observation(
        self,
        sensors: SensorBundle,
        *,
        previous_action: NDArray[np.float64] | None,
    ) -> Observation:
        ctrl = self._controller
        joint_state = JointState(
            position=ctrl.body_qpos(),
            velocity=ctrl.body_qvel(),
            torque=ctrl.body_tau(),
            timestamp=self._data.time,
        )
        base_state = BaseState(
            position=self._data.qpos[0:3].copy(),
            orientation=self._data.qpos[3:7].copy(),
            linear_velocity=self._data.qvel[0:3].copy(),
            angular_velocity=self._data.qvel[3:6].copy(),
            timestamp=self._data.time,
        )
        return Observation(
            joint_state=joint_state,
            base_state=base_state,
            sensors=sensors,
            previous_action=previous_action,
            q_hands=ctrl.hand_qpos(),
            timestamp=self._data.time,
        )

    @property
    def model(self) -> mujoco.MjModel:
        assert self._model is not None
        return self._model

    @property
    def data(self) -> mujoco.MjData:
        assert self._data is not None
        return self._data

    @property
    def controller(self) -> Controller:
        assert self._controller is not None
        return self._controller

    @property
    def compiled(self) -> CompiledModel:
        assert self._compiled is not None
        return self._compiled

    @property
    def config(self) -> G1SimulacrumConfig:
        return self._config
