"""Unit tests for SONIC DDS adapter (no live DDS)."""

from __future__ import annotations

import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import pytest

from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig
from g1_simulacrum.model.joints import BODY_JOINT_NAMES, NUM_BODY_JOINTS

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.lowcmd_actuation import compute_body_torques  # noqa: E402
from extras.sonic_dds.observation_adapter import prepare_obs_dict  # noqa: E402


def _minimal_sim() -> G1Simulacrum:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    return sim


def test_prepare_obs_dict_shapes_and_body_order() -> None:
    sim = _minimal_sim()
    obs = prepare_obs_dict(sim.compiled, sim.data)

    assert obs["floating_base_pose"].shape == (7,)
    assert obs["floating_base_vel"].shape == (6,)
    assert obs["floating_base_acc"].shape == (6,)
    assert obs["secondary_imu_quat"].shape == (4,)
    assert obs["secondary_imu_vel"].shape == (6,)
    assert obs["body_q"].shape == (NUM_BODY_JOINTS,)
    assert obs["left_hand_q"].shape == (7,)
    assert obs["right_hand_q"].shape == (7,)

    for i, name in enumerate(BODY_JOINT_NAMES):
        adr = sim.compiled.body_qposadr[name]
        assert obs["body_q"][i] == pytest.approx(sim.data.qpos[adr])


def test_prepare_obs_dict_hand_idl_order() -> None:
    sim = _minimal_sim()
    obs = prepare_obs_dict(sim.compiled, sim.data)
    left_thumb = sim.compiled.hand_qposadr["left_hand_thumb_0"]
    assert obs["left_hand_q"][0] == pytest.approx(sim.data.qpos[left_thumb])


@dataclass
class _MotorCmd:
    q: float = 0.0
    dq: float = 0.0
    tau: float = 0.0
    kp: float = 0.0
    kd: float = 0.0


class _FakeLowCmd:
    def __init__(self, n: int) -> None:
        self.motor_cmd = [_MotorCmd() for _ in range(n)]


class _FakeBridge:
    def __init__(self) -> None:
        self.low_cmd = _FakeLowCmd(NUM_BODY_JOINTS)
        self.low_cmd_received = True

    def cmd_received(self) -> bool:
        return self.low_cmd_received


def test_compute_body_torques_pd_formula() -> None:
    sim = _minimal_sim()
    bridge = _FakeBridge()
    bridge.low_cmd.motor_cmd[0].q = 0.5
    bridge.low_cmd.motor_cmd[0].kp = 100.0
    bridge.low_cmd.motor_cmd[0].kd = 2.0
    bridge.low_cmd.motor_cmd[0].tau = 1.0

    name = BODY_JOINT_NAMES[0]
    q = sim.data.qpos[sim.compiled.body_qposadr[name]]
    dq = sim.data.qvel[sim.compiled.body_dofadr[name]]
    expected = 1.0 + 100.0 * (0.5 - q) + 2.0 * (0.0 - dq)

    torques = compute_body_torques(sim.compiled, sim.data, bridge)
    assert torques[0] == pytest.approx(expected)


def test_step_physics_without_controller_write() -> None:
    sim = _minimal_sim()
    sim.controller.zero_body_ctrl()
    t0 = sim.data.time
    sim.step_physics()
    assert sim.data.time > t0


def test_apply_gear_parity_keeps_default_timestep_and_syncs_loop_hz() -> None:
    from extras.sonic_dds.sim_loop import apply_gear_parity
    from extras.sonic_dds.wbc_config import SonicDdsConfig

    sim_cfg = G1SimulacrumConfig()
    sonic = SonicDdsConfig()
    sonic.loop.control_hz = 500.0
    apply_gear_parity(sim_cfg, sonic)
    assert sim_cfg.controller.physics_hz == 1000.0
    assert sim_cfg.controller.control_hz == 200.0
    assert sonic.loop.control_hz == 200.0
    assert sonic.gantry.mode == "gear"
    assert sonic.gantry.attach_body == "pelvis"
    assert sonic.loop.cameras is True


def test_step_physics_sensors_false_does_not_call_manager(monkeypatch) -> None:
    from g1_simulacrum.sensors.data_types import SensorBundle

    sim = _minimal_sim()
    called: list[float] = []

    def _step(sim_time: float) -> SensorBundle:
        called.append(sim_time)
        return SensorBundle(timestamp=sim_time)

    monkeypatch.setattr(sim.sensor_manager, "step", _step)
    sim.step_physics(sensors=False)
    assert called == []
    sim.step_physics(sensors=True)
    assert len(called) == 1


def test_sensor_manager_lidar_depth_hooks_when_disabled() -> None:
    sim = _minimal_sim()
    mgr = sim.sensor_manager
    t = float(sim.data.time)
    assert mgr.lidar_due(t) is False
    assert mgr.depth_due(t) is False
    assert mgr.step_lidar(t) is None
    assert mgr.step_depth(t) is None


def test_overlay_presets_dense_is_default() -> None:
    from extras.sonic_dds.overlay import OverlayConfig, overlay_from_preset

    dense = overlay_from_preset("dense")
    default = OverlayConfig()
    assert dense.lidar_dots == default.lidar_dots == 0
    assert dense.depth_stride == default.depth_stride == 4
    assert overlay_from_preset("sparse").lidar_dots == 1800
    assert overlay_from_preset("full").depth_stride == 2


def test_compute_body_torques_clips_q_des_to_joint_range() -> None:
    sim = _minimal_sim()
    name = BODY_JOINT_NAMES[0]
    jid = sim.compiled.body_joint_ids[name]
    if not sim.compiled.model.jnt_limited[jid]:
        pytest.skip("hip pitch is not limited in this MJCF")
    lo, hi = sim.compiled.model.jnt_range[jid]
    bridge = _FakeBridge()
    bridge.low_cmd.motor_cmd[0].q = float(hi) + 10.0
    bridge.low_cmd.motor_cmd[0].kp = 100.0
    bridge.low_cmd.motor_cmd[0].kd = 2.0
    q = sim.data.qpos[sim.compiled.body_qposadr[name]]
    dq = sim.data.qvel[sim.compiled.body_dofadr[name]]
    expected = 100.0 * (float(hi) - q) + 2.0 * (0.0 - dq)
    torques = compute_body_torques(sim.compiled, sim.data, bridge)
    assert torques[0] == pytest.approx(expected)
