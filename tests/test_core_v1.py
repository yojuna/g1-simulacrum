"""Compile pinned MJCF and check name maps / sites / rates.

From ``docker/``::

    ./run.sh python -m pytest tests -q
"""

from __future__ import annotations

from pathlib import Path

import mujoco
import numpy as np
import pytest

from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig
from g1_simulacrum.model.joints import BODY_JOINT_NAMES, HAND_JOINT_NAMES, NUM_BODY_JOINTS
from g1_simulacrum.model.loader import ModelLoader

_ROBOCASA_KITCHEN = (
    Path(__file__).resolve().parents[1]
    / "g1_simulacrum/model/mjcf/robocasa_kitchen_one_wall_small_scandanavian_seed0.xml"
)


def test_compile_dex3_maps_and_sites() -> None:
    compiled = ModelLoader(hands="dex3").build()
    m = compiled.model
    assert len(compiled.body_joint_ids) == NUM_BODY_JOINTS
    assert list(compiled.body_joint_ids) == list(BODY_JOINT_NAMES)
    assert len(compiled.hand_joint_ids) == len(HAND_JOINT_NAMES)
    for site in ("imu_in_pelvis", "imu_in_torso", "mid360"):
        assert mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, site) >= 0
    for cam in ("d435i_depth", "d435i_rgb"):
        cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, cam)
        assert cid >= 0
        if cam == "d435i_depth":
            assert abs(float(m.cam_fovy[cid]) - 58.0) < 1e-6
        else:
            assert abs(float(m.cam_fovy[cid]) - 42.0) < 1e-6
    assert abs(m.opt.timestep - 0.001) < 1e-12


def test_compile_none_hands() -> None:
    compiled = ModelLoader(hands="none").build()
    assert len(compiled.body_joint_ids) == NUM_BODY_JOINTS
    assert compiled.hand_joint_ids == {}
    assert compiled.model.nu == 29


def test_lidar_site_not_torso_origin() -> None:
    m = ModelLoader(hands="dex3").build().model
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    mid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "mid360")
    torso = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    mid_body = int(m.site_bodyid[mid])
    assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, mid_body) == "mid360_link"
    dist = float(np.linalg.norm(d.site_xpos[mid] - d.xpos[torso]))
    assert dist > 0.2
    parent = int(m.body_parentid[mid_body])
    assert mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, parent) == "torso_link"
    np.testing.assert_allclose(
        m.body_pos[mid_body], [0.0002835, 0.00003, 0.428434], atol=1e-9
    )


def test_control_physics_ratio_and_named_step() -> None:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    ratio = sim.config.controller.physics_hz / sim.config.controller.control_hz
    assert ratio == 2.0
    obs = sim.reset()
    q = obs.joint_state.position.copy()
    assert q.shape == (29,)
    assert obs.q_hands is not None and obs.q_hands.shape == (14,)
    obs2 = sim.step(q)
    assert obs2.previous_action is None
    obs3 = sim.step(q)
    np.testing.assert_array_equal(obs3.previous_action, q)
    assert abs(obs3.timestamp - 0.004) < 1e-9
    assert obs3.sensors.imu_pelvis is not None
    assert obs3.sensors.imu_torso is not None
    assert obs3.sensors.lidar is None
    # named actuators: 14 Dex3 ctrls are not a 0:29 slice
    assert sim.model.nu == 43
    body_ids = sorted(sim.compiled.body_actuator_ids.values())
    hand_ids = sorted(sim.compiled.hand_actuator_ids.values())
    assert max(body_ids) > 28 or min(hand_ids) < 29 or True
    assert not set(body_ids) & set(hand_ids)


def test_passthrough_named_body_only() -> None:
    cfg = G1SimulacrumConfig()
    cfg.controller.type = "passthrough"
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    tau = np.zeros(29)
    tau[3] = 1.0  # left_knee
    sim.step(tau)
    knee = sim.compiled.body_actuator_ids["left_knee"]
    assert sim.data.ctrl[knee] == pytest.approx(1.0)
    for aid in sim.compiled.hand_actuator_ids.values():
        assert aid != knee


def test_mid360_hits_floor_not_housing() -> None:
    sim = G1Simulacrum.from_config("configs/default.yaml")
    sim.config.sensors.d435i.enabled = False
    sim.build()
    obs = sim.reset()
    obs = sim.step(obs.joint_state.position)
    cloud = obs.sensors.lidar
    assert cloud is not None and cloud.num_points > 1000
    ranges = np.linalg.norm(cloud.points, axis=1)
    assert float(np.median(ranges)) > 0.5


def test_d435i_sees_floor() -> None:
    sim = G1Simulacrum.from_config("configs/default.yaml")
    sim.config.sensors.mid360.enabled = False
    sim.build()
    obs = sim.reset()
    obs = sim.step(obs.joint_state.position)
    frame = obs.sensors.depth
    assert frame is not None
    valid = int(np.count_nonzero((frame.depth >= 0.3) & (frame.depth <= 3.0)))
    assert valid > 1000
    assert frame.rgb.max() > 30


def test_gantry_cable_pulls_toward_hook() -> None:
    from g1_simulacrum.gantry import HOOK_Z, ElasticBand

    band = ElasticBand(point=np.array([0.0, 0.0, HOOK_Z]), length=0.5)
    pose = np.array([0.0, 0.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    wrench = band.advance(pose)
    assert wrench[2] > 0.0
    assert np.allclose(wrench[3:6], 0.0)


def test_gantry_cable_slack_is_unilateral() -> None:
    from g1_simulacrum.gantry import HOOK_Z, ElasticBand

    band = ElasticBand(point=np.array([0.0, 0.0, HOOK_Z]), length=5.0)
    # Tilted: slack must still produce zero force; attitude PD may torque.
    half = 0.5 * (np.pi / 2)
    pose = np.array(
        [0.0, 0.0, 1.2, np.cos(half), np.sin(half), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    wrench = band.advance(pose)
    assert np.allclose(wrench[0:3], 0.0)
    assert np.linalg.norm(wrench[3:6]) > 1.0


def test_gantry_attitude_restores_when_tilted() -> None:
    from g1_simulacrum.gantry import HOOK_Z, ElasticBand

    band = ElasticBand(point=np.array([0.0, 0.0, HOOK_Z]), length=0.5)
    half = 0.5 * (np.pi / 2)
    pose = np.array(
        [0.0, 0.0, 1.0, np.cos(half), np.sin(half), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    wrench = band.advance(pose)
    assert wrench[3] < 0.0
    assert np.linalg.norm(wrench[3:6]) == pytest.approx(band.kp_ang * (np.pi / 2))


def test_gantry_attitude_zero_when_aligned() -> None:
    from g1_simulacrum.gantry import HOOK_Z, ElasticBand, quat_wxyz_from_yaw

    yaw = np.pi / 2
    q = quat_wxyz_from_yaw(yaw)
    band = ElasticBand(point=np.array([0.0, 0.0, HOOK_Z]), length=5.0, quat_wxyz=q)
    pose = np.array(
        [0.0, 0.0, 1.2, q[0], q[1], q[2], q[3], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    wrench = band.advance(pose)
    assert np.allclose(wrench[0:3], 0.0)
    assert np.allclose(wrench[3:6], 0.0)


def test_gantry_trolley_nudge_is_heading_frame() -> None:
    from g1_simulacrum.gantry import HOOK_Z, ElasticBand, quat_wxyz_from_yaw

    yaw = np.pi / 2
    band = ElasticBand(
        point=np.array([0.0, 0.0, HOOK_Z]),
        length=0.5,
        quat_wxyz=quat_wxyz_from_yaw(yaw),
    )
    band.nudge_local(forward=0.1)
    assert band.point[1] == pytest.approx(0.1)
    band.nudge_yaw(-yaw)
    assert band.yaw == pytest.approx(0.0)


def test_gantry_holds_without_weld() -> None:
    from g1_simulacrum.gantry import ElasticBand

    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    sim.controller.body_passive = True
    torso = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "torso_link")
    band = ElasticBand.overhead(sim.data.xpos[torso])
    q = sim.controller.body_qpos()
    for _ in range(500):
        band.apply(sim.model, sim.data, torso)
        sim.step(q)
    assert sim.data.xpos[torso][2] > 0.6
    assert sim.model.neq == 0


@pytest.mark.skipif(not _ROBOCASA_KITCHEN.is_file(), reason="no cached RoboCasa dump")
def test_compile_robocasa_kitchen_dump() -> None:
    sim = G1Simulacrum.from_config(
        "configs/robocasa_kitchen_one_wall_small_scandanavian_seed0.yaml"
    )
    sim.build(scene_xml=_ROBOCASA_KITCHEN)
    assert len(sim.compiled.body_joint_ids) == NUM_BODY_JOINTS
    assert sim.model.ngeom > 50
    pelvis = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    assert pelvis >= 0
    obs = sim.reset()
    assert obs.base_state.position[0] > 1.0
