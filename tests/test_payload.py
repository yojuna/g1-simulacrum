"""Rigid hand payload application and persistence."""

from __future__ import annotations

import mujoco
import numpy as np
import pytest

from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig
from g1_simulacrum.model.payload import cuboid_diagonal_inertia


def _fast_config() -> G1SimulacrumConfig:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    cfg.sensors.imu.pelvis.enabled = False
    cfg.sensors.imu.torso.enabled = False
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    return cfg


def test_zero_payload_is_mass_neutral_and_named() -> None:
    sim = G1Simulacrum(config=_fast_config())
    model = sim.build()
    applied = sim.applied_payloads
    assert applied.left.mass_kg == 0.0
    assert applied.right.mass_kg == 0.0
    assert applied.model_total_mass_kg == pytest.approx(
        mujoco.mj_getTotalmass(model), abs=1e-12
    )
    for hand in (applied.left, applied.right):
        assert hand.body_id >= 0
        assert model.body_mass[hand.body_id] == 0.0
        assert np.all(model.body_inertia[hand.body_id] == 0.0)


def test_payload_mass_com_inertia_and_reset_persist() -> None:
    baseline = G1Simulacrum(config=_fast_config())
    baseline.build()
    baseline_mass = baseline.applied_payloads.model_total_mass_kg

    cfg = _fast_config()
    cfg.robot.payloads.left.mass_kg = 2.5
    cfg.robot.payloads.right.mass_kg = 1.0
    sim = G1Simulacrum(config=cfg)
    model = sim.build()
    applied = sim.applied_payloads
    assert applied.model_total_mass_kg - baseline_mass == pytest.approx(3.5, abs=1e-10)

    for hand, spec in (
        (applied.left, cfg.robot.payloads.left),
        (applied.right, cfg.robot.payloads.right),
    ):
        np.testing.assert_allclose(model.body_pos[hand.body_id], spec.com_pos_wrist_m)
        np.testing.assert_allclose(
            model.body_inertia[hand.body_id],
            cuboid_diagonal_inertia(spec.mass_kg, spec.size_m),
            rtol=0.0,
            atol=1e-12,
        )
        np.testing.assert_allclose(
            model.geom_size[hand.geom_id],
            np.asarray(spec.size_m) / 2.0,
            rtol=0.0,
            atol=1e-12,
        )

    sim.reset()
    assert model.body_mass[applied.left.body_id] == pytest.approx(2.5)
    assert model.body_mass[applied.right.body_id] == pytest.approx(1.0)


def test_bilateral_five_kg_steps_with_finite_state() -> None:
    cfg = _fast_config()
    cfg.robot.payloads.left.mass_kg = 5.0
    cfg.robot.payloads.right.mass_kg = 5.0
    sim = G1Simulacrum(config=cfg)
    sim.build()
    obs = sim.reset()
    target = obs.joint_state.position.copy()
    for _ in range(100):
        obs = sim.step(target)
    assert np.isfinite(sim.data.qpos).all()
    assert np.isfinite(sim.data.qvel).all()
    assert np.isfinite(obs.joint_state.torque).all()
