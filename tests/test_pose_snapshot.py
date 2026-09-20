"""Tests for Phase 2 pose snapshot buffer."""

from __future__ import annotations

import sys
from pathlib import Path

import mujoco
import numpy as np

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.pose_snapshot import PoseSnapshotBuffer, apply_snapshot  # noqa: E402
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig  # noqa: E402


def _minimal_sim() -> G1Simulacrum:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    return sim


def test_pose_snapshot_publish_increments_seq() -> None:
    sim = _minimal_sim()
    buf = PoseSnapshotBuffer()
    s1 = buf.publish(sim.data)
    s2 = buf.publish(sim.data)
    assert s2.seq == s1.seq + 1
    assert buf.seq == s2.seq


def test_apply_snapshot_updates_site_position() -> None:
    sim = _minimal_sim()
    worker = mujoco.MjData(sim.model)
    buf = PoseSnapshotBuffer()
    sim.data.qpos[2] += 0.05
    mujoco.mj_forward(sim.model, sim.data)
    snap = buf.publish(sim.data)

    apply_snapshot(sim.model, worker, snap)
    site_id = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_SITE, "mid360")
    assert site_id >= 0
    np.testing.assert_allclose(
        worker.site_xpos[site_id],
        sim.data.site_xpos[site_id],
        rtol=0,
        atol=1e-9,
    )
