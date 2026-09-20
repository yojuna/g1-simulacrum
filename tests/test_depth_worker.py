"""Tests for Phase 2 dedicated depth worker."""

from __future__ import annotations

import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.depth_worker import DepthWorker  # noqa: E402
from extras.sonic_dds.pose_snapshot import PoseSnapshotBuffer  # noqa: E402
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig  # noqa: E402


def _sensorized_sim() -> G1Simulacrum:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = False
    cfg.sensors.d435i.enabled = True
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    return sim


def test_depth_worker_produces_frame() -> None:
    sim = _sensorized_sim()
    buf = PoseSnapshotBuffer()
    worker = DepthWorker(sim.model, sim.config.sensors.d435i, buf, rate_hz=15.0)
    worker.start()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            buf.publish(sim.data)
            if worker.last_frame_seq() > 0:
                depth = worker.get_last_depth()
                assert depth is not None
                assert depth.depth.shape[0] > 0
                assert depth.rgb.shape[0] > 0
                assert worker.stats().frames >= 1
                return
            time.sleep(0.02)
        raise AssertionError("depth worker did not produce a frame within 5s")
    finally:
        worker.stop()
