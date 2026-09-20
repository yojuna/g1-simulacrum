"""Tests for SensorWorkerGroup (no DDS / SonicDdsSimLoop required)."""

from __future__ import annotations

import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.sensor_worker_group import SensorWorkerGroup  # noqa: E402
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig  # noqa: E402


def _sensorized_sim() -> G1Simulacrum:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = True
    cfg.sensors.d435i.enabled = True
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    return sim


def test_sensor_worker_group_produces_lidar_and_depth() -> None:
    sim = _sensorized_sim()
    group = SensorWorkerGroup.from_configs(
        sim.model,
        sim.config.sensors.mid360,
        sim.config.sensors.d435i,
    )
    group.start()
    try:
        deadline = time.monotonic() + 6.0
        got_lidar = False
        got_depth = False
        while time.monotonic() < deadline:
            group.publish(sim.data)
            poll = group.poll()
            got_lidar = got_lidar or poll.cloud is not None
            got_depth = got_depth or poll.depth is not None
            if got_lidar and got_depth:
                assert poll.refreshed
                assert poll.lidar_frames == 1 or poll.depth_frames == 1
                return
            time.sleep(0.02)
        raise AssertionError(
            f"SensorWorkerGroup incomplete: lidar={got_lidar} depth={got_depth}"
        )
    finally:
        group.stop()


def test_sensor_worker_poll_idempotent_without_new_frames() -> None:
    sim = _sensorized_sim()
    group = SensorWorkerGroup.from_configs(
        sim.model,
        sim.config.sensors.mid360,
        sim.config.sensors.d435i,
    )
    first = group.poll()
    second = group.poll()
    assert first.cloud is None and first.depth is None
    assert second.cloud is None and second.depth is None
    assert not second.refreshed
