"""Tests for Phase 2 dedicated lidar worker."""

from __future__ import annotations

import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.lidar_worker import LidarWorker  # noqa: E402
from extras.sonic_dds.pose_snapshot import PoseSnapshotBuffer  # noqa: E402
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig  # noqa: E402


def _sensorized_sim() -> G1Simulacrum:
    cfg = G1SimulacrumConfig()
    cfg.sensors.mid360.enabled = True
    cfg.sensors.d435i.enabled = False
    cfg.sensors.imu.mid360.enabled = False
    cfg.sensors.imu.d435i.enabled = False
    sim = G1Simulacrum(config=cfg)
    sim.build()
    sim.reset()
    return sim


def test_lidar_worker_produces_cloud() -> None:
    sim = _sensorized_sim()
    buf = PoseSnapshotBuffer()
    worker = LidarWorker(sim.model, sim.config.sensors.mid360, buf, rate_hz=20.0)
    worker.start()
    try:
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            buf.publish(sim.data)
            if worker.last_scan_seq() > 0:
                cloud = worker.get_last_cloud()
                assert cloud is not None
                assert cloud.num_points > 0
                assert worker.stats().scans >= 1
                return
            time.sleep(0.02)
        raise AssertionError("lidar worker did not produce a cloud within 3s")
    finally:
        worker.stop()


def test_sim_loop_sensor_workers_flag() -> None:
    from extras.sonic_dds.sim_loop import SonicDdsSimLoop
    from extras.sonic_dds.wbc_config import SonicDdsConfig

    sim_cfg = G1SimulacrumConfig()
    sonic = SonicDdsConfig()
    sonic.loop.cameras = True
    sonic.loop.sensor_workers = True
    sonic.gantry.enabled = False
    loop = SonicDdsSimLoop(sim_cfg, sonic)
    loop.reset()
    assert loop._lidar_worker is not None
    assert loop._pose_buffer is not None
    loop._start_sensor_workers()
    try:
        loop._publish_pose_snapshot()
        deadline = time.monotonic() + 4.0
        while time.monotonic() < deadline:
            loop._publish_pose_snapshot()
            loop._poll_lidar_worker()
            if loop._lidar_worker.last_scan_seq() > 0:
                assert loop._last_cloud is not None
                return
            time.sleep(0.02)
        raise AssertionError("SonicDdsSimLoop lidar worker did not scan within 4s")
    finally:
        loop._stop_sensor_workers()
