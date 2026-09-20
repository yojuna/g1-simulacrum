"""Tests for plant platform proprioception builder and publisher."""

from __future__ import annotations

import json
import socket
import sys
import time
from pathlib import Path

import numpy as np
import pytest

from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig
from g1_simulacrum.model.joints import BODY_JOINT_NAMES, NUM_BODY_JOINTS

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.platform_proprio import (  # noqa: E402
    SCHEMA_VERSION,
    PlantProprioPublisher,
    build_proprioception_dict,
    wire_joint_names,
)


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


def test_build_proprioception_dict_schema_and_joints() -> None:
    sim = _minimal_sim()
    payload = build_proprioception_dict(sim.compiled, sim.data, seq=1, scene_id="kitchen")

    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["seq"] == 1
    assert payload["source"] == "plant_zmq"
    assert payload["scene_id"] == "kitchen"
    assert len(payload["joint_positions"]) == NUM_BODY_JOINTS
    assert len(payload["body_q"]) == NUM_BODY_JOINTS
    assert len(payload["left_hand_q"]) == 7
    assert len(payload["right_hand_q"]) == 7
    assert len(payload["qpos"]) == sim.model.nq
    assert len(payload["qvel"]) == sim.model.nv

    wire_names = wire_joint_names()
    assert len(wire_names) == len(BODY_JOINT_NAMES)
    for i, base_name in enumerate(BODY_JOINT_NAMES):
        assert wire_names[i] == f"{base_name}_joint"
        adr = sim.compiled.body_qposadr[base_name]
        assert payload["joint_positions"][wire_names[i]] == pytest.approx(sim.data.qpos[adr])


def test_build_proprioception_dict_uses_pelvis_body_pose() -> None:
    sim = _minimal_sim()
    payload = build_proprioception_dict(sim.compiled, sim.data, seq=2)
    import mujoco

    pelvis_bid = mujoco.mj_name2id(sim.model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
    assert pelvis_bid >= 0
    assert payload["base_pos_world"] == pytest.approx(sim.data.xpos[pelvis_bid].tolist())
    assert payload["base_quat_wxyz"] == pytest.approx(sim.data.xquat[pelvis_bid].tolist())


def _free_tcp_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def test_plant_proprio_publisher_roundtrip() -> None:
    zmq = pytest.importorskip("zmq")
    port = _free_tcp_port()
    bind_addr = f"tcp://127.0.0.1:{port}"
    pub = PlantProprioPublisher(bind_addr)
    pub.start()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.LINGER, 0)
    sub.setsockopt(zmq.CONFLATE, 1)
    sub.setsockopt(zmq.RCVTIMEO, 500)
    sub.setsockopt(zmq.SUBSCRIBE, b"proprio")
    sub.connect(bind_addr)

    sim = _minimal_sim()
    payload = build_proprioception_dict(sim.compiled, sim.data, seq=42)
    for _ in range(20):
        pub.publish_dict(payload)
        try:
            raw = sub.recv()
        except zmq.Again:
            time.sleep(0.01)
            continue
        assert raw.startswith(b"proprio")
        got = json.loads(raw[len(b"proprio") :].decode("utf-8"))
        assert got["seq"] == 42
        break
    else:
        raise AssertionError("subscriber did not receive plant proprio frame")

    pub.close()
    sub.close()
