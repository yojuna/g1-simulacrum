"""Post-step platform proprioception wire format and ZMQ publisher.

Matches ``g1_manip.platform.samples.ProprioceptionSample`` JSON schema without
importing ws_manipulation (cross-repo conformance tested in manip).
"""

from __future__ import annotations

import json
import logging
import os
import time
from typing import Any

import mujoco
import numpy as np

from g1_simulacrum.model.joints import BODY_JOINT_NAMES
from g1_simulacrum.model.loader import CompiledModel

from .observation_adapter import prepare_obs_dict

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 1
_TOPIC_PROPrio = b"proprio"
_WIRE_JOINT_NAMES: tuple[str, ...] = tuple(f"{name}_joint" for name in BODY_JOINT_NAMES)


def wire_joint_names() -> tuple[str, ...]:
    """Joint keys on the platform bus (``RobotDescription`` / manip order)."""
    return _WIRE_JOINT_NAMES


def default_bind_addr() -> str:
    return os.environ.get("PLANT_PROPRIO_BIND", "tcp://*:5560")


def proprio_enabled() -> bool:
    raw = os.environ.get("PLANT_PROPRIO_ENABLED", "1").strip().lower()
    return raw not in ("0", "false", "no", "off")


def _pelvis_pose(model: mujoco.MjModel, data: mujoco.MjData) -> tuple[np.ndarray, np.ndarray]:
    pos = np.asarray(data.qpos[:3], dtype=np.float64)
    quat = np.asarray(data.qpos[3:7], dtype=np.float64)
    try:
        pelvis_bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        if pelvis_bid >= 0:
            pos = np.asarray(data.xpos[pelvis_bid], dtype=np.float64)
            quat = np.asarray(data.xquat[pelvis_bid], dtype=np.float64)
    except Exception:
        pass
    return pos.reshape(3), quat.reshape(4)


def build_proprioception_dict(
    compiled: CompiledModel,
    data: mujoco.MjData,
    *,
    seq: int,
    scene_id: str = "",
) -> dict[str, Any]:
    """Build measured-state payload from post-step ``MjData``."""
    obs = prepare_obs_dict(compiled, data)
    base_pos, base_quat = _pelvis_pose(compiled.model, data)
    body_q = obs["body_q"]
    joint_positions = {
        wire_name: float(body_q[i]) for i, wire_name in enumerate(_WIRE_JOINT_NAMES)
    }
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "seq": int(seq),
        "stamp_motion_s": float(data.time),
        "stamp_wall_s": float(time.monotonic()),
        "base_pos_world": [float(v) for v in base_pos],
        "base_quat_wxyz": [float(v) for v in base_quat],
        "joint_positions": joint_positions,
        "body_q": [float(v) for v in body_q],
        "left_hand_q": [float(v) for v in obs["left_hand_q"]],
        "right_hand_q": [float(v) for v in obs["right_hand_q"]],
        "qpos": [float(v) for v in np.asarray(data.qpos, dtype=np.float64)],
        "qvel": [float(v) for v in np.asarray(data.qvel, dtype=np.float64)],
        "scene_id": str(scene_id),
        "source": "plant_zmq",
    }
    return payload


class PlantProprioPublisher:
    """Keep-latest ZMQ PUB on topic ``proprio`` (``CONFLATE=1``)."""

    def __init__(self, bind_addr: str) -> None:
        self._bind_addr = bind_addr
        self._socket: Any = None
        self._bound = False

    @property
    def bind_addr(self) -> str:
        return self._bind_addr

    def start(self) -> None:
        import zmq

        if self._bound:
            return
        ctx = zmq.Context.instance()
        self._socket = ctx.socket(zmq.PUB)
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.setsockopt(zmq.CONFLATE, 1)
        self._socket.bind(self._bind_addr)
        self._bound = True
        logger.info("PlantProprioPublisher bound %s", self._bind_addr)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:  # pragma: no cover
                logger.debug("PlantProprioPublisher close raised", exc_info=True)
            self._socket = None
        self._bound = False

    def publish_dict(self, payload: dict[str, Any]) -> None:
        import zmq

        if self._socket is None:
            self.start()
        assert self._socket is not None
        raw = _TOPIC_PROPrio + json.dumps(payload).encode("utf-8")
        try:
            self._socket.send(raw, zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("PlantProprioPublisher: send backlog, dropping frame")
        except zmq.ZMQError as exc:
            logger.warning("PlantProprioPublisher send failed: %s", exc)
