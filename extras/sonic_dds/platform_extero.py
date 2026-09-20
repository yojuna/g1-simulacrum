"""Plant exteroception wire format and ZMQ publisher (platform bus :5561)."""

from __future__ import annotations

import base64
import json
import logging
import os
import zlib
from typing import Any

import mujoco
import numpy as np

from g1_simulacrum.sensors.data_types import DepthFrame

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 1
_TOPIC_EXTERO = b"extero"

_R_OPTICAL_TO_MJ = np.array(
    [
        [1.0, 0.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, -1.0],
    ],
    dtype=np.float64,
)


def default_bind_addr() -> str:
    return os.environ.get("PLANT_EXTERO_BIND", "tcp://*:5561")


def extero_enabled() -> bool:
    raw = os.environ.get("PLANT_EXTERO_ENABLED", "1").strip().lower()
    return raw not in ("0", "false", "no", "off")


def camera_T_world_optical(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    camera_name: str,
) -> np.ndarray:
    """Camera optical frame → world (matches manip ``SimCameraExtrinsics``)."""
    cam_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, camera_name)
    if cam_id < 0:
        raise ValueError(f"camera not found: {camera_name!r}")
    cam_pos = np.asarray(data.cam_xpos[cam_id], dtype=np.float64)
    cam_rot = np.asarray(data.cam_xmat[cam_id], dtype=np.float64).reshape(3, 3)
    R_world_optical = cam_rot @ _R_OPTICAL_TO_MJ
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R_world_optical
    T[:3, 3] = cam_pos
    return T


def _encode_rgb(rgb: np.ndarray) -> str:
    return base64.b64encode(np.asarray(rgb, dtype=np.uint8).tobytes()).decode("ascii")


def _encode_depth(depth: np.ndarray) -> str:
    raw = np.asarray(depth, dtype=np.float32).tobytes()
    return base64.b64encode(zlib.compress(raw)).decode("ascii")


def build_exteroception_dict(
    frame: DepthFrame,
    *,
    seq: int,
    ref_proprio_seq: int,
    T_world_camera: np.ndarray | None = None,
) -> dict[str, Any]:
    intr = frame.intrinsics
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "seq": int(seq),
        "ref_proprio_seq": int(ref_proprio_seq),
        "stamp_motion_s": float(frame.timestamp),
        "intrinsics": {
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.cx),
            "cy": float(intr.cy),
            "width": int(intr.width),
            "height": int(intr.height),
        },
        "source": "plant_zmq",
        "color_b64": _encode_rgb(frame.rgb),
        "depth_b64": _encode_depth(frame.depth),
    }
    T = T_world_camera if T_world_camera is not None else frame.T_world_camera
    if T is not None:
        payload["T_world_camera"] = np.asarray(T, dtype=np.float64).reshape(4, 4).tolist()
    return payload


class PlantExteroPublisher:
    """Keep-latest ZMQ PUB on topic ``extero`` (``CONFLATE=1``)."""

    def __init__(self, bind_addr: str) -> None:
        self._bind_addr = bind_addr
        self._socket: Any = None
        self._bound = False

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
        logger.info("PlantExteroPublisher bound %s", self._bind_addr)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:  # pragma: no cover
                logger.debug("PlantExteroPublisher close raised", exc_info=True)
            self._socket = None
        self._bound = False

    def publish_dict(self, payload: dict[str, Any]) -> None:
        import zmq

        if self._socket is None:
            self.start()
        assert self._socket is not None
        raw = _TOPIC_EXTERO + json.dumps(payload).encode("utf-8")
        try:
            self._socket.send(raw, zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("PlantExteroPublisher: send backlog, dropping frame")
        except zmq.ZMQError as exc:
            logger.warning("PlantExteroPublisher send failed: %s", exc)
