"""Plant exteroception wire format and ZMQ publisher (platform bus :5561).

Wire v2 (default): msgpack + JPEG RGB + raw float32 depth — matches
``g1_manip.comms.serialization.encode_extero_stream`` (cross-repo conformance
tested in ws_manipulation). Legacy v1 JSON/base64 decode remains on the manip
subscriber for one release.
"""

from __future__ import annotations

import logging
import os
from typing import Any

import mujoco
import numpy as np

from g1_simulacrum.sensors.data_types import DepthFrame

logger = logging.getLogger(__name__)

SCHEMA_VERSION = 2
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


def _jpeg_quality() -> int:
    raw = os.environ.get("PLANT_EXTERO_JPEG_QUALITY", "85").strip()
    try:
        return max(1, min(100, int(raw)))
    except ValueError:
        return 85


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


def _jpeg_encode_color(color: np.ndarray, quality: int) -> tuple[bytes, str]:
    color_arr = np.asarray(color, dtype=np.uint8)
    try:
        import cv2

        bgr = cv2.cvtColor(color_arr, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
        if ok:
            return buf.tobytes(), "jpeg"
    except ImportError:
        pass
    return color_arr.tobytes(), "raw"


def encode_extero_stream(
    frame: DepthFrame,
    *,
    seq: int,
    ref_proprio_seq: int,
    T_world_camera: np.ndarray | None = None,
    jpeg_quality: int | None = None,
) -> bytes:
    """Build platform extero v2 msgpack payload (no JSON/base64)."""
    try:
        import msgpack
    except ImportError as exc:
        raise ImportError(
            "msgpack is required for plant extero v2: pip install 'g1-simulacrum[sonic]'",
        ) from exc

    quality = _jpeg_quality() if jpeg_quality is None else int(jpeg_quality)
    color = np.asarray(frame.rgb, dtype=np.uint8)
    depth = np.asarray(frame.depth, dtype=np.float32)
    color_bytes, color_fmt = _jpeg_encode_color(color, quality)
    intr = frame.intrinsics
    payload: dict[str, Any] = {
        "schema_version": SCHEMA_VERSION,
        "seq": int(seq),
        "ref_proprio_seq": int(ref_proprio_seq),
        "stamp_motion_s": float(frame.timestamp),
        "source": "plant_zmq",
        "color": color_bytes,
        "color_shape": list(color.shape),
        "color_fmt": color_fmt,
        "depth": depth.tobytes(),
        "depth_shape": list(depth.shape),
        "depth_dtype": str(depth.dtype),
        "frame_id": int(seq),
        "timestamp": float(frame.timestamp),
        "intrinsics": {
            "fx": float(intr.fx),
            "fy": float(intr.fy),
            "cx": float(intr.cx),
            "cy": float(intr.cy),
            "width": int(intr.width),
            "height": int(intr.height),
        },
    }
    T = T_world_camera if T_world_camera is not None else frame.T_world_camera
    if T is not None:
        payload["T_world_camera"] = np.asarray(T, dtype=np.float64).reshape(4, 4).tolist()
    return msgpack.packb(payload, use_bin_type=True)


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
        self._socket.setsockopt(zmq.SNDHWM, 1)
        self._socket.bind(self._bind_addr)
        self._bound = True
        logger.info("PlantExteroPublisher bound %s (wire v%d)", self._bind_addr, SCHEMA_VERSION)

    def close(self) -> None:
        if self._socket is not None:
            try:
                self._socket.close(linger=0)
            except Exception:  # pragma: no cover
                logger.debug("PlantExteroPublisher close raised", exc_info=True)
            self._socket = None
        self._bound = False

    def publish_frame(
        self,
        frame: DepthFrame,
        *,
        seq: int,
        ref_proprio_seq: int,
        T_world_camera: np.ndarray | None = None,
    ) -> None:
        """Encode and send one RGB-D frame (call from depth worker thread only)."""
        import zmq

        if self._socket is None:
            self.start()
        assert self._socket is not None
        wire = encode_extero_stream(
            frame,
            seq=int(seq),
            ref_proprio_seq=int(ref_proprio_seq),
            T_world_camera=T_world_camera,
        )
        try:
            self._socket.send(_TOPIC_EXTERO + wire, zmq.NOBLOCK)
        except zmq.Again:
            logger.debug("PlantExteroPublisher: send backlog, dropping frame")
        except zmq.ZMQError as exc:
            logger.warning("PlantExteroPublisher send failed: %s", exc)
