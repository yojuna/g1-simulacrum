"""Tests for plant platform exteroception wire v2."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from g1_simulacrum.sensors.data_types import CameraIntrinsics, DepthFrame

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_dds.platform_extero import (  # noqa: E402
    SCHEMA_VERSION,
    encode_extero_stream,
)


def test_encode_extero_stream_v2_schema() -> None:
    frame = DepthFrame(
        rgb=np.zeros((48, 64, 3), dtype=np.uint8),
        depth=np.ones((48, 64), dtype=np.float32),
        intrinsics=CameraIntrinsics(
            fx=100.0,
            fy=100.0,
            cx=32.0,
            cy=24.0,
            width=64,
            height=48,
        ),
        timestamp=1.5,
        ref_proprio_seq=11,
        T_world_camera=np.eye(4),
    )
    wire = encode_extero_stream(frame, seq=4, ref_proprio_seq=11)
    assert wire[:1] != b"{"
    import msgpack

    payload = msgpack.unpackb(wire, raw=False)
    assert payload["schema_version"] == SCHEMA_VERSION
    assert payload["seq"] == 4
    assert payload["ref_proprio_seq"] == 11
    assert payload["color_fmt"] in ("jpeg", "raw")
    assert payload["depth"]
    assert payload["T_world_camera"] == np.eye(4).tolist()
