"""Packed ZMQ builders for GEAR ``zmq_manager`` (command / planner topics).

Wire layout::

    [topic_ascii][1280-byte NUL-padded JSON header][little-endian payload]

Copied from ``gear_sonic/utils/teleop/zmq/zmq_planner_sender.py`` so g1-simulacrum
does not depend on the full GR00T tree at runtime.
"""

from __future__ import annotations

import json
import struct
from typing import Sequence

HEADER_SIZE = 1280


def _build_header(fields: list, version: int = 1, count: int = 1) -> bytes:
    header = {
        "v": version,
        "endian": "le",
        "count": count,
        "fields": fields,
    }
    header_json = json.dumps(header, separators=(",", ":")).encode("utf-8")
    if len(header_json) > HEADER_SIZE:
        raise ValueError(f"Header too large: {len(header_json)} > {HEADER_SIZE}")
    return header_json.ljust(HEADER_SIZE, b"\x00")


def build_command_message(
    start: bool,
    stop: bool,
    planner: bool,
    delta_heading: float | None = None,
) -> bytes:
    fields = [
        {"name": "start", "dtype": "u8", "shape": [1]},
        {"name": "stop", "dtype": "u8", "shape": [1]},
        {"name": "planner", "dtype": "u8", "shape": [1]},
    ]
    payload = b"".join(
        (
            struct.pack("B", 1 if start else 0),
            struct.pack("B", 1 if stop else 0),
            struct.pack("B", 1 if planner else 0),
        )
    )
    if delta_heading is not None:
        fields.append({"name": "delta_heading", "dtype": "f32", "shape": [1]})
        payload += struct.pack("<f", float(delta_heading))
    return b"command" + _build_header(fields) + payload


def _append_f32(fields: list, payload: bytes, name: str, values: Sequence[float]) -> bytes:
    fields.append({"name": name, "dtype": "f32", "shape": [len(values)]})
    for value in values:
        payload += struct.pack("<f", float(value))
    return payload


def build_planner_message(
    mode: int,
    movement: Sequence[float],
    facing: Sequence[float],
    speed: float = -1.0,
    height: float = -1.0,
    vr_3pt_position: Sequence[float] | None = None,
    vr_3pt_orientation: Sequence[float] | None = None,
    vr_3pt_compliance: Sequence[float] | None = None,
) -> bytes:
    if len(movement) != 3:
        raise ValueError("movement must have length 3")
    if len(facing) != 3:
        raise ValueError("facing must have length 3")
    if vr_3pt_position is not None and len(vr_3pt_position) != 9:
        raise ValueError("vr_3pt_position must have length 9")
    if vr_3pt_orientation is not None and len(vr_3pt_orientation) != 12:
        raise ValueError("vr_3pt_orientation must have length 12 (wxyz × 3)")

    fields = [
        {"name": "mode", "dtype": "i32", "shape": [1]},
        {"name": "movement", "dtype": "f32", "shape": [3]},
        {"name": "facing", "dtype": "f32", "shape": [3]},
        {"name": "speed", "dtype": "f32", "shape": [1]},
        {"name": "height", "dtype": "f32", "shape": [1]},
    ]
    payload = b"".join(
        (
            struct.pack("<i", int(mode)),
            struct.pack("<fff", float(movement[0]), float(movement[1]), float(movement[2])),
            struct.pack("<fff", float(facing[0]), float(facing[1]), float(facing[2])),
            struct.pack("<f", float(speed)),
            struct.pack("<f", float(height)),
        )
    )
    if vr_3pt_position is not None:
        payload = _append_f32(fields, payload, "vr_position", vr_3pt_position)
    if vr_3pt_orientation is not None:
        payload = _append_f32(fields, payload, "vr_orientation", vr_3pt_orientation)
    if vr_3pt_compliance is not None:
        payload = _append_f32(fields, payload, "vr_compliance", vr_3pt_compliance)
    return b"planner" + _build_header(fields) + payload
