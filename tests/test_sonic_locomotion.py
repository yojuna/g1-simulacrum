"""Unit tests for sonic_locomotion (no live ZMQ / deploy required)."""

from __future__ import annotations

import json
import math

from extras.sonic_locomotion.client import parse_cmd_vel_json
from extras.sonic_locomotion.modes import LocomotionMode, clamp_speed, mode_for_speed
from extras.sonic_locomotion.packed_zmq import build_command_message, build_planner_message
from extras.sonic_locomotion.twist import TwistIntegrator


def test_build_command_message_prefix() -> None:
    raw = build_command_message(start=True, stop=False, planner=True)
    assert raw.startswith(b"command")
    assert len(raw) > 1280


def test_build_planner_message_no_vr() -> None:
    raw = build_planner_message(
        mode=2,
        movement=(1.0, 0.0, 0.0),
        facing=(1.0, 0.0, 0.0),
        speed=0.5,
    )
    assert raw.startswith(b"planner")
    assert b"vr_position" not in raw[:1400]


def test_mode_for_speed() -> None:
    assert mode_for_speed(0.0) == LocomotionMode.IDLE
    assert mode_for_speed(0.3) == LocomotionMode.SLOW_WALK
    assert mode_for_speed(1.0) == LocomotionMode.WALK
    assert mode_for_speed(3.0) == LocomotionMode.RUN


def test_clamp_speed_walk() -> None:
    assert clamp_speed(LocomotionMode.WALK, 5.0) == 2.5
    assert clamp_speed(LocomotionMode.WALK, 1.0) == 1.0


def test_twist_integrator_forward() -> None:
    integrator = TwistIntegrator()
    cmd = integrator.update(0.5, 0.0, 0.0, dt=0.02)
    assert cmd.mode == int(LocomotionMode.SLOW_WALK)
    assert math.isclose(cmd.movement[0], 1.0, abs_tol=1e-6)
    assert math.isclose(cmd.movement[1], 0.0, abs_tol=1e-6)
    assert cmd.speed > 0.0


def test_twist_integrator_turn() -> None:
    integrator = TwistIntegrator()
    integrator.update(0.0, 0.0, math.pi / 2, dt=1.0)
    cmd = integrator.idle()
    assert math.isclose(cmd.facing[0], 0.0, abs_tol=1e-5)
    assert math.isclose(cmd.facing[1], 1.0, abs_tol=1e-5)


def test_parse_cmd_vel_json() -> None:
    payload = json.dumps(
        {"linear": {"x": 0.3, "y": 0.1, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0.05}}
    ).encode()
    vx, vy, wz = parse_cmd_vel_json(payload)
    assert vx == 0.3
    assert vy == 0.1
    assert wz == 0.05
