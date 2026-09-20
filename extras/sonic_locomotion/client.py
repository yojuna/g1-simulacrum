"""Programmatic locomotion client for SONIC ``zmq_manager`` deploy."""

from __future__ import annotations

import json
import threading
import time
from dataclasses import dataclass
from typing import Any

import zmq

from .modes import LocomotionMode
from .packed_zmq import build_command_message, build_planner_message
from .twist import PlannerCommand, TwistIntegrator

DEFAULT_BIND = "tcp://*:5556"
DEFAULT_RATE_HZ = 50.0
COMMAND_PERIOD_S = 0.5
TWIST_TIMEOUT_S = 0.5


@dataclass
class LocomotionState:
    started: bool = False
    linear_x: float = 0.0
    linear_y: float = 0.0
    angular_z: float = 0.0
    mode_override: LocomotionMode | None = None


class LocomotionClient:
    """Publish ``command`` + ``planner`` heartbeats to SONIC deploy ``zmq_manager``.

    The deploy binary SUBscribes to ``--zmq-host`` / ``--zmq-port`` (default
    ``127.0.0.1:5556``). This client **binds** on that port so deploy can connect.

    Locomotion-only commands omit ``vr_position`` so the policy stays in g1 /
    joint encode mode (mode 0).
    """

    def __init__(
        self,
        *,
        bind: str = DEFAULT_BIND,
        rate_hz: float = DEFAULT_RATE_HZ,
        twist_timeout_s: float = TWIST_TIMEOUT_S,
    ) -> None:
        self._bind = bind
        self._dt_s = 1.0 / max(rate_hz, 1.0)
        self._twist_timeout_s = max(0.1, twist_timeout_s)
        self._ctx = zmq.Context.instance()
        self._sock = self._ctx.socket(zmq.PUB)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(bind)
        time.sleep(0.05)

        self._integrator = TwistIntegrator()
        self._state = LocomotionState()
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._last_twist_mono = 0.0

    def close(self) -> None:
        self.stop()
        self._sock.close(0)

    def start(self) -> None:
        """Send ``command.start`` and begin the planner heartbeat thread."""
        with self._lock:
            self._state.started = True
            self._last_twist_mono = time.monotonic()
        self._send_command(start=True, stop=False, planner=True)
        self._start_thread()

    def stop(self) -> None:
        """Stop heartbeat and send ``command.stop``."""
        self._stop.set()
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        self._thread = None
        self._stop.clear()
        with self._lock:
            if self._state.started:
                self._send_command(start=False, stop=True, planner=True)
            self._state.started = False
            self._state.linear_x = 0.0
            self._state.linear_y = 0.0
            self._state.angular_z = 0.0

    def idle(self) -> None:
        with self._lock:
            self._state.linear_x = 0.0
            self._state.linear_y = 0.0
            self._state.angular_z = 0.0
            self._last_twist_mono = time.monotonic()

    def set_velocity(
        self,
        linear_x: float,
        linear_y: float,
        angular_z: float = 0.0,
        *,
        mode: LocomotionMode | None = None,
    ) -> None:
        """Set a holonomic body-frame velocity (m/s, m/s, rad/s)."""
        with self._lock:
            if not self._state.started:
                self._state.started = True
                self._send_command(start=True, stop=False, planner=True)
                self._start_thread()
            self._state.linear_x = float(linear_x)
            self._state.linear_y = float(linear_y)
            self._state.angular_z = float(angular_z)
            self._state.mode_override = mode
            self._last_twist_mono = time.monotonic()

    def walk_forward(self, speed: float, angular_z: float = 0.0) -> None:
        self.set_velocity(speed, 0.0, angular_z)

    def strafe(self, speed: float) -> None:
        self.set_velocity(0.0, speed, 0.0)

    def turn_in_place(self, angular_z: float) -> None:
        self.set_velocity(0.0, 0.0, angular_z)

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            cmd = self._planner_from_state()
        return {
            "started": self._state.started,
            "mode": cmd.mode,
            "movement": list(cmd.movement),
            "facing": list(cmd.facing),
            "speed": cmd.speed,
            "linear_x": self._state.linear_x,
            "linear_y": self._state.linear_y,
            "angular_z": self._state.angular_z,
        }

    def _start_thread(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self._thread.start()

    def _heartbeat_loop(self) -> None:
        t_cmd = time.monotonic()
        while not self._stop.is_set():
            t0 = time.monotonic()
            with self._lock:
                if not self._state.started:
                    break
                if (
                    time.monotonic() - self._last_twist_mono > self._twist_timeout_s
                    and (
                        abs(self._state.linear_x) > 1e-6
                        or abs(self._state.linear_y) > 1e-6
                        or abs(self._state.angular_z) > 1e-6
                    )
                ):
                    self._state.linear_x = 0.0
                    self._state.linear_y = 0.0
                    self._state.angular_z = 0.0
                cmd = self._planner_from_state()
            self._send_planner(cmd)
            if time.monotonic() - t_cmd >= COMMAND_PERIOD_S:
                self._send_command(start=True, stop=False, planner=True)
                t_cmd = time.monotonic()
            elapsed = time.monotonic() - t0
            if elapsed < self._dt_s:
                time.sleep(self._dt_s - elapsed)

    def _planner_from_state(self) -> PlannerCommand:
        if (
            abs(self._state.linear_x) < 1e-6
            and abs(self._state.linear_y) < 1e-6
            and abs(self._state.angular_z) < 1e-6
        ):
            return self._integrator.idle()
        return self._integrator.update(
            self._state.linear_x,
            self._state.linear_y,
            self._state.angular_z,
            dt=self._dt_s,
            mode_override=self._state.mode_override,
        )

    def _send_command(self, *, start: bool, stop: bool, planner: bool) -> None:
        self._sock.send(build_command_message(start=start, stop=stop, planner=planner))

    def _send_planner(self, cmd: PlannerCommand) -> None:
        self._sock.send(
            build_planner_message(
                mode=cmd.mode,
                movement=cmd.movement,
                facing=cmd.facing,
                speed=cmd.speed,
                height=cmd.height,
            )
        )


def parse_cmd_vel_json(payload: bytes) -> tuple[float, float, float]:
    """Parse ROS ``cmd_vel_zmq`` JSON payload into (vx, vy, wz)."""
    data = json.loads(payload.decode("utf-8"))
    linear = data.get("linear", {})
    angular = data.get("angular", {})
    return (
        float(linear.get("x", 0.0)),
        float(linear.get("y", 0.0)),
        float(angular.get("z", 0.0)),
    )
