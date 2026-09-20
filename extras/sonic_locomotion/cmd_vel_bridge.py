"""Bridge Nav2 ``cmd_vel_zmq`` (:5558 JSON) into SONIC locomotion (:5556 packed).

Topology (matches ``behavior_controller`` twist SUB design)::

    cmd_vel_zmq (PUB connect)  →  this bridge (SUB bind :5558)
                                        ↓
                                 LocomotionClient (PUB bind :5556)
                                        ↓
                                 gear_sonic_deploy zmq_manager
"""

from __future__ import annotations

import argparse
import json
import threading
import time

import zmq

from .client import DEFAULT_BIND, LocomotionClient, parse_cmd_vel_json


def run_cmd_vel_bridge(
    *,
    cmd_vel_bind: str = "tcp://*:5558",
    sonic_bind: str = DEFAULT_BIND,
    rate_hz: float = 50.0,
    twist_timeout_s: float = 0.5,
) -> None:
    client = LocomotionClient(
        bind=sonic_bind,
        rate_hz=rate_hz,
        twist_timeout_s=twist_timeout_s,
    )
    client.start()

    ctx = zmq.Context.instance()
    sub = ctx.socket(zmq.SUB)
    sub.setsockopt(zmq.SUBSCRIBE, b"cmd_vel")
    sub.setsockopt(zmq.RCVTIMEO, 200)
    sub.bind(cmd_vel_bind)

    stop = threading.Event()

    def _shutdown(*_: object) -> None:
        stop.set()

    import signal

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    print(f"[cmd_vel_bridge] SUB {cmd_vel_bind} → locomotion PUB {sonic_bind}")
    try:
        while not stop.is_set():
            try:
                parts = sub.recv_multipart()
            except zmq.Again:
                continue
            if len(parts) < 2:
                continue
            topic, payload = parts[0], parts[1]
            if topic != b"cmd_vel":
                continue
            try:
                vx, vy, wz = parse_cmd_vel_json(payload)
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                print(f"[cmd_vel_bridge] parse error: {exc}")
                continue
            client.set_velocity(vx, vy, wz)
    finally:
        client.stop()
        client.close()
        sub.close(0)
        print("[cmd_vel_bridge] stopped")


def main() -> int:
    parser = argparse.ArgumentParser(description="cmd_vel ZMQ → SONIC locomotion bridge")
    parser.add_argument("--cmd-vel-bind", default="tcp://*:5558")
    parser.add_argument("--sonic-bind", default=DEFAULT_BIND)
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--twist-timeout-s", type=float, default=0.5)
    args = parser.parse_args()
    run_cmd_vel_bridge(
        cmd_vel_bind=args.cmd_vel_bind,
        sonic_bind=args.sonic_bind,
        rate_hz=args.rate_hz,
        twist_timeout_s=args.twist_timeout_s,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
