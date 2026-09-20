#!/usr/bin/env python3
"""CLI for programmatic SONIC locomotion in simulacrum."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[2]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_locomotion.client import DEFAULT_BIND, LocomotionClient  # noqa: E402
from extras.sonic_locomotion.modes import LocomotionMode  # noqa: E402


def _mode(name: str) -> LocomotionMode:
    key = name.strip().lower().replace("-", "_")
    aliases = {
        "idle": LocomotionMode.IDLE,
        "slow": LocomotionMode.SLOW_WALK,
        "slow_walk": LocomotionMode.SLOW_WALK,
        "walk": LocomotionMode.WALK,
        "run": LocomotionMode.RUN,
    }
    if key not in aliases:
        raise argparse.ArgumentTypeError(f"unknown mode {name!r}")
    return aliases[key]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Programmatic locomotion for SONIC zmq_manager + g1-simulacrum",
    )
    parser.add_argument("--bind", default=DEFAULT_BIND, help="ZMQ PUB bind (default tcp://*:5556)")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--twist-timeout-s", type=float, default=0.5)

    sub = parser.add_subparsers(dest="cmd", required=True)

    walk = sub.add_parser("walk", help="Walk with body-frame velocity for N seconds")
    walk.add_argument("--vx", type=float, default=0.3, help="Forward m/s")
    walk.add_argument("--vy", type=float, default=0.0, help="Left m/s")
    walk.add_argument("--wz", type=float, default=0.0, help="Yaw rad/s")
    walk.add_argument("--seconds", type=float, default=5.0)
    walk.add_argument("--mode", type=_mode, default=None, help="Force gait (slow|walk|run)")

    idle = sub.add_parser("idle", help="Hold standing planner idle")
    idle.add_argument("--seconds", type=float, default=3.0)

    pulse = sub.add_parser("pulse", help="Square-wave forward walk")
    pulse.add_argument("--speed", type=float, default=0.4)
    pulse.add_argument("--on-s", type=float, default=2.0)
    pulse.add_argument("--off-s", type=float, default=1.0)
    pulse.add_argument("--cycles", type=int, default=2)

    args = parser.parse_args()
    client = LocomotionClient(
        bind=args.bind,
        rate_hz=args.rate_hz,
        twist_timeout_s=args.twist_timeout_s,
    )

    try:
        client.start()
        if args.cmd == "walk":
            print(
                f"[locomotion] vx={args.vx:.2f} vy={args.vy:.2f} wz={args.wz:.2f} "
                f"for {args.seconds:.1f}s"
            )
            client.set_velocity(args.vx, args.vy, args.wz, mode=args.mode)
            time.sleep(max(0.0, args.seconds))
            client.idle()
            time.sleep(0.5)
        elif args.cmd == "idle":
            print(f"[locomotion] idle for {args.seconds:.1f}s")
            client.idle()
            time.sleep(max(0.0, args.seconds))
        elif args.cmd == "pulse":
            for i in range(max(1, args.cycles)):
                print(f"[locomotion] pulse {i + 1}/{args.cycles}: on {args.on_s:.1f}s")
                client.walk_forward(args.speed)
                time.sleep(max(0.0, args.on_s))
                print(f"[locomotion] pulse {i + 1}/{args.cycles}: off {args.off_s:.1f}s")
                client.idle()
                time.sleep(max(0.0, args.off_s))
    finally:
        client.stop()
        client.close()
    print("[locomotion] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
