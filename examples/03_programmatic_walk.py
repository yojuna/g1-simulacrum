"""Programmatic walking via SONIC zmq_manager (no keyboard / Nav2 required).

Three terminals (all host network):

Terminal 1 — MuJoCo plant + DDS::

    cd docker
    ./run.sh sonic python examples/02_sonic_dds_bridge.py

Terminal 2 — SONIC deploy with zmq_manager::

    cd GR00T-WholeBodyControl/gear_sonic_deploy
    ./docker/run-ros2-dev.sh
    bash deploy.sh sim --input-type zmq_manager --zmq-host 127.0.0.1

    Press ``]`` to init, ``9`` to lower gantry (or wait for harness settle).

Terminal 3 — locomotion client (this script)::

    cd docker
    ./run.sh sonic python examples/03_programmatic_walk.py --vx 0.35 --seconds 8
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))

from extras.sonic_locomotion import LocomotionClient  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Walk the simulacrum via SONIC planner ZMQ")
    parser.add_argument("--vx", type=float, default=0.35, help="Forward speed m/s")
    parser.add_argument("--vy", type=float, default=0.0, help="Left speed m/s")
    parser.add_argument("--wz", type=float, default=0.0, help="Yaw rate rad/s")
    parser.add_argument("--seconds", type=float, default=8.0, help="Walk duration")
    parser.add_argument("--settle-s", type=float, default=1.0, help="Idle hold before stop")
    args = parser.parse_args()

    client = LocomotionClient()
    try:
        print("[walk] starting locomotion heartbeat (command.start + planner @ 50 Hz)")
        client.start()
        time.sleep(0.5)
        print(f"[walk] set_velocity vx={args.vx:.2f} vy={args.vy:.2f} wz={args.wz:.2f}")
        client.set_velocity(args.vx, args.vy, args.wz)
        time.sleep(max(0.0, args.seconds))
        print("[walk] idle")
        client.idle()
        time.sleep(max(0.0, args.settle_s))
    finally:
        client.stop()
        client.close()
    print("[walk] done")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
