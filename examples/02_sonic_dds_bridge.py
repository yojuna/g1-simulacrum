"""SONIC DDS sim2sim: MuJoCo viewer + Unitree DDS in one process (GEAR run_sim_loop).

Terminal 1 (this sim, Docker, host network)::

    cd docker
    ./run.sh sonic python examples/02_sonic_dds_bridge.py

Terminal 2 (GEAR deploy, also Docker with host network)::

    cd GR00T-WholeBodyControl/gear_sonic_deploy
    ./docker/run-ros2-dev.sh
    bash deploy.sh sim

Deploy controls: ``]`` init, ``9`` gantry, ``T`` planner, ``O`` operator.
Viewer (this window): ``7``/``8`` hold height, ``9`` toggle crane (GEAR); numpad trolley.
Sensors (Mid-360 + D435i worker threads) are on by default (not in ``step_physics``).
``--no-sensors`` disables them. ``--overlay sparse|dense|full`` enables GLFW debug overlays.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig

_PKG = Path(__file__).resolve().parents[1]
if str(_PKG) not in sys.path:
    sys.path.insert(0, str(_PKG))


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="SONIC DDS bridge for g1-simulacrum")
    p.add_argument(
        "--config",
        default="configs/default.yaml",
        help="G1Simulacrum YAML (controller passthrough is forced by the bridge)",
    )
    p.add_argument(
        "--sonic-config",
        default="configs/sonic_dds.yaml",
        help="SONIC DDS bridge YAML",
    )
    p.add_argument(
        "--scene",
        type=Path,
        default=None,
        help="Optional MJCF scene (must include g1_robot.xml)",
    )
    p.add_argument("--no-gantry", action="store_true", help="Disable overhead crane")
    p.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Run for N seconds then exit (default: run until Ctrl+C / viewer close)",
    )
    p.add_argument(
        "--headless",
        action="store_true",
        help="No MuJoCo GLFW window (DDS still runs)",
    )
    p.add_argument(
        "--headless-smoke",
        action="store_true",
        help="Run 2 s without DDS (physics smoke)",
    )
    p.add_argument(
        "--gear-parity",
        action="store_true",
        help="200 Hz control (5× 1 ms substeps) to match GEAR SIMULATE_DT wall rate; "
        "does not widen MuJoCo timestep",
    )
    p.add_argument(
        "--no-sensors",
        action="store_true",
        help="Disable Mid-360 + D435i worker threads (on by default)",
    )
    p.add_argument(
        "--overlay",
        choices=("sparse", "dense", "full"),
        default=None,
        help="Enable GLFW lidar/depth debug overlays (off by default). sparse|dense|full",
    )
    return p.parse_args()


def main() -> None:
    args = _parse_args()
    sim_cfg = G1SimulacrumConfig.from_yaml(args.config)

    if args.headless_smoke:
        sim_cfg.sensors.mid360.enabled = False
        sim_cfg.sensors.d435i.enabled = False
        for imu in (
            sim_cfg.sensors.imu.pelvis,
            sim_cfg.sensors.imu.torso,
            sim_cfg.sensors.imu.mid360,
            sim_cfg.sensors.imu.d435i,
        ):
            imu.enabled = False

        sim = G1Simulacrum(config=sim_cfg)
        sim.build(scene_xml=args.scene)
        obs = sim.reset()
        q_hold = obs.joint_state.position.copy()
        dt = 1.0 / sim.config.controller.control_hz
        t_end = time.monotonic() + 2.0
        while time.monotonic() < t_end:
            sim.step(q_hold)
            time.sleep(dt)
        print("headless smoke finished")
        return

    from extras.sonic_dds import SonicDdsConfig, SonicDdsSimLoop  # noqa: E402
    from extras.sonic_dds.overlay import overlay_from_preset  # noqa: E402

    sonic_cfg = SonicDdsConfig.from_yaml(args.sonic_config)

    if args.no_gantry:
        sonic_cfg.gantry.enabled = False
    if args.gear_parity:
        sonic_cfg.loop.gear_parity = True
    if args.no_sensors:
        sonic_cfg.loop.cameras = False
    if args.headless:
        sonic_cfg.loop.onscreen = False

    print("=== g1-simulacrum SONIC DDS (sim + viewer + DDS, one process) ===", flush=True)
    overlay = overlay_from_preset(args.overlay) if args.overlay else None
    loop = SonicDdsSimLoop(
        sim_cfg,
        sonic_cfg,
        scene_xml=args.scene,
        overlay=overlay,
    )
    loop.reset()
    print(
        "\nStart deploy in another terminal (host network):\n"
        "  cd GR00T-WholeBodyControl/gear_sonic_deploy\n"
        "  ./docker/run-ros2-dev.sh    # then: bash deploy.sh sim\n"
        "Controls (deploy): ] init, 9 gantry, T planner, O operator\n"
        "Viewer: 7/8 cable, 9 toggle crane\n",
        flush=True,
    )
    loop.run(duration_s=args.duration)


if __name__ == "__main__":
    main()
