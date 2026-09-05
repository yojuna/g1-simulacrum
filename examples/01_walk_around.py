"""Example 1: Walk the G1 around using GEAR-SONIC.

This example starts the MuJoCo simulation with the sensorized G1 and
connects to the GEAR-SONIC deployment stack via DDS. You control the
robot with keyboard input from the SONIC terminal.

Usage:
    # Terminal 1: Start simulation
    python examples/01_walk_around.py

    # Terminal 2: Start SONIC policy
    cd /path/to/GR00T-WholeBodyControl/gear_sonic_deploy
    ./deploy.sh \
        --cp policy/sonic_v1_1/model \
        --obs-config policy/sonic_v1_1/observation_config.yaml \
        sim

    # In SONIC terminal:
    #   Press ] to start policy
    #   Click MuJoCo viewer, press 9 to drop robot
    #   Press T to play reference motion
    #   Press N/P to cycle motions
"""

import mujoco
import mujoco.viewer

from g1_simulacrum import G1SimulacrumConfig, G1Simulacrum


def main():
    # Load config tuned for SONIC v1.1
    config = G1SimulacrumConfig.from_yaml("configs/sonic_v1_1.yaml")

    # Build the sensorized G1
    sim = G1Simulacrum(config=config)
    sim.build_model()

    print("=" * 60)
    print("G1 Simulation Ready")
    print(f"  Controller: {config.controller.type}")
    print(f"  Sensors: Mid-360={config.sensors.mid360.enabled}, "
          f"D435i={config.sensors.d435i.enabled}")
    print(f"  Physics: {config.controller.physics_hz} Hz")
    print(f"  Control: {config.controller.control_hz} Hz")
    print("=" * 60)
    print("\nWaiting for SONIC deploy connection...")
    print("Start gear_sonic_deploy in another terminal.\n")

    # Run with MuJoCo interactive viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        while viewer.is_running():
            # The SONIC bridge handles control via DDS callbacks.
            # We just need to step the physics and collect sensors.
            obs = sim.step()

            # Optional: print sensor stats periodically
            if obs.sensors.lidar is not None:
                pc = obs.sensors.lidar
                print(
                    f"\r  t={obs.timestamp:.2f}s  "
                    f"lidar={pc.num_points} pts  "
                    f"height={obs.base_state.position[2]:.3f}m",
                    end="",
                )

            viewer.sync()


if __name__ == "__main__":
    main()
