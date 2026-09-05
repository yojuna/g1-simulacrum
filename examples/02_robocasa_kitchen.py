"""Example 2: G1 in a RoboCasa kitchen environment.

Drops the sensorized G1 into a RoboCasa kitchen scene and runs the
Gym interface, demonstrating how the full sensor suite interacts with
a rich indoor environment (cabinets, appliances, objects).

Usage:
    pip install 'g1-simulacrum[robocasa,gym]'
    python examples/02_robocasa_kitchen.py
"""

import numpy as np

from g1_simulacrum import G1SimulacrumConfig
from g1_simulacrum.interface.gym_env import G1SimulacrumEnv


def main():
    config = G1SimulacrumConfig()
    config.controller.type = "pd"  # standalone PD (no SONIC needed)
    config.environment.type = "robocasa"
    config.environment.robocasa_scene = "kitchen-001"
    config.sensors.mid360.backend = "cpu"
    config.sensors.d435i.resolution = (320, 240)  # lower res for speed
    config.interface.gym.obs_keys = ["proprioception", "lidar", "depth", "rgb"]

    env = G1SimulacrumEnv(config=config, render_mode="human")

    obs, info = env.reset()
    print(f"Observation keys: {list(obs.keys())}")
    for key, val in obs.items():
        print(f"  {key}: shape={val.shape}, dtype={val.dtype}")

    # Run for 1000 steps with zero action (standing still)
    for step in range(1000):
        action = np.zeros(29, dtype=np.float64)
        obs, reward, terminated, truncated, info = env.step(action)

        if step % 100 == 0:
            print(
                f"Step {step}: reward={reward:.4f}, "
                f"height={info['base_height']:.3f}m, "
                f"sim_time={info['sim_time']:.2f}s"
            )

        if terminated or truncated:
            print(f"Episode ended at step {step}")
            obs, info = env.reset()

    env.close()
    print("Done.")


if __name__ == "__main__":
    main()
