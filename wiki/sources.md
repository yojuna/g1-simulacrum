# Sources

Checked 2026-09-05. Pins below are what Architecture and future MJCF snapshots
must follow until we bump them on purpose.

## Canonical (use these first)

| What | Where | Pin / note |
|------|--------|------------|
| G1 body MJCF | [unitree_ros `g1_29dof_rev_1_0.xml`](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.xml) | Menagerie copies this. IMU **sites** live here. No lidar/camera bodies. |
| G1 sensor mounts | [unitree_ros `g1_29dof_rev_1_0.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_rev_1_0.urdf) | **`7c40519e02d7`** (2026-06-16, “fix g1 mid360_joint transform”) |
| Dex3 on G1 | [unitree_ros `g1_29dof_with_hand_rev_1_0.xml`](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml) | Extract wrist subtrees; pin SHA with the body snapshot |
| Standalone Dex3 | [dex3_1_l/r.urdf](https://github.com/unitreerobotics/unitree_ros/tree/master/robots/dexterous_hand_description/dex3_1) | Alternate include source |
| LiDAR rays | [MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR) | Pattern + backends. **Not** the mount pose (their G1 site is simplified) |
| Menagerie G1 | [mujoco_menagerie `unitree_g1`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1) | Derived from the Unitree MJCF above. Docker image clones this to `/opt/mujoco_menagerie`. |
| Low-level SDK | [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | G1 uses `unitree_hg` (`LowCmd_`, `LowState_`), not `unitree_go`. |
| G1 low-level example | [`g1_ankle_swing_example.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp) | 2 ms loop, `G1JointIndex` 0–28, pelvis vs torso IMU topics |
| Developer docs | [support.unitree.com G1](https://support.unitree.com/home/en/G1_developer) | Product DoF, FOV prose, lidar/camera service pages |
| Product page | [unitree.com/g1](https://www.unitree.com/g1/) | Size, mass, EDU options; not CAD |
| Mid-360 datasheet | [livoxtech.com/mid-360/specs](https://www.livoxtech.com/mid-360/specs) | FOV, range, precision, IMU model, mass |
| D435i product | [intelrealsense.com/depth-camera-d435i](https://www.intelrealsense.com/depth-camera-d435i/) | Depth/RGB FOV, rates, ideal range |
| D435i IMU | Intel RealSense SDK docs (D435i) | Bosch BMI055; gyro/accel streams, no factory IMU cal |

## Secondary (do not override canonical)

- Local trees of `unitree_ros` (often **behind** `7c40519`). Treat as stale
  unless `git log` shows the mid360 fix.
- Third-party recaps (Isaac ROS, vendor ROS drivers). Useful for topic
  names; not for mount poses.
- [unitree_ros#121](https://github.com/unitreerobotics/unitree_ros/issues/121)
  — pelvis IMU is `LowState`; torso is a second topic.

## How to bump a pin

1. Diff the official URDF/MJCF against the numbers in [g1-sensors.md](g1-sensors.md).
2. Add a dated “revision” row (old → new, SHA, one-line why).
3. Change Architecture **and** the pinned MJCF in the same change set.
4. Do not silently “fix” a pose in Python.
