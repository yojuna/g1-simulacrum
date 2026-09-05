# G1 platform

This package’s default robot is **G1 EDU, 29 actuated body joints + Dex3-1
hands (7 DoF each)**. Body joints match SDK2 `G1JointIndex` 0–28 and Unitree
`g1_29dof_rev_1_0`. Fingers are a separate kit; see [g1-hands.md](g1-hands.md).

## Variants (official)

From [unitree.com/g1](https://www.unitree.com/g1/) and
[G1 developer docs](https://support.unitree.com/home/en/G1_developer)
(product-page numbers; “about” / “contact sales” apply):

| | G1 (base) | G1 EDU |
|--|-----------|--------|
| Standing envelope | 1320 × 450 × 200 mm | same |
| Folded envelope | 690 × 450 × 300 mm | same |
| Mass with battery | ~35 kg | ~35 kg+ |
| Joint DoF | **23** | **23–43** |
| Leg DoF (each) | 6 | 6 |
| Waist DoF | 1 | 1 + optional 2 |
| Arm DoF (each) | 5 | 5 + optional 2 wrist |
| Hands | — | optional Dex3-1 (7 DoF/hand) + optional tactile |
| Knee max torque | 90 N·m | 120 N·m |
| Arm max load | ~2 kg | ~3 kg |
| Thigh + calf | 0.6 m | 0.6 m |
| Arm span | ~0.45 m | ~0.45 m |
| Sensing | depth camera + 3D LiDAR | same |
| Secondary development | no | yes (Jetson Orin class on EDU) |
| Battery | 9000 mAh, ~2 h | same |

DoF arithmetic used here:

- **23** = 12 leg + 1 waist yaw + 10 arm (5 per arm: shoulder 3 + elbow + wrist roll).
- **29** = 23 + waist roll + waist pitch + 2×(wrist pitch + wrist yaw).
- **43** = 29 + two Dex3-1 hands (7+7). **This is the default sim DoF count**
  (29 actuated body + 14 finger). Body API stays 29.

SDK2 comments mark waist roll/pitch **invalid** on 23-DoF and on 29-DoF
**with waist locked**, and wrist pitch/yaw **invalid** on 23-DoF. See
[g1-control.md](g1-control.md).

## What this sim is / is not

| In this package | Not in this package |
|-----------------|---------------------|
| `g1_29dof_rev_1_0` kinematics + inertias | 23-DoF locked-waist XML (deferred) |
| Dex3-1 on the wrist flange (default) | Dex3 *controller* / DDS `rt/dex3/...` (next stage) |
| Mid-360 + D435i on `torso_link` | Actuated neck / `head_link` (G1 has none) |
| Pelvis + torso IMUs | Full SDK2 runtime |

Unitree’s docs say lidar and camera sit in the **head assembly**. The URDF
parent for both is still **`torso_link`**. There is no actuated head joint
in `g1_29dof_rev_1_0`.

## Joint range (product page, EDU)

Quoted from unitree.com (degrees). Use URDF `limit` tags in MJCF for the
sim; this table is the public spec, not a substitute for XML.

| Joint | Range (product page) |
|-------|----------------------|
| Waist | Z ±155°, X ±30°, Y ±30° (EDU 3-DoF waist) |
| Knee | 0–165° |
| Hip | P ±154°, R −30°–+170°, Y ±158° |
| Wrist (EDU) | P ±92.5°, Y ±92.5° |

## Model lineage

1. Unitree publishes `g1_29dof_rev_1_0.xml` + `.urdf` in `unitree_ros`.
2. [Menagerie `unitree_g1`](https://github.com/google-deepmind/mujoco_menagerie/tree/main/unitree_g1)
   copies the MJCF and adds defaults/keyframes/actuators.
3. This package **pins** Unitree at `unitree_ros@7c40519e02d7` into
   `g1_simulacrum/model/mjcf/upstream/` + `assets/`, then **adds** lidar,
   camera, and device-IMU sites from the URDF (`scripts/pin_mjcf.py`,
   record in [`PIN.md`](../g1_simulacrum/model/mjcf/PIN.md)). Runtime
   loads that pin (`from_xml_path`). It does not fetch unpinned Menagerie
   for the robot.
