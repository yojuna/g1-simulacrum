# G1 hands (Dex3-1 and kits)

Default end-effector is **Unitree Dex3-1**, one per wrist. The *controller*
(DDS `HandCmd_`, finger PD) is **not** this page’s implementation stage;
the model and flange are.

Product facts: [unitree.com/g1](https://www.unitree.com/g1/) (7 DoF per
hand: thumb 3, index 2, middle 2). Hardware DDS (`rt/dex3/...`) is documented
by Unitree; this package does not speak it.

## Flange (stable across kits)

Every kit attaches with the same Unitree palm joints. Rubber hand and Dex3
share this origin; only the child link changes.

From `g1_29dof_rev_1_0.urdf` / `g1_29dof_with_hand_rev_1_0.urdf`:

| Joint | Parent | Child (rubber) | Child (Dex3) | xyz | rpy |
|-------|--------|----------------|--------------|-----|-----|
| `left_hand_palm_joint` | `left_wrist_yaw_link` | `left_rubber_hand` | `left_hand_palm_link` | `0.0415 0.003 0` | `0 0 0` |
| `right_hand_palm_joint` | `right_wrist_yaw_link` | `right_rubber_hand` | `right_hand_palm_link` | `0.0415 -0.003 0` | `0 0 0` |

Do not invent a second attachment frame. Inspire / Dex5 / gripper kits must
use this flange (or document a measured delta).

## What is pinned

The Dex3 (and rubber) kits are already extracted into
`g1_simulacrum/model/mjcf/end_effectors/`. Prefer that layout over inlining
a second copy of the 29-DoF body. Pin record: [`PIN.md`](../g1_simulacrum/model/mjcf/PIN.md)
at `unitree_ros@7c40519e02d7`.

| Source | Use |
|--------|-----|
| [`g1_29dof_with_hand_rev_1_0.xml`](https://github.com/unitreerobotics/unitree_ros/blob/7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74/robots/g1_description/g1_29dof_with_hand_rev_1_0.xml) | Finger bodies, joints, geoms, actuators under each `*_wrist_yaw_link` |
| [`g1_29dof_rev_1_0.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74/robots/g1_description/g1_29dof_rev_1_0.urdf) | Palm joint origins (same flange on the with-hand URDF) |
| [`dex3_1_l.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/dexterous_hand_description/dex3_1/dex3_1_l.urdf) / `_r.urdf` | Standalone hand if a later bump needs a clean include |
| `g1_29dof_rev_1_0` rubber geoms | `end_effectors/none/` |

Do not mix a rubber palm geom with Dex3 on the same wrist.

Menagerie `g1_with_hands.xml` is a processed copy of Unitree’s with-hand
MJCF. Same rule: the package pin is Unitree; do not fetch unpinned
Menagerie at runtime.

## Dex3 joints (14)

Unitree MJCF names (left shown; right is `right_hand_*`). IDL order is
**thumb, middle, index** — middle before index.

| MJCF joint | IDL index | Role |
|------------|-----------|------|
| `left_hand_thumb_0_joint` | 0 | thumb yaw (geared) |
| `left_hand_thumb_1_joint` | 1 | thumb proximal |
| `left_hand_thumb_2_joint` | 2 | thumb distal |
| `left_hand_middle_0_joint` | 3 | middle proximal |
| `left_hand_middle_1_joint` | 4 | middle distal |
| `left_hand_index_0_joint` | 5 | index proximal |
| `left_hand_index_1_joint` | 6 | index distal |

URDF uses `left_hand_zero`…`six` with a **left/right numbering swap** for
index vs middle. Prefer MJCF/IDL names in this package.

Approximate ranges from Unitree XML (left middle/index are mirrored
negative): thumb_0 ±1.047; thumb_1/2 and finger curls as in the official
MJCF. Copy limits from the snapshot, do not retype from memory.

## How kits swap (architecture)

MJCF `<include>` is compile-time. Python does not attach meshes.

```
mjcf/end_effectors/
  dex3/left.xml     # default
  dex3/right.xml
  none/left.xml     # rubber visual, 0 finger joints
  none/right.xml
  inspire/…         # later
  gripper/…         # later
```

`g1_robot.xml` includes `end_effectors/dex3/{left,right}.xml` on the wrist
links. `g1_robot_none.xml` includes `none/`. Config `robot.hands: dex3|none`
selects `g1_sensorized.xml` or `g1_sensorized_none.xml` (those scenes
include the matching `g1_robot_*.xml`).

A new kit = new folder + new `g1_robot_<kit>.xml`. Body 29, sensors, and
lidar site stay shared.

## Body API vs finger API

| Surface | Size | Notes |
|---------|------|--------|
| `step(q_target)` | `(29,)` | `G1JointIndex` only |
| Body PD / passthrough | 29 named actuators | never `ctrl[:29]` if hands exist |
| Finger joints | 14 (Dex3) or 0 | light PD hold at reset qpos until `HandController` |
| Hardware Dex3 | `rt/dex3/left/cmd` etc. | later extra, not core |

Do not extend `G1JointIndex` to 0–42.

## Core v1 vs next stage

**Now (model):** Dex3 in the compiled robot, collisions and inertias,
fingers held with `FINGER_HOLD_*` on named actuators.

**Next:** `HandController` writing named finger actuators; optional DDS
mirror. Same kit files.
