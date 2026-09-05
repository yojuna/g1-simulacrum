# g1-simulacrum — Architecture

Normative for this repo. Code follows this file. If they disagree, this file
wins and the code is wrong.

Hardware facts (DoF, mounts, rates, datasheets) live in [`wiki/`](wiki/README.md).
This file records **design decisions**. Numbers here must match the wiki and
cite it. Do not invent poses or rates in code.

A **sensorized Unitree G1 EDU (29-DoF body + Dex3-1 hands by default)** in
MuJoCo: pinned body from Unitree/`g1_29dof_rev_1_0`, Livox Mid-360 and
RealSense D435i on `torso_link`, pelvis + torso IMUs, Dex3 on the wrist-yaw
flange, and a **500 Hz** PD (or torque) loop matching SDK2 low-level.
Hands are a **swap-in MJCF kit**, not a second robot. The Dex3 *controller*
is deferred until the body+sensors core is tested. Downstream stacks compose
with this object.

Runtime is `docker/run.sh`. See [`docs/docker_usage.md`](docs/docker_usage.md).

---

## Scope

**This package is**

- A pinned, sensorized G1 MJCF model the MuJoCo compiler can load.
- Modular end-effectors (Dex3 default) as wrist-yaw includes.
- Sensors that emit typed readings at hardware-like rates.
- Actuation: joint-position PD and torque passthrough **for the 29 body
  joints**. Hand PD is a later controller, not core v1.
- A Python facade (`G1Simulacrum`) that compiles, steps, resets, and returns
  an `Observation`.

**This package is not**

- GEAR-SONIC, DDS, or a 95-D policy observation. Whole-body control lives in
  `GR00T-WholeBodyControl`. How to compose the two is **out of scope** until
  we revisit it against that repo.
- A RoboCasa / RoboSuite task suite.
- A ROS2 robot driver.
- A Gym locomotion environment as the primary API.

Gym, ROS2, RoboCasa, and a SONIC adapter may return later as optional extras.
They do not shape the core.

---

## Principles

1. **Pinned MJCF includes** — mounts, sites, cameras, and sensors live in XML
   the compiler includes. Python does not rewrite trees, write temp XML, or
   inject bodies at runtime.
2. **Named joints, not slices** — 29 DoF follow Unitree `G1JointIndex` /
   Menagerie names. Never `qpos[7:36]` as the contract.
3. **Typed contracts** — `PointCloud`, `DepthFrame`, `ImuReading`,
   `JointState`, `BaseState`, `Observation` are the API. Downstream code
   depends on these, not on Gym spaces or ROS messages.
4. **Config-driven sensors** — rates, mounts (as XML, not live pose hacks),
   and noise come from YAML / Pydantic. No new magic numbers in sensor code.
5. **Lean core** — one way to compile, step, and read sensors. Adapters wait.

---

## Invariants

Facts and URLs: [`wiki/g1-platform.md`](wiki/g1-platform.md),
[`wiki/g1-sensors.md`](wiki/g1-sensors.md),
[`wiki/g1-control.md`](wiki/g1-control.md),
[`wiki/g1-hands.md`](wiki/g1-hands.md).

### Degrees of freedom

Default model is **G1 EDU 29 body DoF + Dex3-1 (7+7 finger DoF)**.
The 29 body joints match Unitree `G1JointIndex` 0–28 and
`g1_29dof_rev_1_0`. Fingers are a **separate named group**, never padded
into the 29. The base retail G1 is 23 DoF (waist roll/pitch and wrist
pitch/yaw locked). We do not treat 23 as a second product; a locked-joint
XML can come later. `hands: none` restores rubber-hand visuals only.

Canonical name order (same as SDK2 indices 0–28):

```
left_hip_pitch, left_hip_roll, left_hip_yaw, left_knee,
left_ankle_pitch, left_ankle_roll,
right_hip_pitch, right_hip_roll, right_hip_yaw, right_knee,
right_ankle_pitch, right_ankle_roll,
waist_yaw, waist_roll, waist_pitch,
left_shoulder_pitch, left_shoulder_roll, left_shoulder_yaw, left_elbow,
left_wrist_roll, left_wrist_pitch, left_wrist_yaw,
right_shoulder_pitch, right_shoulder_roll, right_shoulder_yaw, right_elbow,
right_wrist_roll, right_wrist_pitch, right_wrist_yaw
```

The loader builds a **name → MuJoCo id** map at compile time. Freejoint
base (`qpos[0:7]`, `qvel[0:6]`) stays separate from the 29.

### Frames and mounts

G1 has **no actuated head**. Unitree’s docs say lidar and camera sit in the
head assembly; the URDF parent for both is **`torso_link`**. Do not invent
`head_link` as a camera parent.

Poses from Unitree `g1_29dof_rev_1_0.urdf` at
`unitree_ros@7c40519e02d7` (2026-06-16, “fix g1 mid360_joint transform”).
xyz in metres, rpy in radians. Copy into pinned MJCF — do not mix with older
URDF snapshots (pre-fix Mid-360 had z=`0.41618`, rpy=`0, 0.04014, 0`).

| Body / site | Parent | xyz | rpy |
|-------------|--------|-----|-----|
| `mid360_link` | `torso_link` | `0.0002835, 0.00003, 0.428434` | `π, 0.051121, 0` (~180° roll, 2.93° pitch) |
| `d435i_link` (URDF `d435_link`) | `torso_link` | `0.0576235, 0.01753, 0.42987` | `0, 0.830777, 0` (~47.60° pitch) |
| `imu_in_torso` | `torso_link` | `-0.03959, -0.00224, 0.14792` | `0, 0, 0` |
| `imu_in_pelvis` | `pelvis` | `0.04525, 0, -0.08339` | `0, 0, 0` |

| Frame | Meaning |
|-------|---------|
| `world` | MuJoCo inertial |
| `pelvis` | Base / freejoint; **primary robot IMU** (`LowState`) |
| `torso_link` | **Secondary robot IMU** (`rt/secondary_imu`) |
| `mid360_link` | LiDAR optical; point clouds in this frame |
| `d435i_link` | Camera body (depth + RGB cameras on this body) |
| `d435i_color_optical_frame` | RGB optical (OpenCV: +Z forward, +X right, +Y down) |

LiDAR rays originate at a **site on `mid360_link`**, never at the torso origin.
Mounts change only by editing XML (and the wiki if Unitree revises the URDF).

[`mujoco-lidar`](https://github.com/discoverse-dev/MuJoCo-LiDAR) does not
own the mount. It traces `LivoxGenerator("mid360")` rays in whatever site
we name. Discoverse’s G1 demo uses a **simplified** site on `torso_link`
(`pos="0 0 0.405" quat="0 0 1 0"` = 180° about Y). That is the same *idea*
(optical +Z down so the +52° lobe covers the floor) but **not** the Unitree
URDF (roll π, 2.93° pitch, z=0.428434, +X still forward). We pin Unitree.
No API conflict: `MjLidarWrapper(model, site_name="mid360", …)` plus
`bodyexclude=torso_link` so rays skip the head mesh. Do not copy
discoverse’s site pose into our XML. Details: [`wiki/g1-sensors.md`](wiki/g1-sensors.md).

### Rates

| Loop | Rate | Source |
|------|------|--------|
| MuJoCo physics | 1000 Hz | Sim choice (`timestep="0.001"`) |
| PD / torque apply | **500 Hz** | SDK2 G1 low-level (2 ms). Two physics substeps per control step |
| Pelvis / torso IMU | 500 Hz | Same as low-state (read every control step) |
| Mid-360 IMU (ICM-40609) | 200 Hz | Livox datasheet |
| D435i IMU (BMI055) | gyro ~200 Hz, accel ~63–250 Hz | Intel; we may sample gyro at 200 Hz |
| D435i RGB | 30 Hz | Intel default stream |
| D435i depth | 30 Hz in this package (hardware up to 90 Hz) | Sim cost; wiki notes 90 Hz cap |
| Mid-360 scan | 10 Hz | Livox typical frame rate |

A sensor that is not due this control step returns `None` in `SensorBundle`.
Callers must not treat missing readings as zeros.

### Noise

Start from **datasheet 1σ**, labeled as such, not as “calibrated on our G1”:

- Mid-360 range: ≤ 2 cm @ 10 m, ≤ 3 cm @ 0.2 m (Livox). Dropout/clutter remain
  placeholders until we have robot logs.
- D435i: stereo approximation only (edge holes, range-dependent σ). MuJoCo is
  not a RealSense. Ideal depth range 0.3–3 m, not a flat 0.105–10 m quality.

Four IMU models: pelvis, torso, Mid-360, D435i. Do not share one noise block.

---

## MJCF (pinned includes)

The compiler is the composer. No `ElementTree` mutation, no
`NamedTemporaryFile` XML.

### Files

| File | Role |
|------|------|
| `g1_simulacrum/model/mjcf/g1_29dof.xml` | Pinned snapshot of Unitree/Menagerie `g1_29dof_rev_1_0` (body SHA + URDF mount SHA in wiki) |
| `g1_simulacrum/model/mjcf/mounts/mid360.xml` | Lidar body at URDF pose, site, visual geom, device IMU site |
| `g1_simulacrum/model/mjcf/mounts/d435i.xml` | Camera body at URDF pose; **two** cameras (depth fovy 58°, RGB fovy 42°) |
| `g1_simulacrum/model/mjcf/mounts/imus.xml` | Accel/gyro for pelvis, torso, Mid-360, D435i sites |
| `g1_simulacrum/model/mjcf/end_effectors/dex3/{left,right}.xml` | Dex3-1 subtree + actuators, snapshot from Unitree `g1_29dof_with_hand_rev_1_0` |
| `g1_simulacrum/model/mjcf/end_effectors/none/{left,right}.xml` | Rubber-hand visual only (from `g1_29dof_rev_1_0`) |
| `g1_simulacrum/model/mjcf/g1_robot.xml` | Default robot: G1 tree with includes on `torso_link` (sensors) and both `*_wrist_yaw_link` (**dex3**) |
| `g1_simulacrum/model/mjcf/g1_robot_none.xml` | Same body+sensors, rubber hands (`end_effectors/none`) |
| `g1_simulacrum/model/mjcf/g1_sensorized.xml` | Default complete model: `g1_robot.xml` + empty arena |
| `g1_simulacrum/model/mjcf/scenes/` | Optional scenes that `<include>` `g1_robot.xml` |

`g1_29dof.xml` is a snapshot we own. Keep pelvis/torso IMU sites from Unitree;
do not delete them when adding lidar/camera.

Sensor includes go on `torso_link`:

```xml
<include file="mounts/mid360.xml"/>
<include file="mounts/d435i.xml"/>
```

Hand includes go on the wrist-yaw links (Unitree flange). Default files
are Dex3. Swapping hands means compiling a different robot XML that
includes a different kit — not rewriting the tree in Python.

```xml
<!-- inside left_wrist_yaw_link -->
<include file="end_effectors/dex3/left.xml"/>
```

A later Inspire / Dex5 / gripper kit is another folder under
`end_effectors/` plus a `g1_robot_<kit>.xml` entry. Same flange pose.

`compiler meshdir` (and texture paths) are set so meshes resolve relative to
these MJCF files, whether loaded from the package or from `/opt/mujoco_menagerie`
copies we pin. Python only calls `mujoco.MjModel.from_xml_path` on
`g1_sensorized.xml` or on a scene that includes `g1_robot.xml`.

### Dropping the robot into a user scene

The user scene is MJCF that includes `g1_robot.xml` (or copies that include
line). Python does not merge worldbodies in memory. If a scene cannot use
`<include>`, that is a scene-authoring problem, not a reason to resurrect
runtime injection.

---

## Core modules

```
G1Simulacrum          facade: build / reset / step → Observation
  ModelLoader         from_xml_path only; body + hand name maps
  SensorManager       Mid-360, D435i, IMUs at their rates
  Controller          body PD or torque passthrough → named body actuators
  HandController      deferred; fingers held at keyframe in core v1
```

### 1. Model (`g1_simulacrum.model`)

`ModelLoader.build()` compiles pinned XML. It returns `MjModel`, the body
joint name map (29), and the hand joint name map (14 for Dex3, empty for
`none`). It does not accept a “patch this tree” API.

### 2. Sensors (`g1_simulacrum.sensors`)

- `Mid360Lidar` — `mujoco-lidar.MjLidarWrapper` on site **`mid360`** (on
  `mid360_link`). Pattern: `LivoxGenerator("mid360")`. Pass
  `bodyexclude=torso_link` (and keep the lidar visual on a geom group the
  wrapper can ignore) so the head mesh is not a hit. CPU first.
- `D435iCamera` — MuJoCo renderer: depth camera **fovy 58°** (87° H at 4:3
  is the RealSense depth FOV), RGB camera **fovy 42°**. Do not render both
  from one camera. This is a pinhole stand-in for stereo (see wiki).
- `ImuSensor` — **four** named pairs: `imu_in_pelvis`, `imu_in_torso`,
  Mid-360, D435i. Core `SensorBundle` exposes pelvis (primary) and torso
  (secondary). Device IMUs are optional fields.
- `SensorManager` — `step(sim_time) → SensorBundle`.
- `noise.py` — datasheet 1σ where cited; otherwise labeled placeholder.

`SensorBundle` fields are `None` when disabled or not due.

### 3. Actuation (`g1_simulacrum.controllers`)

Two implementations of `Controller`:

| Type | Input | Output |
|------|--------|--------|
| `pd` | `(29,)` position targets, canonical order | PD torques, gains from `gains.py` / config |
| `passthrough` | `(29,)` torques | written to `data.ctrl` |

Default is `pd`. There is no `sonic` controller type.

Gains stay in `gains.py` as a named table. Config may scale named joints,
not integer indices in comments that drift.

The body controller writes **named body actuators only**. It must not index
`data.ctrl[:29]` if hand actuators exist.

### 4. Hands (model now, controller later)

See [`wiki/g1-hands.md`](wiki/g1-hands.md).

The wrist-yaw links are a **flange**. Unitree uses the same fixed joint for
rubber hands and Dex3:

| Joint | Parent | xyz (m) | rpy |
|-------|--------|---------|-----|
| `left_hand_palm_joint` | `left_wrist_yaw_link` | `0.0415, 0.003, 0` | `0, 0, 0` |
| `right_hand_palm_joint` | `right_wrist_yaw_link` | `0.0415, -0.003, 0` | `0, 0, 0` |

Default kit is Dex3-1: extract the finger tree + palm geoms + 14 actuators
from Unitree `g1_29dof_with_hand_rev_1_0` into `end_effectors/dex3/`. Do not
keep rubber-hand geoms on the same wrist.

Until `HandController` exists: compile Dex3 kinematics, hold finger joints
at the keyframe (named actuators stay at default `ctrl`). They must not
flop. `Observation` may include `q_hands` for inspection; `step()` still
takes body `(29,)` only.

Swap kit = different `g1_robot_*.xml` include, same flange. Inspire, Dex5,
and a parallel gripper are later folders, not Python attachment hacks.

### 5. Facade (`G1Simulacrum`)

```python
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig

sim = G1Simulacrum.from_config("configs/default.yaml")
sim.build()                          # compile pinned MJCF
obs = sim.reset()
obs = sim.step(q_target)             # (29,) or None to hold last PD target
```

`Observation` is body joint state (29), optional `q_hands`, base state,
`SensorBundle`, timestamp, and the **previous** control-step **body** action
(not the action just applied).

`build(scene_xml=...)` is only valid if `scene_xml` is an MJCF file that
includes `g1_robot.xml` (or `g1_robot_none.xml`). No ad-hoc merge.

---

## Configuration

YAML + Pydantic (`G1SimulacrumConfig`). Core keys:

```yaml
robot:
  model: g1_sensorized          # pinned MJCF entry (not a Menagerie hunt)
  hands: dex3                   # dex3 | none; later: inspire, gripper, dex5

sensors:
  mid360: { enabled: true, backend: cpu, rate_hz: 10 }
  d435i:  { enabled: true, rate_hz: 30, resolution: [640, 480] }
  imu:
    pelvis: { enabled: true, rate_hz: 500 }   # LowState
    torso:  { enabled: true, rate_hz: 500 }   # rt/secondary_imu
    mid360: { enabled: true, rate_hz: 200 }   # device IMU
    d435i:  { enabled: true, rate_hz: 200 }   # gyro; accel may be slower

controller:
  type: pd                      # pd | passthrough
  physics_hz: 1000
  control_hz: 500               # SDK2 G1 low-level; not 200

render: true
seed: 42
```

`robot.hands` selects which **pinned robot XML** to compile (`g1_robot.xml`
for Dex3, `g1_robot_none.xml` for rubber). It is not a pose patch.

Drop `controller.type: sonic`, `environment.type: robocasa`, and `interface.ros2`
from the core schema. Gym can remain a later optional extra, not a required
config block.

---

## Directory (v1)

What we intend to keep. Absence of a listed file means implementation is
incomplete, not that a fifth layer is “coming.”

```
g1_simulacrum/                    git root
├── pyproject.toml
├── README.md
├── ARCHITECTURE.md               this file (normative decisions)
├── wiki/                         compiled G1 hardware facts + sources
├── configs/default.yaml
├── docs/docker_usage.md
├── docker/                       run.sh image
├── examples/                     small demos of the facade (no SONIC)
├── tests/
└── g1_simulacrum/
    ├── __init__.py               G1Simulacrum, G1SimulacrumConfig
    ├── simulacrum.py
    ├── config.py
    ├── model/
    │   ├── loader.py
    │   └── mjcf/                 pinned XML (body, mounts, end_effectors/)
    ├── sensors/                  as today: base, mid360, d435i, imu,
    │                             manager, noise, data_types
    ├── controllers/
    │   ├── base.py
    │   ├── pd.py
    │   ├── passthrough.py
    │   ├── gains.py
    │   └── hands.py              later: HandController; not core v1
    └── interface/
        └── gym_env.py            optional extra only; not the core API
```

---

## Later (explicitly deferred)

Revisit only when we open that work:

- **SONIC** — composition with the official `GR00T-WholeBodyControl` repo.
  Not a controller type in this package. No DDS in core.
- **HandController** — Dex3 (then Inspire / gripper) PD or SDK2-like cmd
  after body+sensors core is tested. No DDS in core.
- **Other hand kits** — Inspire, Dex5, parallel gripper: new
  `end_effectors/<kit>/` + `g1_robot_<kit>.xml`. Same flange.
- **Gym extra** — thin wrapper over `G1Simulacrum` if RL needs it. No padded
  lidar `Box`, no baked “don’t fall” reward as the package default.
- **ROS2 extra** — topics from `Observation`, not a second robot.
- **RoboCasa** — a scene that includes `g1_robot.xml`, or a separate project.
- **GPU lidar backends** — config flag after CPU is correct.
- **Calibrated noise from robot logs** — datasheet 1σ is the start, not the end.
- **23 DoF locked-waist XML** — same package, different MJCF entry.
- **Mid-360s** — G1 units after April 2026 may ship Livox Mid-360s. Stay on
  Mid-360 until we pin that datasheet.

---

## Dependencies (core)

| Dependency | Role |
|------------|------|
| `mujoco` | Physics, renderer |
| `mujoco-lidar` | Mid-360 rays (CPU) |
| `numpy`, `scipy` | Arrays, noise helpers |
| `pydantic`, `pyyaml` | Config |
| Menagerie G1 | **Pinned snapshot** in `mjcf/`, not an unpinned pip import |

Optional extras (`gymnasium`, GPU lidar, etc.) stay extras. `unitree_sdk2py`
is not a core dependency.
