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
(`HandController`) is still deferred. Body+sensors core is in; downstream
stacks compose with this object.

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
| `d435i_link` | Camera body (depth + RGB cameras on this body; look along **+X**) |
| `d435i_color_optical_frame` | RGB observation `frame_id` (OpenCV: +Z forward, +X right, +Y down) |

LiDAR rays originate at a **site on `mid360_link`**, never at the torso origin.
Mounts change only by editing XML (and the wiki if Unitree revises the URDF).

[`mujoco-lidar`](https://github.com/discoverse-dev/MuJoCo-LiDAR) does not
own the mount. It traces `LivoxGenerator("mid360")` rays in whatever site
we name. Discoverse’s G1 demo uses a **simplified** site on `torso_link`
(`pos="0 0 0.405" quat="0 0 1 0"` = 180° about Y). That is the same *idea*
(optical +Z down so the +52° lobe covers the floor) but **not** the Unitree
URDF (roll π, 2.93° pitch, z=0.428434, +X still forward). We pin Unitree.
No API conflict: `MjLidarWrapper(model, site_name="mid360", …)` plus
`bodyexclude=torso_link` so rays skip the head mesh, and `geomgroup[4]=0`
so the cosmetic Mid-360 cylinder (geom group 4) is not a hit. Do not copy
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

The compiler is the composer. The **runtime** loader does not mutate XML
(`from_xml_path` only). Authoring (`scripts/pin_mjcf.py`) uses named
ElementTree edits; that script is not on the step path.

### Files

Pin record: [`g1_simulacrum/model/mjcf/PIN.md`](g1_simulacrum/model/mjcf/PIN.md).
Pristine Unitree XML/URDF live under `mjcf/upstream/`; STLs under `mjcf/assets/`.
Authoring: `docker/run.sh python scripts/pin_mjcf.py` (optional `--fetch`).

| File | Role |
|------|------|
| `g1_simulacrum/model/mjcf/g1_29dof.xml` | Composed Dex3 robot (body + mounts + hands). Copy, not a nested include of a full `<mujoco>` document |
| `g1_simulacrum/model/mjcf/g1_29dof_none.xml` | Same body+sensors, rubber hands |
| `g1_simulacrum/model/mjcf/mounts/mid360.xml` | Lidar body at URDF pose, site `mid360`, group-4 visual cylinder, device IMU site |
| `g1_simulacrum/model/mjcf/mounts/d435i.xml` | Camera body at URDF pose; **two** cameras looking along camera-link **+X** (depth fovy 58°, RGB fovy 42°) |
| `g1_simulacrum/model/mjcf/mounts/imus.xml` | Accel/gyro for pelvis, torso, Mid-360, D435i sites |
| `g1_simulacrum/model/mjcf/end_effectors/dex3/{left,right}.xml` | Dex3-1 subtree + actuators, snapshot from Unitree `g1_29dof_with_hand_rev_1_0` |
| `g1_simulacrum/model/mjcf/end_effectors/none/{left,right}.xml` | Rubber-hand visual only (from `g1_29dof_rev_1_0`) |
| `g1_simulacrum/model/mjcf/g1_robot.xml` | Default robot file the scenes include (`g1_29dof.xml` contents) |
| `g1_simulacrum/model/mjcf/g1_robot_none.xml` | Rubber-hand robot file |
| `g1_simulacrum/model/mjcf/g1_sensorized.xml` | Default complete model: `g1_robot.xml` + floor |
| `g1_simulacrum/model/mjcf/g1_sensorized_none.xml` | Floor scene with rubber hands |
| `g1_simulacrum/model/mjcf/g1_inspect.xml` | Same include + coloured boxes (inspect viewer). Same directory so `meshdir=assets` resolves |

Keep pelvis/torso IMU sites from Unitree; do not delete them when adding
lidar/camera.

Fragments (`mounts/`, `end_effectors/`) **must** be rooted at
`<mujocoinclude>`. Nested includes of a full `<mujoco>` document duplicate
the tree. `g1_robot.xml` is therefore a copy of the composed body, not
`<include file="g1_29dof.xml"/>` of another complete model.

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

D435i: Unitree `d435_link` is a ROS camera_link (**+X** forward, +Z up).
MuJoCo cameras default to looking along **+Z**. Both cameras use
`xyaxes="0 -1 0 0 0 1"` so they look along body +X (the 47.6° URDF pitch
then aims at the floor, not the sky).

`compiler meshdir=assets` is relative to these MJCF files. Python only
calls `mujoco.MjModel.from_xml_path` on `g1_sensorized.xml`,
`g1_inspect.xml`, or a scene that includes `g1_robot.xml`.

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
  `bodyexclude=torso_link` and skip geom **group 4** (cosmetic lidar/camera
  housings) so rays hit the floor and scene, not a 3 cm shell. Default
  backend is **CPU**.
- `D435iCamera` — MuJoCo renderer: depth camera **fovy 58°** (87° H at 4:3
  is the RealSense depth FOV), RGB camera **fovy 42°**. Cameras look along
  `d435i_link` **+X**. Do not render both from one camera. This is a pinhole
  stand-in for stereo (see wiki). On **MuJoCo 3.12**,
  `Renderer.enable_depth_rendering()` already returns **metric metres**;
  do not apply the older OpenGL z-buffer formula a second time.
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
at reset qpos with a light named-actuator PD (`FINGER_HOLD_KP` /
`FINGER_HOLD_KD` in `gains.py`). They must not flop. `Observation.q_hands`
is the 14 finger qpos (or `None` for `hands: none`); `step()` still takes
body `(29,)` only.

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

### 6. Inspect viewer (example, not the core API)

`examples/01_empty_arena.py` is the live GLFW check. It is not a second
facade. From `docker/`:

```bash
./run.sh python examples/01_empty_arena.py
```

Default scene is `g1_inspect.xml` (floor + boxes). `--empty` compiles
`g1_sensorized.xml` instead.

**Gantry** (`g1_simulacrum/gantry.py`, `ElasticBand`): spring-damper wrench
on `pelvis` via `data.xfrc_applied`. Same idea as Unitree MuJoCo /
GEAR-SONIC. The freejoint stays free — this is not a weld and not a
balance policy. Keys **7 / 8** change length, **9** toggles. `--no-gantry`
/ `--free-base` lets the robot fall (PD is joints only).

**Cameras:** click the 3D view, **C** cycles free → `d435i_rgb` →
`d435i_depth` (or Rendering → Camera in the right panel). No PiP window.

**Overlays:** green Mid-360, cyan D435i depth. YAML `viewer:` sets density;
CLI `--overlay sparse|dense|full` and `--lidar-dots` / `--depth-stride`
override. Dense default is all ~24k lidar returns and every 4th depth
pixel. User geoms are **boxes** inited once, then only `pos` is written,
and clouds refresh at sensor rate (not every display frame). MuJoCo has
no point-sprite primitive. Overlay shadows/reflections are off.

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

viewer:                         # inspect overlay only; not sensor rates
  lidar_dots: 0                 # 0 = all Mid-360 returns (~24k)
  depth_stride: 4
  lidar_radius: 0.006
  depth_radius: 0.008

render: true
seed: 42
```

`robot.hands` selects which **pinned scene XML** to compile
(`g1_sensorized.xml` for Dex3, `g1_sensorized_none.xml` for rubber). Those
scenes include `g1_robot.xml` / `g1_robot_none.xml`. It is not a pose patch.

The core schema has no `sonic` controller, no `robocasa` environment, and
no `ros2` interface block. `interface/gym_env.py` is an optional extra,
not a required config key.

---

## Directory (v1)

What is in the tree today. Deferred work is listed under Later, not as
missing files here.

```
g1_simulacrum/                    git root
├── pyproject.toml
├── README.md
├── ARCHITECTURE.md               this file (normative decisions)
├── wiki/                         compiled G1 hardware facts + sources
├── configs/default.yaml
├── docs/docker_usage.md
├── docs/inspect_viewer.png       README screenshot
├── docker/                       run.sh image
├── scripts/pin_mjcf.py           authoring only; runtime does not run this
├── examples/01_empty_arena.py    GLFW inspect viewer
├── tests/test_core_v1.py
└── g1_simulacrum/
    ├── __init__.py               G1Simulacrum, G1SimulacrumConfig
    ├── cli.py                    compile + print maps
    ├── simulacrum.py
    ├── config.py
    ├── gantry.py                 inspect ElasticBand; not the facade
    ├── model/
    │   ├── joints.py             BODY_JOINT_NAMES / HAND_JOINT_NAMES
    │   ├── loader.py
    │   └── mjcf/                 pinned XML + PIN.md + upstream/ + assets/
    ├── sensors/                  base, mid360, d435i, imu, manager, noise, data_types
    ├── controllers/
    │   ├── base.py               body API + Dex3 hold_hands
    │   ├── pd.py
    │   ├── passthrough.py
    │   └── gains.py              SDK2-cited body PD; FINGER_HOLD_*
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
- **Gym extra** — `interface/gym_env.py` exists as a thin optional wrapper.
  Do not make it the core API. No padded lidar `Box` as the package default.
- **ROS2 extra** — topics from `Observation`, not a second robot.
- **RoboCasa** — a scene that includes `g1_robot.xml`, or a separate project.
- **GPU lidar backends** — Warp (and others) as a **config flag** after CPU
  is the measured default. The image may install `mujoco-lidar[warp]`;
  `sensors.mid360.backend` stays `cpu` until we switch on purpose.
- **Calibrated noise from robot logs** — datasheet 1σ is the start, not the end.
- **23 DoF locked-waist XML** — same package, different MJCF entry.
- **Mid-360s** — G1 units after April 2026 may ship Livox Mid-360s. Stay on
  Mid-360 until we pin that datasheet.

---

## Dependencies (core)

| Dependency | Role |
|------------|------|
| `mujoco` | Physics, renderer |
| `mujoco-lidar` | Mid-360 rays (CPU default; Warp extra in the image, not the YAML default) |
| `numpy`, `scipy` | Arrays, noise helpers |
| `pydantic`, `pyyaml` | Config |
| `unitree_ros` G1 | **Pinned** in `mjcf/upstream/` + `assets/` (`PIN.md`). Not an unpinned pip import |
| Menagerie G1 | Docker clones it to `/opt/mujoco_menagerie` (`PYTHONPATH=/opt`). Runtime robot XML is the package pin, not this clone |

Optional extras (`gymnasium`, GPU lidar, etc.) stay extras. `unitree_sdk2py`
is not a core dependency.
