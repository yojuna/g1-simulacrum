# g1-simulacrum — Architecture

**A modular MuJoCo simulation package for the Unitree G1 humanoid with full sensor suite and GEAR-SONIC whole-body control integration.**

## Problem

Developing autonomous behaviors for a Unitree G1 equipped with a Livox Mid-360 LiDAR and RealSense D435i requires iterating in simulation with high fidelity before deploying on hardware. Today, the pieces exist (MuJoCo Menagerie G1 model, MuJoCo-LiDAR, GEAR-SONIC WBC) but are separate projects with incompatible interfaces. Wiring them together is a multi-week integration task that every team repeats.

## Solution

`g1-simulacrum` is a drop-in Python package that composes these components into a single coherent system. A downstream user writes:

```python
from g1_simulacrum import G1Simulacrum, environments

sim = G1Simulacrum(
    controller="sonic-v1.1",
    sensors=["mid360", "d435i"],
)
env = environments.load("robocasa:kitchen-001")
env.attach(sim)
obs = env.reset()

while True:
    action = my_policy(obs)
    obs, reward, done, info = env.step(action)
```

## Design Principles

1. **Composition over inheritance** — each layer is a standalone component that can be used independently or composed.
2. **MJCF-native** — sensor mounts, collision geoms, and actuators are defined in XML includes, not monkey-patched at runtime.
3. **Interface contracts** — every component exposes typed dataclasses for its inputs/outputs so downstream systems can rely on stable shapes.
4. **Backend-agnostic sensors** — the LiDAR abstraction works with CPU, Taichi, JAX, or Warp backends; switching is a config flag.
5. **Sim-to-real parity** — sensor noise models, control frequencies, and observation vectors match the real G1 hardware stack.

---

## Layer Architecture

```
┌─────────────────────────────────────────────────────────┐
│                   Downstream Consumers                   │
│         (nav stack, MoveIt, RL training, VLA)            │
├──────────────┬──────────────────────┬────────────────────┤
│  ROS2 Bridge │    Gym-style API     │   Raw Python API   │
├──────────────┴──────────────────────┴────────────────────┤
│                                                          │
│                  5. Interface Layer                       │
│          g1_simulacrum.interface                           │
│   ┌────────────┐ ┌────────────┐ ┌──────────────┐        │
│   │ ROS2Bridge │ │  GymEnv    │ │ ObsManager   │        │
│   └────────────┘ └────────────┘ └──────────────┘        │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                                                          │
│                  4. Environment Layer                    │
│          g1_simulacrum.environments                       │
│   ┌────────────┐ ┌────────────┐ ┌──────────────┐        │
│   │ RoboCasa   │ │ Empty      │ │ Custom MJCF  │        │
│   │ Adapter    │ │ Arena      │ │ Loader       │        │
│   └────────────┘ └────────────┘ └──────────────┘        │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                                                          │
│                  3. Controller Layer                      │
│          g1_simulacrum.controllers                         │
│   ┌────────────┐ ┌────────────┐ ┌──────────────┐        │
│   │ SONIC      │ │ PD Ctrl    │ │ Passthrough   │        │
│   │ Bridge     │ │ (native)   │ │ (torque)      │        │
│   └────────────┘ └────────────┘ └──────────────┘        │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                                                          │
│                  2. Sensor Layer                          │
│          g1_simulacrum.sensors                             │
│   ┌────────────┐ ┌────────────┐ ┌──────────────┐        │
│   │ Mid360     │ │ D435i      │ │ IMU          │        │
│   │ LiDAR      │ │ DepthCam   │ │ (accel+gyro) │        │
│   └────────────┘ └────────────┘ └──────────────┘        │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                                                          │
│                  1. Model Layer                           │
│          g1_simulacrum.model                               │
│   ┌────────────┐ ┌────────────┐ ┌──────────────┐        │
│   │ G1 MJCF    │ │ Sensor     │ │ Composed     │        │
│   │ (29 DOF)   │ │ Mounts XML │ │ Assembly     │        │
│   └────────────┘ └────────────┘ └──────────────┘        │
│                                                          │
├──────────────────────────────────────────────────────────┤
│                   External Dependencies                  │
│  mujoco  ·  mujoco-lidar  ·  mujoco_menagerie           │
│  unitree_sdk2  ·  gear-sonic  ·  robosuite/robocasa      │
└──────────────────────────────────────────────────────────┘
```

---

## Layer Details

### 1. Model Layer (`g1_simulacrum.model`)

**Purpose:** Assemble the G1 MJCF with sensor mount points, producing a single composite model that can be dropped into any MuJoCo scene.

**Key files:**
- `mjcf/g1_29dof.xml` — base G1 model (sourced from Menagerie, pinned version)
- `mjcf/sensor_mounts.xml` — MJCF `<include>` that adds sensor bodies, sites, cameras
- `mjcf/g1_sensorized.xml` — top-level composed model
- `model.py` — Python API to load, configure, and attach the model

**Sensor mount points on real G1:**
- **Mid-360 LiDAR**: head/torso top, pos `[0, 0, 0.05]` relative to `torso_link`
- **D435i cameras**: head link, facing forward (can be configured for wrist mount too)
- **IMU (Mid-360 built-in)**: co-located with LiDAR mount site
- **IMU (D435i built-in)**: co-located with camera body

**G1 joint configuration (29 DOF, matching GEAR-SONIC):**
```
Legs (12):   left/right × [hip_pitch, hip_roll, hip_yaw, knee, ankle_pitch, ankle_roll]
Waist (3):   waist_yaw, waist_roll, waist_pitch (→ torso_link anchor)
Arms (14):   left/right × [shoulder_pitch, shoulder_roll, shoulder_yaw, elbow,
              wrist_roll, wrist_pitch, wrist_yaw]
```

### 2. Sensor Layer (`g1_simulacrum.sensors`)

**Purpose:** Unified sensor interfaces that produce data matching real hardware output formats.

#### Mid-360 LiDAR (`sensors/mid360.py`)

Wraps `mujoco_lidar.LidarSensor` with the `mid360` preset. Outputs:
- `PointCloud` dataclass: `points: ndarray (N,3)`, `intensities: ndarray (N,)`, `timestamp: float`
- Configurable: backend (`cpu`/`taichi`/`jax`/`warp`), noise model, scan rate

Noise model parameters (matching real sensor):
- Range noise: Gaussian σ=0.02m
- Random point dropout: 2% of points
- Near-field clutter: 1% of points assigned random [0, 0.3] range

#### D435i Depth Camera (`sensors/d435i.py`)

Uses MuJoCo native `Renderer` for RGB + depth. Outputs:
- `DepthFrame` dataclass: `rgb: ndarray (H,W,3)`, `depth: ndarray (H,W)`, `timestamp: float`
- Intrinsics matching real D435i: 640×480 @ 30fps, 87° H-FOV (58° V-FOV)
- Depth noise model: edge erosion, distance-dependent precision degradation, hole injection on flat/reflective surfaces

#### IMU (`sensors/imu.py`)

Reads MuJoCo `accelerometer` and `gyro` sensors. Outputs:
- `ImuReading` dataclass: `accel: ndarray (3,)`, `gyro: ndarray (3,)`, `timestamp: float`
- Noise model: bias drift, white noise (matching ICM-40609-D specs for Mid-360, BMI055 for D435i)

#### Sensor Manager (`sensors/manager.py`)

Orchestrates all sensors with configurable rates:
```python
manager = SensorManager(model, data, config={
    "mid360": {"rate_hz": 10, "backend": "warp"},
    "d435i":  {"rate_hz": 30, "resolution": [640, 480]},
    "imu":    {"rate_hz": 200},
})
readings = manager.step(sim_time)
# → SensorBundle(lidar=PointCloud, depth=DepthFrame, imu=ImuReading)
```

### 3. Controller Layer (`g1_simulacrum.controllers`)

**Purpose:** Bridge between high-level action commands and MuJoCo actuator torques.

#### SONIC Bridge (`controllers/sonic_bridge.py`)

Interfaces with GEAR-SONIC deployment stack via DDS (Unitree SDK2):

```
┌──────────────────┐     DDS loopback      ┌───────────────────┐
│  g1-simulacrum    │ ◄──── rt/lowcmd ────── │  gear_sonic_deploy │
│  (MuJoCo 200Hz)  │ ────► rt/lowstate ───► │  (ONNX 50Hz)      │
│                  │ ────► rt/odostate ───► │                   │
│                  │ ────► rt/secondary_imu─►│                   │
└──────────────────┘                        └───────────────────┘
```

- Publishes: `rt/lowstate` (joint pos/vel/torque), `rt/odostate` (base pose/vel), `rt/secondary_imu`
- Subscribes: `rt/lowcmd` (target joint positions from policy)
- Applies PD control: `τ = Kp(q_target - q) + Kd(0 - q̇)` at 200Hz
- Per-joint PD gains from SONIC config (Kp: 4–400, Kd: 0.1–5.0)

#### PD Controller (`controllers/pd_controller.py`)

Standalone PD position controller (no SONIC dependency). Accepts raw joint position targets.

#### Passthrough Controller (`controllers/passthrough.py`)

Direct torque control for custom RL policies that output torques.

### 4. Environment Layer (`g1_simulacrum.environments`)

**Purpose:** Adapt various MuJoCo scene sources into a uniform environment API.

#### Base Environment (`environments/base.py`)

```python
class G1Environment:
    def attach(self, robot: G1Simulacrum) -> None: ...
    def reset(self) -> Observation: ...
    def step(self, action: Action) -> tuple[Observation, float, bool, dict]: ...
    def render(self) -> ndarray | None: ...
```

#### RoboCasa Adapter (`environments/robocasa_adapter.py`)

- Loads RoboCasa kitchen scenes (120 scenes, 2500+ objects)
- Replaces the default robot model with sensorized G1
- Preserves RoboCasa's task definitions and reward functions
- Handles MJCF merging: G1 model → RoboCasa Arena → composed scene

#### Empty Arena (`environments/empty_arena.py`)

Flat ground plane with configurable obstacles. For locomotion testing.

#### Custom MJCF Loader (`environments/custom_loader.py`)

Loads any user-provided MJCF/XML scene and attaches the G1.

### 5. Interface Layer (`g1_simulacrum.interface`)

**Purpose:** Connect the simulation to downstream systems.

#### Gym API (`interface/gym_env.py`)

Wraps everything into a `gymnasium.Env`:

```python
observation_space = Dict({
    "proprioception": Box(shape=(29*2 + 6,)),  # joint pos/vel + base vel
    "lidar": Box(shape=(N, 3)),                 # point cloud
    "depth": Box(shape=(480, 640)),              # depth image
    "rgb": Box(shape=(480, 640, 3)),             # RGB image
    "imu": Box(shape=(6,)),                      # accel + gyro
})
action_space = Box(shape=(29,))  # joint position targets
```

#### ROS2 Bridge (`interface/ros2_bridge.py`)

Publishes standard ROS2 topics:

| Topic                      | Message Type              | Rate  |
|----------------------------|---------------------------|-------|
| `/g1/lidar/points`        | `sensor_msgs/PointCloud2` | 10 Hz |
| `/g1/camera/color/image`  | `sensor_msgs/Image`       | 30 Hz |
| `/g1/camera/depth/image`  | `sensor_msgs/Image`       | 30 Hz |
| `/g1/camera/camera_info`  | `sensor_msgs/CameraInfo`  | 30 Hz |
| `/g1/imu/data`            | `sensor_msgs/Imu`         | 200Hz |
| `/g1/joint_states`        | `sensor_msgs/JointState`  | 200Hz |
| `/g1/odom`                | `nav_msgs/Odometry`       | 50 Hz |
| `/tf`                     | `tf2_msgs/TFMessage`      | 50 Hz |

Subscribes:

| Topic                      | Message Type                  | Use           |
|----------------------------|-------------------------------|---------------|
| `/g1/cmd_vel`             | `geometry_msgs/Twist`         | Velocity cmd  |
| `/g1/joint_cmd`           | `trajectory_msgs/JointTrajectory` | Joint targets |

#### Observation Manager (`interface/obs_manager.py`)

Assembles the SONIC-compatible observation vector:
- Base linear velocity (3)
- IMU gravity projection (3)
- Joint positions (29)
- Joint velocities (29)
- Previous actions (29)
- Phase signals (2)

Total: 95-dimensional proprioceptive observation (matches SONIC training).

---

## Directory Structure

```
g1_simulacrum/
├── pyproject.toml
├── README.md
├── ARCHITECTURE.md
│
├── g1_simulacrum/
│   ├── __init__.py
│   ├── simulacrum.py                    # G1Simulacrum top-level facade
│   ├── config.py                   # Pydantic config models
│   │
│   ├── model/
│   │   ├── __init__.py
│   │   ├── loader.py               # MJCF loading and composition
│   │   ├── mjcf/
│   │   │   ├── g1_29dof.xml        # Base G1 (from Menagerie, pinned)
│   │   │   ├── sensor_mounts.xml   # Sensor bodies, sites, cameras
│   │   │   └── g1_sensorized.xml   # Composed top-level model
│   │   └── assets/                 # Meshes (G1, D435i)
│   │
│   ├── sensors/
│   │   ├── __init__.py
│   │   ├── base.py                 # Abstract sensor interface
│   │   ├── mid360.py               # Livox Mid-360 LiDAR wrapper
│   │   ├── d435i.py                # RealSense D435i depth + RGB
│   │   ├── imu.py                  # IMU sensor wrapper
│   │   ├── noise.py                # Sensor noise models
│   │   ├── manager.py              # Multi-sensor orchestrator
│   │   └── data_types.py           # PointCloud, DepthFrame, ImuReading
│   │
│   ├── controllers/
│   │   ├── __init__.py
│   │   ├── base.py                 # Abstract controller interface
│   │   ├── sonic_bridge.py         # GEAR-SONIC DDS bridge
│   │   ├── pd_controller.py        # Standalone PD controller
│   │   ├── passthrough.py          # Direct torque passthrough
│   │   └── gains.py                # Per-joint PD gain configs
│   │
│   ├── environments/
│   │   ├── __init__.py
│   │   ├── base.py                 # Base environment ABC
│   │   ├── robocasa_adapter.py     # RoboCasa scene adapter
│   │   ├── empty_arena.py          # Flat ground + obstacles
│   │   └── custom_loader.py        # User MJCF scene loader
│   │
│   └── interface/
│       ├── __init__.py
│       ├── gym_env.py              # gymnasium.Env wrapper
│       ├── ros2_bridge.py          # ROS2 topic publisher/subscriber
│       └── obs_manager.py          # SONIC-format observation builder
│
├── configs/
│   ├── default.yaml                # Default full-stack config
│   ├── sonic_v1_1.yaml             # SONIC v1.1 matched config
│   ├── lidar_only.yaml             # LiDAR-only (no camera) config
│   └── headless.yaml               # Headless training config
│
├── examples/
│   ├── 01_walk_around.py           # SONIC keyboard-driven walking
│   ├── 02_robocasa_kitchen.py      # G1 in RoboCasa kitchen scene
│   ├── 03_nav_stack.py             # ROS2 Nav2 integration
│   ├── 04_custom_scene.py          # Custom MJCF environment
│   ├── 05_rl_training.py           # RL policy training with Gym API
│   └── 06_sensor_visualization.py  # Visualize all sensor outputs
│
├── tests/
│   ├── test_model_loading.py
│   ├── test_sensors.py
│   ├── test_sonic_bridge.py
│   ├── test_environments.py
│   └── test_gym_env.py
│
└── docker/
    ├── Dockerfile                  # Full stack with ROS2
    ├── Dockerfile.headless         # Headless training
    └── docker-compose.yaml         # Multi-container (sim + SONIC deploy)
```

---

## Timing and Control Frequencies

All frequencies match the real G1 hardware stack and SONIC training:

| Component              | Rate     | Notes                                    |
|------------------------|----------|------------------------------------------|
| MuJoCo physics         | 1000 Hz  | `model.opt.timestep = 0.001`            |
| PD control loop        | 200 Hz   | Matches Unitree SDK2 low-level rate      |
| SONIC policy inference  | 50 Hz    | Encoder + decoder cycle                  |
| SONIC planner           | 10 Hz    | Kinematic trajectory generation          |
| IMU sensor              | 200 Hz   | Matches PD control rate                  |
| D435i camera            | 30 Hz    | Matches real D435i default               |
| Mid-360 LiDAR           | 10 Hz    | 200k points/sec, ~20k points/scan        |
| ROS2 TF broadcast       | 50 Hz    | State estimation rate                    |

---

## Integration Points

### With GEAR-SONIC (`NVlabs/GR00T-WholeBodyControl`)

The SONIC bridge runs as a separate process. The sim publishes robot state over DDS and receives joint commands back. This matches the real deployment architecture exactly — SONIC doesn't know if it's talking to MuJoCo or real hardware.

```bash
# Terminal 1: Simulation
python -m g1_simulacrum.run --config configs/sonic_v1_1.yaml

# Terminal 2: SONIC policy
cd GR00T-WholeBodyControl/gear_sonic_deploy
./deploy.sh --cp policy/sonic_v1_1/model \
             --obs-config policy/sonic_v1_1/observation_config.yaml \
             sim
```

### With RoboCasa

RoboCasa uses RoboSuite's Arena/Robot/Task composition. The adapter:
1. Loads the RoboCasa scene XML (arena + objects + fixtures)
2. Removes the default robot model
3. Injects the sensorized G1 model at the spawn point
4. Wires the G1's actuators into RoboSuite's controller interface
5. Preserves task rewards and success criteria

### With GR00T N1.7 VLA

The full perception → action pipeline:
```
Mid-360 + D435i → ROS2 topics → GR00T N1.7 VLA → action targets
                                                     ↓
SONIC decoder → joint positions → PD controller → MuJoCo torques
```

### With Nav2 / Navigation Stacks

The ROS2 bridge publishes standard `PointCloud2`, `Image`, `Imu`, `Odometry`, and `TF` — any ROS2 navigation stack (Nav2, RTAB-Map, cartographer) can consume these directly without custom adapters.

---

## Configuration

All configuration is via YAML with Pydantic validation:

```yaml
# configs/default.yaml
robot:
  model: "g1_29dof"
  initial_height: 0.82     # standing height (m)

sensors:
  mid360:
    enabled: true
    backend: "cpu"          # cpu | taichi | jax | warp
    rate_hz: 10
    noise:
      range_sigma: 0.02
      dropout_rate: 0.02
    mount_body: "torso_link"
    mount_pos: [0.0, 0.0, 0.05]

  d435i:
    enabled: true
    rate_hz: 30
    resolution: [640, 480]
    noise:
      edge_erosion: true
      depth_noise_sigma: 0.005
    mount_body: "head_link"
    mount_pos: [0.05, 0.0, 0.0]
    mount_quat: [1, 0, 0, 0]   # forward-facing

  imu:
    enabled: true
    rate_hz: 200
    noise:
      accel_sigma: 0.01
      gyro_sigma: 0.005
      bias_drift: 0.0001

controller:
  type: "sonic"               # sonic | pd | passthrough
  physics_hz: 1000
  control_hz: 200
  sonic:
    dds_domain: 0
    checkpoint: "sonic_v1_1"

environment:
  type: "empty_arena"         # empty_arena | robocasa | custom
  ground_friction: [1.0, 0.005, 0.0001]

interface:
  gym:
    enabled: true
    obs_keys: ["proprioception", "lidar", "depth"]
  ros2:
    enabled: false
    namespace: "/g1"
```

---

## Dependency Matrix

| Dependency          | Version     | Required | Purpose                   |
|---------------------|-------------|----------|---------------------------|
| `mujoco`           | ≥3.2        | Yes      | Physics engine             |
| `mujoco-lidar`     | ≥0.3        | Yes      | Mid-360 LiDAR simulation   |
| `mujoco_menagerie` | latest      | Yes      | G1 and D435i MJCF models   |
| `numpy`            | ≥1.24       | Yes      | Array operations           |
| `pydantic`         | ≥2.0        | Yes      | Config validation          |
| `gymnasium`        | ≥0.29       | Optional | Gym API wrapper            |
| `unitree_sdk2py`   | ≥0.1        | Optional | DDS bridge for SONIC       |
| `rclpy`            | ROS2 Jazzy  | Optional | ROS2 bridge                |
| `robosuite`        | ≥1.5        | Optional | RoboCasa adapter           |
| `robocasa`         | ≥0.2        | Optional | Kitchen environments       |
| `taichi`           | ≥1.7        | Optional | GPU LiDAR backend          |
| `jax`              | ≥0.4        | Optional | JAX LiDAR backend          |
| `warp-lang`        | ≥1.0        | Optional | Warp LiDAR backend         |
