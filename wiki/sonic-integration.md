# SONIC integration (g1-simulacrum as the MuJoCo plant)

How this package composes with **GEAR-SONIC deploy** (`g1_deploy_onnx_ref`)
over Unitree DDS. Not a hardware fact sheet: it is the integration log for
why the first bridge looked like GEAR and behaved like a different robot.

Runtime how-to: [`docs/sonic_dds.md`](../docs/sonic_dds.md).
Code: `extras/sonic_dds/`, `examples/02_sonic_dds_bridge.py`.
GEAR reference: `gear_sonic/scripts/run_sim_loop.py` + `base_sim.py` +
`unitree_sdk2py_bridge.py`.
Process/clock model (MCU vs lidar vs cameras):
[`sim-process-model.md`](sim-process-model.md).

## Goal

Keep **unchanged** GEAR deploy (`deploy.sh sim`). Replace GEAR’s MuJoCo
loop with g1-simulacrum so sim2sim can use this package’s Mid-360, D435i,
and RoboCasa scenes, while the policy still sees `rt/lowstate` / writes
`rt/lowcmd` as it does against GEAR’s sim.

Two processes, both **host network**, CycloneDDS domain 0 on `lo`:

1. Sim: `./run.sh sonic python examples/02_sonic_dds_bridge.py`
2. Deploy: GEAR docker `bash deploy.sh sim` — `]` init, `9` gantry, `T`
   planner, `O` operator.

There is no separate “bridge vs sim” split. One Python process owns MuJoCo,
the GLFW viewer, and DDS (same shape as GEAR `run_sim_loop.py`).

## Two products, one DDS pipe

| | GEAR `run_sim_loop` | g1-simulacrum (inspect / default YAML) |
|---|---|---|
| Job | Cheap plant for the ONNX policy | 1 ms plant + lidar/depth/kitchen |
| Scene | `scene_43dof.xml`, floor plane | Sensorized G1, optional RoboCasa |
| Physics | One `mj_step` at **5 ms** (`SIMULATE_DT`) | `timestep=0.001`, 1000 Hz physics |
| Control wall | **200 Hz** | **500 Hz** SDK2-style |
| Crane | Cartesian 6-D spring on **pelvis** to `(spawn_xy, 1)`, `kp=10000`, `kd=1000` | Unitree **cable** on **torso**, hook `z=2` |
| Joint dissipation | MJCF `damping=0.05`, `armature=0.01`, `frictionloss` | None in our body XML |
| Lidar / depth in the control loop | Off (`USE_SENSOR: False`) | On in `configs/default.yaml` |
| Spawn joints | XML zeros; deploy INIT interpolates to standing | XML zeros |

The policy (and `temporary_motion` heading math) was tuned on the **left**
column. Copying GEAR’s *flags* onto the right column does not make the
plants equal.

DDS packing itself was already aligned: 29 motors in `G1JointIndex` order,
`LowState` IMU from floating-base `qpos[3:7]` / `qvel[3:6]` (wxyz, same as
GEAR `PublishLowState`), torso `rt/secondary_imu` from `torso_link` xquat.

## What went wrong (in order)

### 1. Docker / DDS, then motion

Early blockers were not physics: `:local` image without host network, CycloneDDS
on `lo` (“not multicast-capable”), deploy looping on `LowState is not
available`. Once both sides used `:sonic` + host network, LowState flowed.
Motion still did not match GEAR.

### 2. Wrong crane (wonky, not yet exploding)

Inspect’s `ElasticBand` is a **unilateral cable**: tension along hook→body,
plus heading PD. SONIC attached that to `torso_link` and placed the hook from
**pelvis** `qpos[0:3]`. Rest length and force body disagreed.

GEAR’s class of the same name is a **Cartesian PD**:

```text
f = kp_pos * ((point - pos) + [0, 0, length]) + kd_pos * (0 - lin_vel)
```

with `point = (0, 0, 1)` at origin (we use `(spawn_xy, 1)` so RoboCasa spawn
does not yank across the kitchen), `kp_pos=10000`, `kd_pos=1000`, attach
**pelvis** when waist is enabled. Linear damping on all axes is what keeps
5 ms (their XML) or 1 ms (ours) from ringing.

A cable on the chest is a different wrench than a pelvis spring to `z=1`.
The policy tracks the latter.

### 3. `--gear-parity` copied the 5 ms step onto a 1 ms model (NaN)

GEAR: `model.opt.timestep = 0.005`, one `mj_step` per 200 Hz tick.

We set `timestep=0.005` on `g1_sensorized.xml` (authored at 1 ms, no hinge
damping). Measured (sensors off, no LowCmd):

| Setup | Result |
|---|---|
| Our cable + attitude on torso, 5 ms | NaN QACC at **t = 0.625 s** (125 × 0.005), then `z = -16.7 m` |
| Cable only (`kp_ang=0`), 5 ms | Survived 1 s |
| Same gantry, **5 × 1 ms** substeps | Survived 1 s |
| **GEAR’s own 6-D spring** on our pelvis/torso, 5 ms | Still NaN |
| Default 1 ms, 2 substeps, our gantry | Stable |

So the 5 ms Euler step on **this** MJCF is an unstable plant. GEAR’s spring
is not a stabilizer once you leave their XML. `Time = 0.6250` then stuck
because `mj_step` refuses to advance after NaN; fall reset reprinted the
same time.

`--gear-parity` also left `loop.control_hz=500`, so wall aimed at 2 ms while
each tick advanced 5 ms of sim (2.5× realtime). That did not cause the NaN;
it was still not GEAR’s loop.

### 4. Policy `max_abs_q` of 8–13 rad (off-distribution)

After the crane and 1 ms substeps were fixed, INIT LowCmd was the standing
pose (`-0.312, 0, 0, 0.669, …`). As soon as mode `g1` ran, `q` jumped to
several radians. Deploy logs showed **LowState age 13–75 ms** on a 50 Hz
controller that wants ~20 ms.

Cause: `step_physics()` always ran `SensorManager` (Mid-360 10 Hz Livox
raycast, D435i 30 Hz **two** 640×480 renders). **None of that is packed
into LowState.** GEAR does not render lidar/depth in `sim_step`. Extra
cost plus GLFW `viewer.sync` dropped the DDS loop off realtime. History
observations saw stale or bunched packets; the network output
`q = default_angles + action × g1_action_scale` with huge `action`.

Also vs GEAR’s plant: we spawned joints at **zero** (straight) with **no**
hinge `damping`/`armature`/`frictionloss`. Deploy INIT interpolates to
standing; a slow sim finishes INIT before the body has actually squatted.

The log line `Reset init reference data root rotation to current frame:
0.71, …, -0.70` is the **motion clip** quaternion, not a bad IMU. Heading
from our LowState at init was ~identity (`0.999, …`).

## What we changed (DDS path only)

Inspect (`examples/01_empty_arena.py`) still uses the torso cable. SONIC
does not.

| Change | Why |
|---|---|
| Gantry `mode: gear` on **pelvis** | Same 6-D spring as GEAR `ElasticBand.Advance` |
| Never set `timestep=0.005` | Keep 1 ms MJCF |
| 200 Hz control, `physics_hz=1000` (5 substeps) | Same **wall** rate as GEAR without their coarse step |
| `loop.control_hz` synced to controller | Wall tick = 5 ms sim |
| Mid-360 / D435i **off** unless `--sensors` | `--sensors` is leftover-budget, not inlined in `step_physics` |
| Absolute sleep to `t0 + n × dt`; skip sensors and `viewer.sync` when late | RTF returns to 1 after a scan that overruns the 5 ms slot |
| Inspect-style overlays on `launch_passive` | GLFW can show clouds without putting them in LowState |
| Copy GEAR hinge dissipation at runtime | `dof_damping=0.05`, `armature=0.01`, `frictionloss=0.2` for `dof ≥ 6` |
| Spawn 29 joints at deploy `default_angles` | Standing squat before INIT |
| Clip `q_des` to `jnt_range` | Wild policy cannot command through joint stops |
| Reset on NaN / `z < 0.2` | Clear `xfrc`, `mj_resetData`, `mj_forward`; do not tight-loop on `Time=0.625` |

Startup line to trust:

```text
physics: dt=0.001  physics_hz=1000  control_hz=200  substeps=5
         loop_hz=200  gantry=gear@pelvis  cameras=off  sensors=off
```

With `--sensors` the last fields are `cameras=on` and
`sensors=budgeted leftover (slot 0.65 … not in step_physics)`.

Every 5 s: `~200 Hz` and **`sim/wall≈1.00`**. If `sim/wall` is ~0.5, deploy
will again see 30–70 ms LowState age.

`--gear-parity` now means “force 200 Hz / 5×1 ms” (already the YAML
default). It does **not** widen MuJoCo’s timestep.

## `--sensors`: leftover budget, not inlined in `step_physics`

`02_sonic_dds_bridge.py` now paints inspect-style overlays on
`launch_passive` (`extras/sonic_dds/overlay.py`): green Mid-360, cyan
depth, orange FOV, depth PiP. Lidar/depth are **not** inside
`step_physics`. One product per leftover slice of the 5 ms slot; skip
when the absolute wall deadline has already passed (same rule for
`viewer.sync`).

Before that split, `--sensors` called `SensorManager.step()` on every
control tick and the GLFW window never drew the clouds. Measured stall
(2026-09-07): **~110 Hz**, **`sim/wall≈0.55`**.

```text
# headless
cameras=on  sensors=budgeted leftover …
~200 Hz  sim/wall=1.00  lidar=9.0 Hz depth=26–27 Hz skip=133–159

# GLFW on (sync skipped when the slot is already late)
~199–201 Hz  sim/wall=1.00  lidar≈1 Hz depth≈1 Hz skip≈950
```

GLFW `sync` of overlay geoms is expensive; if it runs while the motion
slot is already late, LowState falls to ~6 Hz.

## How to use the sensorized suite *with* SONIC

Constraint: the ONNX policy needs **fresh LowState** (~20 ms). Lidar and
depth are slower products (10 Hz / 30 Hz) and must not steal that budget.

**Stopgap on this loop:** `--sensors` does **not** call `SensorManager.step`
inside `step_physics`. Lidar and depth run one product per leftover fraction
of the 5 ms slot (`loop.sensor_budget`) and skip when the tick already
overran. Overlays paint the last cloud. Target architecture (separate
machines) is still [`sim-process-model.md`](sim-process-model.md).

Until that split exists, treat leftover-budget as debug: if `sim/wall`
drops below ~1, turn `--sensors` off for policy work.

RoboCasa: `--scene` on the same **motion** loop. Gantry hold XY follows
spawn so the spring does not pull to the world origin.

## What we did *not* do

- Drop the sensorized G1 XML into GEAR `ROBOT_SCENE`. Extra mount bodies
  would load; Python `SensorManager` would not; 5 ms on our tree still
  NaNs (measured with GEAR’s spring).
- Clone GEAR-SONIC into the sim image. `:sonic` only vendors
  `unitree_sdk2py` from the local GEAR tree.
- Change inspect’s cable crane or the 500 Hz SDK2 default in
  `configs/default.yaml`.

## Checklist

- [ ] Both containers host network, domain 0, `lo`; LowState flowing.
- [ ] Startup `dt=0.001`, `gantry=gear@pelvis`, `loop_hz=200`.
- [ ] Without `--sensors`: `sim/wall≈1.00`, `~200 Hz`, policy `max_abs_q` modest.
- [ ] With `--sensors`: banner `budgeted leftover`; LowState still `~200 Hz` /
      `sim/wall≈1.00`; overlays if GLFW (clouds may be ~1 Hz). Headless lidar
      ~9 Hz / depth ~26 Hz is the leftover ceiling on this machine.
- [ ] INIT squat visible before `T` / `O`.
