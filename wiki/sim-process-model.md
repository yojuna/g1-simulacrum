# Sim process model (hardware-like clocks)

How the **real G1** splits motion from sensors, why SONIC broke when we
did not, and how this package should copy that split in MuJoCo.

Related: [Control](g1-control.md) (500 Hz LowState), [Sensors](g1-sensors.md)
(Mid-360 / D435i), [SONIC integration](sonic-integration.md) (measured
sim2sim failures), [Sim fidelity](sim-fidelity.md) (what physics can match).

## Decision in one line

**Motion is a realtime machine. Sensors are other machines.** They share
pose over a keep-last-1 channel (DDS on the robot; a triple buffer in
one process is the same idea). They do **not** share one `mjData` and
they do **not** steal the 2 ms LowState slot.

## Why this page exists

On 2026-09-07, GEAR SONIC + `02_sonic_dds_bridge.py` with cameras off
held **~200 Hz** LowState and **`sim/wall ≈ 1.0`**. The same binary with
`--sensors` dropped to **~110 Hz** and **`sim/wall ≈ 0.53–0.55`**.
Lidar + depth ran **inside** `G1Simulacrum.step_physics` on the motion
thread. GLFW `launch_passive` has no inspect overlays, so it looked like
sensors were off. They were on and costing ~0.45 of wall time.

SONIC’s policy measures **wall-clock** age of `rt/lowstate` (`max_delay_t`
≈ 20 ms in GEAR `g1.py`). A 110 Hz plant ages that sample to 9–75 ms.
The policy then tracks a late pose and `max_abs_q` blows up. That is
the same failure mode as a stalled MCU, not a “need leftover budget”
problem.

The leftover-time idea (raycast if the 5 ms slot is not full) is a
**stopgap on the MCU**. Hardware does not do that. This page is the
target model. The SONIC extra currently ships that stopgap
(`--sensors`, absolute wall deadline, skip when late). Measured
(2026-09-07): LowState stays **~200 Hz** / **`sim/wall≈1.00`**; lidar
drops (headless ~9 Hz, GLFW ~1 Hz). That is not phase 2.

## What the real G1 actually is

Not one process. Three (or more) computers on one DDS domain.

| Machine | Job | Clock | DDS (examples) |
|---------|-----|-------|----------------|
| Motion MCU (`unitree_hg`) | Integrate joints, write LowState, apply LowCmd | **500 Hz**, 2 ms | `rt/lowstate`, `rt/lowcmd` |
| Lidar service | Livox Mid-360 driver | Cloud **10 Hz**, IMU **200 Hz** | `rt/utlidar/cloud`, `cloud_compact`, `cloud_livox_mid360`, `imu` |
| Perception / Jetson | RealSense D435i, high-level | Depth/RGB **~30 Hz** | Camera topics; **not** packed into LowState |

Sources:

- Motion rate: [g1-control.md](g1-control.md), Unitree
  [`g1_ankle_swing_example.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)
  (`dt = 0.002`, comment “500Hz”).
- Lidar topics and 10 Hz / 200 Hz: [Unitree G1 lidar service](https://support.unitree.com/home/en/G1_developer/lidar_service)
  (`Lidar Driver >= 1.0.0.5` for `cloud_livox_mid360`;
  `InitChannel(..., 1)` — **one** sample queued, latest wins).
- LowState contents: motor + IMUs + battery + wireless + sport-mode
  bits. **No** point cloud. See [g1-sensors.md](g1-sensors.md) and
  `unitree_hg` IDL.

If lidar stalls, walking continues. If LowState stalls, everything dies.
That is the invariant the sim must keep.

## What GEAR deploy already copied (policy side)

`g1_deploy_onnx_ref` is **not** one 500 Hz blob either. Comments in
`g1.cpp` / `g1.h`:

| Thread | Rate | Role |
|--------|------|------|
| Input | 100 Hz | Read latest LowState (and cameras if used) |
| Control | 50 Hz | ONNX / WBC |
| Planner | 10 Hz | High-level |
| Command writer | 500 Hz | `rt/lowcmd` |

DDS callbacks write a **triple buffer** (`DataBuffer`: `Buffer0/1/2`).
Consumers take the **latest** sample. There is no 1 ms history ring and
no “wait for lidar before the next LowCmd”.

That is the hardware pattern on the **policy** side. The plant should
mirror it on the **sim** side.

## Two clocks (do not mix them)

| Clock | What it is | Who cares |
|-------|------------|-----------|
| **Sim time** | `data.time`, incremented by `mj_step` | Physics, sensor stamps *inside the world* |
| **Wall time** | `time.perf_counter()` on the host | DDS age, GLFW, the human, SONIC `max_delay_t` |

**RTF** = sim_time / wall_time (Isaac Lab / Isaac Sim definition:
[Real-Time Factor](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/gain_tuner.html)).
RTF = 1 means 1 s of physics in 1 s of wall. RTF = 0.5 means the plant
is half speed and every “20 ms” LowState is 40 ms old to the policy.

Rules:

1. Motion DDS (`rt/lowstate` / `rt/lowcmd`) must run at **wall-clock**
   200 Hz (SONIC) or 500 Hz (SDK2 default) with **RTF ≈ 1**. Otherwise
   the policy’s delay check is lying.
2. Sensor products may drop. A 10 Hz cloud that becomes 8 Hz is fine.
   A 200 Hz LowState that becomes 110 Hz is not.
3. If this host **cannot** hold RTF = 1 with the chosen physics, stop.
   Do not “catch up” by skipping sleeps while still publishing
   wall-clock DDS — that is the 2026-09-07 failure. Alternatives:
   fewer rays, GPU lidar later, or a sim-time clock (`/clock`) that
   **both** plant and policy honor. GEAR deploy does **not** use
   `/clock` today.

`mj_step` is not a wall-clock timer. Sleep (or a sim-time barrier) is
what makes RTF = 1. GEAR `run_sim_loop.py` sleeps on leftover wall time
in the control slot for this reason.

## MuJoCo: one `mjData` per thread

Official constraint ([MuJoCo programming](https://mujoco.readthedocs.io/en/stable/programming.html)):
one `mjData` per thread. `mjModel` is const and shareable.

`mj_step` and a lidar raycast both **read and write** the same `mjData`
(contact lists, ray scratch, sensor slots). They must not run at the
same time on one handle.

Copy is the documented escape hatch:

- [`mj_copyData`](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-copydata)
  — deep copy of dynamics state into another `mjData`.
- Viewer `launch_passive`: the UI thread owns a copy; `sync()` copies
  from the physics thread ([passive viewer](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer)).
- Maintainers, [mujoco#1402](https://github.com/google-deepmind/mujoco/issues/1402):
  copy `mjData`, then `mj_forward` / sensors on the copy. Same pattern
  as `simulate`.

Python `threading` does **not** make CPU lidar cheaper (GIL). It only
helps if the worker releases the GIL (C++ / Warp) **and** uses a
**private** `mjData`. Two Python threads on `sim.data` is a data race,
not parallelism.

## What not to do

| Idea | Why it fails |
|------|----------------|
| Raycast in leftover motion-thread budget | Still the MCU doing perception. Hardware does not. A slow cloud still stretches the 2 ms slot when the slot is already tight. |
| Share `sim.data` across threads | Illegal in MuJoCo. |
| Deep ring of 1 ms full physics states | Real lidar does not rewind a kinematics tape. Latest pose is enough. Copies of full `mjData` at 200–1000 Hz will RAM-choke. |
| `queue.Queue` of pickled `mjData` | Latency + copies + GIL. Use a **single slot** overwrite (triple buffer). |
| One process “because Docker is simpler” *and* calling `SensorManager.step` inside `step_physics` | That is what made SONIC `sim/wall ≈ 0.55`. |

## Target topology

```
┌─────────────────────────────────────────────────────────┐
│  Motion plant  (this is the MCU)                        │
│  mj_step · gantry · LowCmd → PD · publish LowState      │
│  sleep so RTF ≈ 1                                       │
│  NEVER SensorManager.step / lidar.scan / camera.render  │
└──────────────────────────┬──────────────────────────────┘
                           │ pose snapshot, keep-last-1
                           │ {qpos, qvel, time}  or DDS equivalent
           ┌───────────────┼────────────────┐
           ▼               ▼                ▼
    Lidar service    Camera service     Viewer
    copy mjData      copy mjData        last cloud overlay
    Mid-360 @ 10 Hz  D435i @ 30 Hz      no scan on motion thread
    drop if busy     drop if busy
    rt/utlidar/*     camera topics
```

**In-process** (phase 2): the arrows are a triple buffer of pose, not
DDS. Same contract: producer never waits; consumer copies `mjData` from
the latest pose and works on **its** handle.

**Multi-process** (phase 3): the arrows *are* DDS, like Jetson vs MCU.
Heavier, closest to the robot, useful when lidar should survive a
motion-container restart.

### Pose snapshot (keep-last-1)

Minimum payload the sensor machine needs to match the world:

- `qpos`, `qvel` (and `mocap_*` / `ctrl` if used)
- `data.time` (sim stamp on the cloud)
- optionally `userdata` / xfrc if anything else moves the robot

**Not** required: full `mjData` history, contact cache, 200 copies/s.

Apply snapshot → `mj_forward` on the **worker** `mjData` → raycast /
render. If the previous scan is still running, **skip** this period
(latest-wins). Same as `InitChannel(..., 1)` on the lidar service.

GEAR already implements this for LowState (`DataBuffer`). Same shape:
[`ioai-tech/robot_hardware_interface`](https://github.com/ioai-tech/robot_hardware_interface)
(`DataBuffer`, lock, three slots).

## Phased plan (this package)

Do not jump to three containers. Prove the invariant first.

### Phase 1 — motion never calls sensors

`G1Simulacrum.step_physics(..., sensors=False)` on the SONIC (and any
realtime DDS) path. **Done** in `SonicDdsSimLoop.sim_step`. Default inspect
example still steps sensors on the main thread; that path is not SONIC.

Pass: cameras-off LowState **~200 Hz**, **`sim/wall≈1.00`**.

### Phase 1.5 — leftover-budget stopgap (current `--sensors`)

Still the motion thread. After `mj_step`, at most one of lidar or depth
if the absolute slot deadline is in the future and elapsed motion time
is under `loop.sensor_budget` (0.65). Same skip for `viewer.sync`.
Overlays paint the last cloud (`extras/sonic_dds/overlay.py`).

Pass (measured 2026-09-07): LowState **~200 Hz**, **`sim/wall≈1.00`**
with `--sensors`. Clouds may be 9 Hz headless or ~1 Hz with GLFW.
Forbidden: LowState ~110 Hz from inlined `SensorManager.step`, or ~6 Hz
from `viewer.sync` while already late.

This is **not** a lidar machine. Do not treat leftover-budget as the
architecture.

### Phase 2 — same process, two machines

- Motion thread: physics + LowState + sleep.
- Sensor thread (or subprocess later): wait on 10 Hz / 30 Hz timers,
  copy latest pose, `mj_copyData` or set `qpos`/`qvel` + `mj_forward`,
  scan/render, store last cloud for overlay, optional `rt/utlidar`
  publish.
- Viewer: paint last overlay only. `launch_passive` still must not
  `SensorManager.step`.

Pass: cameras on, overlays visible, **`rt/lowstate` still ~200 Hz wall**,
**`sim/wall ≈ 1.0`**. Cloud rate may be 8 Hz. Log `lidar skip=`.

### Phase 3 — split processes (optional, hardware-true)

Motion container and sensor container, host network, CycloneDDS domain 0.
Pose via DDS or a shared-memory triple buffer. Same drop policy.

## What we will not copy from Isaac

Isaac can run the **whole stack** on sim time (`/clock`, use_sim_time).
That is valid when plant and policy agree.

SONIC deploy today is a **real-time** consumer: wall-clock DDS age.
Until GEAR reads `/clock`, this plant must be wall-clock realtime for
motion topics. Sensors can be “best effort in sim time” only after
motion RTF is 1.

## Mapping to code (current vs target)

| Current (2026-09-07 leftover-budget) | Target |
|--------------------------------------|--------|
| `step_physics(sensors=False)` on the DDS path | Same |
| `--sensors`: one lidar **or** depth per leftover slice; skip if past absolute deadline | Dedicated lidar/camera **machine**, private `mjData` |
| `viewer.sync` skipped when the slot is late; last cloud overlay | Viewer paints last cloud only; never `SensorManager.step` |
| No `rt/utlidar/*` | Phase 2+ publish, keep-last-1 |
| Inspect (`01_empty_arena.py`) still steps sensors on the main thread | Unchanged until inspect is ported |
| `loop.sensor_budget` leftover raycast | Debug fallback only; not the design |

`G1Simulacrum.step_physics` takes `sensors: bool`. SensorManager also
exposes `lidar_due` / `step_lidar` / `depth_due` / `step_depth` for the
leftover path (and later a worker).

## Pass / fail (copy onto a PR)

With `--gear-parity --sensors` and GEAR `deploy.sh sim` running:

1. Banner: `cameras=on` and `sensors=budgeted leftover … not in step_physics`.
2. Every 5 s: LowState **~200 Hz** wall, **`sim/wall ≈ 1.00`**
   (same as cameras off, measured 2026-09-07).
3. Overlays: green lidar / cyan depth (GLFW). Headless needs no overlays.
4. Allowed: `lidar 9/10 Hz skip=N` headless, or ~1 Hz with GLFW `sync`.
   Forbidden: LowState 110 Hz or 6 Hz.

If (2) fails, the architecture is wrong — do not tune policy gains.

## Sources (extra)

| What | Where |
|------|--------|
| MuJoCo one `mjData` / thread | [Programming](https://mujoco.readthedocs.io/en/stable/programming.html) |
| `mj_copyData` | [API](https://mujoco.readthedocs.io/en/stable/APIreference/APIfunctions.html#mj-copydata) |
| Passive viewer copy + `sync` | [Python](https://mujoco.readthedocs.io/en/stable/python.html#passive-viewer) |
| Copy-then-forward | [mujoco#1402](https://github.com/google-deepmind/mujoco/issues/1402) |
| RTF = sim / wall | [Isaac Gain Tuner](https://docs.isaacsim.omniverse.nvidia.com/5.1.0/robot_setup/gain_tuner.html) |
| Lidar service 10 / 200 Hz, queue length 1 | [Unitree lidar_service](https://support.unitree.com/home/en/G1_developer/lidar_service) |
| Triple buffer on robot stacks | GEAR `DataBuffer`; [ioai DataBuffer](https://github.com/ioai-tech/robot_hardware_interface) |
| Measured 200 Hz vs 110 Hz | [sonic-integration.md](sonic-integration.md) (2026-09-07 logs) |
