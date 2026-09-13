# SONIC DDS sim2sim

`examples/02_sonic_dds_bridge.py` is the GEAR `run_sim_loop.py` equivalent: **one
process** owns MuJoCo, the GLFW viewer, and Unitree DDS (`rt/lowstate` /
`rt/lowcmd`). There is no separate “bridge vs sim” split.

Deploy (`g1_deploy_onnx_ref`) talks DDS only. It does not start MuJoCo.

Code: `extras/sonic_dds/`. Config: `configs/sonic_dds.yaml`.
Integration log: [`wiki/sonic-integration.md`](../wiki/sonic-integration.md).
Target process split (not this loop): [`wiki/sim-process-model.md`](../wiki/sim-process-model.md).

## Prerequisites

1. Base image once: `./run.sh up --build`
2. Sonic image (layers `unitree_sdk2py` on `:local`): `./run.sh sonic up --build`
3. GEAR deploy image/binary as you already run it (`gear_sonic_deploy/docker/run-ros2-dev.sh`)

## Run

Both sides **must use host network** so CycloneDDS on `lo` is the same stack.

```bash
# Terminal 1 — sim + viewer + DDS
cd ws_simulacra/src/g1_simulacrum/docker
./run.sh sonic python examples/02_sonic_dds_bridge.py

# Terminal 2 — SONIC policy (GEAR Docker, also --network host)
cd GR00T-WholeBodyControl/gear_sonic_deploy
./docker/run-ros2-dev.sh
# inside that container:
bash deploy.sh sim
```

Wait until Terminal 1 prints `Publishing rt/lowstate` before relying on deploy.
Then press **`]`** in the deploy terminal.

Healthy motion-only startup:

```text
physics: dt=0.001  physics_hz=1000.0  control_hz=200.0  substeps=5
         loop_hz=200.0  gantry=gear@pelvis  cameras=off  sensors=off
```

Every 5 s: `~200 Hz` and `sim/wall=1.00`.

## Controls

| Where | Key | Action |
|-------|-----|--------|
| Deploy | `]` | Init / enter control |
| Deploy | `9` | Toggle overhead gantry |
| Deploy | `T` | Planner mode |
| Deploy | `O` | Operator mode |
| Viewer | `7` / `8` | Hold height down / up (GEAR `length`) |
| Viewer | `9` | Toggle crane (GEAR) |
| Viewer | numpad | Trolley / heading (same as inspect viewer) |

## Loop (what the plant actually does)

Default YAML is **200 Hz** control, **5 × 1 ms** `mj_step`, GEAR pelvis
6-D spring (`gantry.mode: gear`), Mid-360/D435i **off**. That is the path
the ONNX policy was measured on.

Each control tick:

1. Publish `rt/lowstate` (and optional wireless).
2. Apply gantry wrench + LowCmd PD into `data.ctrl`.
3. `G1Simulacrum.step_physics(sensors=False)` — physics only.
4. If `--sensors`: at most **one** of lidar or depth, and only if the
   tick’s **absolute wall deadline** is still in the future and motion
   used less than `loop.sensor_budget` (0.65) of the 5 ms slot.
5. Sleep until `t0 + n × 5 ms`. Missed deadlines skip sleep so RTF can
   catch up. `viewer.sync` uses the same skip (a late sync used to drop
   LowState to ~6 Hz).

`--sensors` is a **stopgap on the motion thread**, not a lidar machine.
Clouds may drop (`skip=` in the 5 s line). LowState must not.

Measured 2026-09-07 (this machine, `--gear-parity --sensors`):

| Mode | LowState | `sim/wall` | Lidar / depth |
|------|----------|------------|----------------|
| Headless | ~200 Hz | 1.00 | ~9 Hz / ~26 Hz |
| GLFW overlays | ~199–201 Hz | 1.00 | ~1 Hz (sync eats leftover) |
| Inlined `SensorManager.step` (old) | ~110 Hz | ~0.55 | n/a |

5 s log with sensors (enable `loop.stats_log_interval_s: 5` in sonic YAML):

```text
rt/lowstate published N times  ~200 Hz  sim/wall=1.00  lidar=9.0 Hz depth=26.2 Hz skip=133
```

By default periodic stats are **off** so bringup logs stay readable.

If `sim/wall` is not ~1, turn `--sensors` off for policy work.

## CLI

```bash
./run.sh sonic python examples/02_sonic_dds_bridge.py --help
```

| Flag | Purpose |
|------|---------|
| `--headless` | DDS on, no GLFW window |
| `--headless-smoke` | 2 s physics, no DDS |
| `--no-gantry` | Disable crane |
| `--gear-parity` | Force 200 Hz control, 5× 1 ms `mj_step` (already the YAML default). Does **not** set `timestep=0.005`. |
| `--sensors` | Leftover-budget Mid-360 + D435i + inspect overlays. Off by default. |
| `--scene` | RoboCasa or other MJCF that already includes `g1_robot.xml` |
| `--duration N` | Exit after N wall seconds |

## YAML (`configs/sonic_dds.yaml`)

| Key | Default | Meaning |
|-----|---------|---------|
| `loop.control_hz` | 200 | Wall LowState / LowCmd rate |
| `loop.cameras` | false | Construct Mid-360 + D435i (`--sensors` sets true) |
| `loop.sensor_budget` | 0.65 | Fraction of the 5 ms slot reserved for LowState + `mj_step` |
| `loop.viewer_dt` | 0.02 | Attempt GLFW `sync` this often (skipped if the slot is late) |
| `gantry.mode` | `gear` | Pelvis cartesian PD. `cable` is inspect’s torso hook |
| `gantry.attach_body` | `pelvis` | Force body for the gear spring |

## Image: copy `unitree_sdk2py` from the local GEAR tree

This Docker does **not** install GEAR-SONIC. At sonic-image build it copies
`GR00T-WholeBodyControl/external_dependencies/unitree_sdk2_python` from this
machine and `uv pip install`s it into `/opt/venv` (same as GEAR’s
`install_scripts/install_mujoco_sim.sh`). Deploy stays in
`gear_sonic_deploy/docker`.

`export GROOT_ROOT=...` if GR00T is not the sibling `robo_ops/GR00T-WholeBodyControl`.

## Troubleshooting

- **LowState is not available:** sim must be the **`:sonic` image** with
  `network_mode: host`. `./run.sh sonic …` recreates the container if it was
  started with `./run.sh up` (`:local`, bridge network). Deploy must also be
  host-network (`run-ros2-dev.sh`). Unset `CYCLONEDDS_URI` for local `lo`.
- **No viewer:** this example launches `mujoco.viewer.launch_passive` like GEAR.
  Use `--headless` only if you do not want a window. Needs `DISPLAY` + X11 mount
  (already in compose). `--sensors` adds green/cyan overlays and a depth PiP.
- **NaN / fallen at Time = 0.625:** old `--gear-parity` set `timestep=0.005` on
  the 1 ms sensorized MJCF. It now keeps 1 ms and takes 5 substeps at 200 Hz.
  Default SONIC gantry is GEAR's pelvis 6-D spring (`mode: gear`), not the
  inspect cable.
- **Wonky policy / LowState age 10–80 ms / `max_abs_q` of several rad:** lidar
  and D435i used to run inside every `step_physics`. They are off unless
  `--sensors`. Check `sim/wall≈1.00` (and `skip=`) in the 5 s LowState line.
  GLFW overlay `sync` must not run after the slot deadline (~6 Hz if it does).
