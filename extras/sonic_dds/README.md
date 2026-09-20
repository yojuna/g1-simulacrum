# SONIC DDS bridge (optional extra)

Compose **g1-simulacrum** with **GEAR-SONIC deploy** over Unitree SDK2 DDS.
`SonicDdsSimLoop` is one process: MuJoCo + GLFW viewer + DDS, like
`gear_sonic/scripts/run_sim_loop.py`. Not imported by core.

Runtime how-to: [`docs/sonic_dds.md`](../../docs/sonic_dds.md).
Why the first plant looked like GEAR and was not:
[`wiki/sonic-integration.md`](../../wiki/sonic-integration.md).
Target MCU vs lidar split (still later):
[`wiki/sim-process-model.md`](../../wiki/sim-process-model.md).

## Two-terminal workflow

```bash
# Terminal 1 — sim (Docker :sonic, host network)
cd docker
./run.sh sonic up --build    # once: copy unitree_sdk2py from local GEAR tree
./run.sh sonic python examples/02_sonic_dds_bridge.py
# optional: --sensors  dedicated-thread Mid-360/D435i + overlays

# Terminal 2 — deploy (GEAR Docker, also host network)
cd GR00T-WholeBodyControl/gear_sonic_deploy
./docker/run-ros2-dev.sh
# then:
bash deploy.sh sim
```

## Loop contract

- **200 Hz** wall (`configs/sonic_dds.yaml`), **5 × 1 ms** `mj_step`.
  Never widen `model.opt.timestep` to 5 ms on the sensorized MJCF.
- Gantry: `ElasticBand.gear_hold` on **pelvis** (GEAR cartesian PD).
- `step_physics(sensors=False)` every tick. LowState does not contain
  lidar or depth.
- `--sensors`: `SensorWorkerGroup` runs Mid-360 (~10 Hz) and D435i (~30 Hz)
  on dedicated threads with private `mjData`. Motion publishes pose
  snapshots; overlays read the latest cloud/depth (`overlay.py`).
- Sleep to `t0 + n × dt` so RTF can return to 1 after an overrun.

## Module map

| Module | Role |
|--------|------|
| `unitree_sdk2py_bridge.py` | Vendored GEAR `UnitreeSdk2Bridge` |
| `channel.py` | `init_channel()` for CycloneDDS (stock SDK API) |
| `observation_adapter.py` | `MjData` → dict for `PublishLowState` |
| `lowcmd_actuation.py` | `LowCmd` / Dex3 → named actuator torques; clip `q_des` to `jnt_range` |
| `sim_loop.py` | `SonicDdsSimLoop` (physics + sensor worker group + viewer + DDS) |
| `sensor_worker_group.py` | Lidar/depth thread orchestration + overlay poll |
| `lidar_worker.py` / `depth_worker.py` | Dedicated scan threads |
| `pose_snapshot.py` | Keep-last-1 qpos/qvel for workers |
| `overlay.py` | Sparse Mid-360 / depth geoms + PiP for `launch_passive` |
| `wbc_config.py` | YAML → GEAR-style bridge config dict |

## Image

`docker/Dockerfile.sonic` copies `$GROOT_ROOT/external_dependencies/unitree_sdk2_python`
from this machine and `uv pip install`s it. It does not install GEAR-SONIC.

## Upstream sync

See [`SOURCE.md`](SOURCE.md) when updating from GEAR.
