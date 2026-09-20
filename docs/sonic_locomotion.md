# Programmatic locomotion (SONIC zmq_manager)

Drive the g1-simulacrum plant with velocity commands instead of keyboard teleop.
The locomotion client publishes packed `command` + `planner` messages on ZMQ port
**5556** for `gear_sonic_deploy --input-type zmq_manager`.

Code: `extras/sonic_locomotion/`.

## Architecture

```
┌─────────────────────────┐   DDS    ┌──────────────────────────┐
│ 02_sonic_dds_bridge.py  │ ◄──────► │ g1_deploy_onnx_ref       │
│ MuJoCo + rt/lowstate      │          │ --input-type zmq_manager │
└─────────────────────────┘          │ SUB tcp://127.0.0.1:5556   │
                                      └────────────▲─────────────┘
                                                   │ ZMQ PUB (packed)
                                      ┌────────────┴─────────────┐
                                      │ LocomotionClient         │
                                      │ extras/sonic_locomotion  │
                                      └──────────────────────────┘
```

This is the **sim-local** path. It does not require Nav2, ROS 2, or
`ws_nav_fastlio/cmd_vel_zmq`. For Nav2 later, use `cmd_vel_bridge.py` (see below).

## Quick start (Docker, three terminals)

All g1-simulacrum commands use the **`:sonic` image** (`./run.sh sonic …`) with
**host network** so CycloneDDS matches `gear_sonic_deploy`.

**Terminal 1 — plant** (from `ws_simulacra/src/g1_simulacrum/docker`)

```bash
./run.sh sonic up --build   # first time only
./run.sh sonic python examples/02_sonic_dds_bridge.py
```

**Terminal 2 — deploy** (GEAR Docker, also host network)

```bash
cd GR00T-WholeBodyControl/gear_sonic_deploy
./docker/run-ros2-dev.sh
bash deploy.sh sim --input-type zmq_manager --zmq-host 127.0.0.1
```

Wait for `Publishing rt/lowstate` in Terminal 1, then in Terminal 2:

1. Press **`]`** to start control
2. Press **`9`** to lower the gantry (or wait for harness settle)

**Terminal 3 — walk** (same `:sonic` container pattern)

```bash
cd ws_simulacra/src/g1_simulacrum/docker
./run.sh sonic python examples/03_programmatic_walk.py --vx 0.35 --seconds 8
```

**Unit tests** (no deploy required):

```bash
./run.sh sonic python -m pytest tests/test_sonic_locomotion.py -q
```

## Python API

```python
from extras.sonic_locomotion import LocomotionClient

client = LocomotionClient()  # binds tcp://*:5556
client.start()                 # command.start + 50 Hz planner heartbeat
client.walk_forward(0.4)       # body-frame forward m/s
client.set_velocity(0.3, 0.1, 0.05)  # vx, vy, wz
client.idle()
client.stop()
client.close()
```

Planner messages **omit** `vr_position` so the policy stays in g1 / joint encode
mode (locomotion only, no VR upper-body override).

## CLI

```bash
python extras/sonic_locomotion/cli.py walk --vx 0.3 --vy 0 --wz 0 --seconds 5
python extras/sonic_locomotion/cli.py idle --seconds 3
python extras/sonic_locomotion/cli.py pulse --speed 0.4 --on-s 2 --off-s 1 --cycles 3
```

## Nav2 / cmd_vel path (optional)

The incomplete `ws_nav_fastlio/src/cmd_vel_zmq` node only **exports** Nav2
`/cmd_vel` as JSON on port **5558**. It does not drive SONIC.

To connect Nav2 → SONIC:

```
Nav2 /cmd_vel → cmd_vel_zmq (PUB connect :5558)
                     ↓
              cmd_vel_bridge.py (SUB bind :5558)
                     ↓
              LocomotionClient (PUB bind :5556)
                     ↓
              deploy zmq_manager
```

```bash
# Terminal A: locomotion + cmd_vel bridge (one process)
python extras/sonic_locomotion/cmd_vel_bridge.py

# Terminal B: Nav2 stack with cmd_vel_zmq publishing to tcp://127.0.0.1:5558
```

Or use `behavior_controller.py --enable-twist 5558` from `ws_manipulation` /
`behavior_controller` (full mode FSM + web UI).

## Protocol reference

| Topic | Fields | Notes |
|-------|--------|-------|
| `command` | `start`, `stop`, `planner` | Edge-style lifecycle; heartbeat re-sends `start` every 0.5 s |
| `planner` | `mode`, `movement[3]`, `facing[3]`, `speed`, `height` | 50 Hz required; 1 s timeout → deploy holds |

Modes: `0=IDLE`, `1=SLOW_WALK`, `2=WALK`, `3=RUN` (auto-selected from speed unless overridden).

Twist convention (body frame, holonomic):

- `linear.x` — forward m/s
- `linear.y` — left m/s
- `angular.z` — yaw rad/s (integrated into `facing`)

## Troubleshooting

| Symptom | Check |
|---------|-------|
| Robot stands still | Deploy running with `--input-type zmq_manager`? Pressed `]`? Gantry down? |
| Planner timeout / hold | LocomotionClient heartbeat running? Port 5556 not double-bound |
| Jerky motion | Lower `--vx`; try `SLOW_WALK` with `--mode slow` |
| `Address already in use` | Stop ws_sonic_redux teleop heartbeat or other PUB on :5556 |
