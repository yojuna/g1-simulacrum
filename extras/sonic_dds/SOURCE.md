# Vendored GEAR-SONIC sources

Copied from `GR00T-WholeBodyControl` at commit `3e319b5ad983d117affd17c8ec5a5dbe22ef2f76`.

| File | Upstream |
|------|----------|
| `unitree_sdk2py_bridge.py` | `gear_sonic/utils/mujoco_sim/unitree_sdk2py_bridge.py` |
| `channel.py` (`init_channel`) | `gear_sonic/utils/mujoco_sim/simulator_factory.py` (stock SDK; no Tailscale `peer_address`) |
| `observation_adapter.py` | logic from `gear_sonic/utils/mujoco_sim/base_sim.py` `prepare_obs()` |
| `lowcmd_actuation.py` | logic from `gear_sonic/utils/mujoco_sim/base_sim.py` `compute_body_torques()` / `compute_hand_torques()` |
| `wbc_config.py` `motor_effort_limit_list` | `gear_sonic/utils/mujoco_sim/wbc_configs/g1_29dof_sonic_model12.yaml` |

`unitree_sdk2py` is **not** vendored here. `docker/Dockerfile.sonic` copies it
from `$GROOT_ROOT/external_dependencies/unitree_sdk2_python` at image build.

Re-sync the Python adapters when upstream bridge or observation layout changes.
