# g1-simulacrum GPU container

Image and compose files live in this directory. **Usage:**
[`../docs/docker_usage.md`](../docs/docker_usage.md).

```bash
./run.sh up --build   # once, or after Dockerfile / extras change
./run.sh              # shell
./run.sh python examples/01_empty_arena.py
./run.sh sonic python examples/02_sonic_dds_bridge.py   # host network + unitree_sdk2py
```

SONIC extra: [`../docs/sonic_dds.md`](../docs/sonic_dds.md).
