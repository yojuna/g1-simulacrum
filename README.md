# g1-simulacrum

Sensorized Unitree G1 simulation in MuJoCo: 29-DoF EDU body, Dex3-1 hands by default (controller later), Livox Mid-360, RealSense D435i, pelvis and torso IMUs, 500 Hz PD or torque loop.

This is an early prototype. The public import is `g1_simulacrum`; the distribution name is `g1-simulacrum`.

- Design decisions: [`ARCHITECTURE.md`](ARCHITECTURE.md) (normative).
- Compiled hardware facts and citations: [`wiki/`](wiki/README.md).
- Runtime is Docker, not a host venv: [`docs/docker_usage.md`](docs/docker_usage.md).

```bash
cd docker
./run.sh up --build
./run.sh python -c "import g1_simulacrum; print(g1_simulacrum.__version__)"
```

```python
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig

sim = G1Simulacrum.from_config("configs/default.yaml")
```
