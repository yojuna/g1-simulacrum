# g1-simulacrum

Sensorized Unitree G1 simulation in MuJoCo: 29-DoF model, Livox Mid-360, RealSense D435i, and IMU.

This is an early prototype. The public import is `g1_simulacrum`; the distribution name is `g1-simulacrum`. Architecture notes live in [`ARCHITECTURE.md`](ARCHITECTURE.md) and will be rewritten before the implementation is treated as stable.

Runtime is a Docker image (not a host venv). See [`docs/docker_usage.md`](docs/docker_usage.md).

```bash
cd docker
./run.sh up --build
./run.sh python -c "import g1_simulacrum; print(g1_simulacrum.__version__)"
```

```python
from g1_simulacrum import G1Simulacrum, G1SimulacrumConfig

sim = G1Simulacrum.from_config("configs/default.yaml")
```
