# g1-simulacrum

Sensorized Unitree G1 in MuJoCo: 29-DoF EDU body, Dex3-1 hands by default
(controller later), Livox Mid-360, RealSense D435i, pelvis and torso IMUs,
500 Hz PD or torque loop.

The public import is `g1_simulacrum`; the distribution name is `g1-simulacrum`.

- Design decisions: [`ARCHITECTURE.md`](ARCHITECTURE.md) (normative).
- Compiled hardware facts and citations: [`wiki/`](wiki/README.md).
- Runtime is Docker, not a host venv: [`docs/docker_usage.md`](docs/docker_usage.md).

```bash
cd docker
./run.sh up --build
./run.sh python -c "import g1_simulacrum; print(g1_simulacrum.__version__)"
./run.sh python -m pytest tests -q
```

```python
from g1_simulacrum import G1Simulacrum

sim = G1Simulacrum.from_config("configs/default.yaml")
sim.build()
obs = sim.reset()
obs = sim.step(obs.joint_state.position)  # (29,) body targets; None holds last
```

## Inspect viewer

![G1 inspect viewer with Mid-360 (green), D435i depth (cyan), and camera frustum](docs/inspect_viewer.png)

GLFW on the host X display. From `docker/`:

```bash
./run.sh python examples/01_empty_arena.py
```

Green overlays are Mid-360, cyan is D435i depth. Default scene is
`g1_inspect.xml` (floor + boxes). `--empty` is floor only.

| Key / flag | Effect |
|------------|--------|
| **C** | Cycle free camera → `d435i_rgb` → `d435i_depth` |
| **7 / 8 / 9** | Gantry length down / up / toggle |
| `--no-gantry` | Drop the elastic band (robot falls; PD is joints only) |
| `--overlay sparse\|dense\|full` | Overlay density (default dense) |
| `--headless` | 50 control steps, no window |

The gantry is a spring-damper wrench on `pelvis`, not a weld. Overlay dots
are cheap boxes, refreshed at sensor rate.
