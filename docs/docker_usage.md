# Docker usage

GPU container for this repository. Files: [`../docker/`](../docker/). Always
use [`../docker/run.sh`](../docker/run.sh) — never `docker compose run` (it
fights the fixed name `g1-simulacrum`).

## Image

`g1-simulacrum:local` — CUDA 12.6, Python 3.10, MuJoCo, mujoco-lidar
(CPU default; Warp extra layer), pytest/ruff/mypy, and a clone
of `mujoco_menagerie` at `/opt/mujoco_menagerie`. The venv is
**`/opt/venv` inside the image**. Startup does not run pip. Core YAML still
uses `sensors.mid360.backend: cpu`.

MuJoCo viewers: **EGL** offscreen on the NVIDIA GPU, **GLFW** on the host X
display (Intel). Do not force `__GLX_VENDOR_LIBRARY_NAME=nvidia`.

The named container does **not** start on boot. `./run.sh` starts it; leaving
the session (or `./run.sh stop`) sends SIGTERM. The git root is bind-mounted;
there are no Docker volumes. Host files survive stop and reboot.

| Host (this repo) | In container | Role |
|------------------|--------------|------|
| this `g1_simulacrum/` tree | `/workspace` | compose bind `..:/workspace` from `docker/` |
| `docker/` | `/workspace/docker` | Dockerfile, `run.sh`, compose |
| `docker/home/` | `/workspace/docker/home` | `$HOME` |
| (not a host path) | `/opt/venv` | image Python |
| (not a host path) | `/opt/mujoco_menagerie` | Menagerie clone (`PYTHONPATH=/opt`). Robot MJCF is the package pin, not this tree |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | host X for GLFW |

Needs NVIDIA driver + nvidia-container-toolkit. Do not create a host `.venv`.

GEAR-SONIC is **not** in this image. Compose it later from
`GR00T-WholeBodyControl`, not as a sidecar here.

## Run

From `docker/`:

```bash
./run.sh up --build    # first time, or after Dockerfile changes
./run.sh               # start if needed, interactive bash, then graceful stop
./run.sh python …      # start if needed, run the command, then graceful stop
./run.sh up            # start and leave running until stop or host shutdown
./run.sh stop          # graceful SIGTERM
```

If the container was already up (`./run.sh up`), a later `./run.sh python …`
does **not** stop it — call `./run.sh stop` when finished.

`./run.sh up --build` **always** rebuilds, even when the container is already
running. Without `--build`, a running container is reused.

```bash
./run.sh python -c "import g1_simulacrum, mujoco; print(g1_simulacrum.__version__)"
./run.sh python -c "import mujoco_menagerie; print(mujoco_menagerie.__path__)"
./run.sh python examples/01_empty_arena.py
```

Pin MJCF (authoring only). Default is offline against vendored
`g1_simulacrum/model/mjcf/upstream/` + `assets/`. `--fetch` downloads **only**
the named XML, URDF, and STL files from GitHub raw (not the unitree_ros tarball):

```bash
./run.sh python scripts/pin_mjcf.py
./run.sh python scripts/pin_mjcf.py --fetch
```

**Don't** `docker compose down` unless you mean to delete the container
(image and host files stay). Don't `up --build` every session — only after
Dockerfile or `pyproject.toml` extras change.

To change Python packages, edit `pyproject.toml` (and the Dockerfile if the
install extras change) and `./run.sh up --build` once.
