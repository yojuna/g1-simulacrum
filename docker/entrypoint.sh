#!/usr/bin/env bash
# Thin wrapper: PATH + MuJoCo/X11 env. Packages live in the image at /opt/venv.
set -euo pipefail

export VIRTUAL_ENV="${VIRTUAL_ENV:-/opt/venv}"
export PATH="${VIRTUAL_ENV}/bin:${PATH}"
export PYTHONPATH="${PYTHONPATH:-/workspace:/opt}"
export MUJOCO_GL="${MUJOCO_GL:-egl}"
export PYOPENGL_PLATFORM="${PYOPENGL_PLATFORM:-egl}"
export HOME="${HOME:-/workspace/docker/home}"
# Headless only when no X11. Dummy SDL breaks the MuJoCo GLFW viewer.
if [[ -z "${DISPLAY:-}" ]]; then
  export SDL_VIDEODRIVER="${SDL_VIDEODRIVER:-dummy}"
fi

mkdir -p "$HOME" 2>/dev/null || true
cd /workspace
exec "$@"
