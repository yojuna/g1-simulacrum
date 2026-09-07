#!/usr/bin/env bash
# Dedicated GPU container — never `docker compose run` (that fights container_name).
#
#   ./run.sh              start if needed, bash, then graceful stop
#   ./run.sh <cmd...>     start if needed, run cmd, then graceful stop
#   ./run.sh exec <cmd>   same as above
#   ./run.sh up           start and leave running (until stop or host shutdown)
#   ./run.sh up --build   rebuild image then up
#   ./run.sh stop         SIGTERM + grace period (container object remains)
#   ./run.sh start        start an existing stopped container (leave running)
#   ./run.sh sonic …      g1-simulacrum:sonic (unitree_sdk2py) + host network
#
# No restart-on-boot. Bind-mounted /workspace keeps code and results.
set -euo pipefail
cd "$(dirname "$0")"

COMPOSE_FILES=(-f compose.yaml)
SONIC_MODE=0
SONIC_IMAGE="g1-simulacrum:sonic"
BASE_IMAGE="g1-simulacrum:local"
if [[ "${1:-}" == "sonic" ]]; then
  shift
  SONIC_MODE=1
  COMPOSE_FILES+=(-f compose.sonic.yaml)
  export GROOT_ROOT="${GROOT_ROOT:-$(cd ../../../.. && pwd)/GR00T-WholeBodyControl}"
fi

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"
export RENDER_GID="$(getent group render | cut -d: -f3 || echo 110)"

mkdir -p home

if [[ -n "${DISPLAY:-}" ]]; then
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true
fi

is_running() {
  docker compose "${COMPOSE_FILES[@]}" ps --status running --services 2>/dev/null | grep -qx sim
}

wait_running() {
  local i
  for i in $(seq 1 60); do
    if is_running; then
      return 0
    fi
    sleep 1
  done
  echo "[g1-simulacrum] container did not start:" >&2
  docker compose "${COMPOSE_FILES[@]}" logs --tail 80 sim >&2
  return 1
}

container_image() {
  docker inspect -f '{{.Config.Image}}' g1-simulacrum 2>/dev/null || true
}

container_image_id() {
  docker inspect -f '{{.Image}}' g1-simulacrum 2>/dev/null || true
}

tagged_image_id() {
  docker image inspect -f '{{.Id}}' "$1" 2>/dev/null || true
}

image_exists() {
  docker image inspect "$1" >/dev/null 2>&1
}

ensure_up() {
  local build_flag="${1:-}"

  if [[ "$SONIC_MODE" == "1" ]]; then
    # :sonic is FROM g1-simulacrum:local. Never rebuild the base here —
    # that is ./run.sh up --build. Only build :local if the tag is missing.
    if ! image_exists "$BASE_IMAGE"; then
      echo "[g1-simulacrum] $BASE_IMAGE missing — building base first"
      docker compose -f compose.yaml build
    fi
    if [[ "$build_flag" == "--build" ]] || ! image_exists "$SONIC_IMAGE"; then
      local sdk="${GROOT_ROOT}/external_dependencies/unitree_sdk2_python"
      if [[ ! -f "${sdk}/setup.py" && ! -f "${sdk}/pyproject.toml" ]]; then
        echo "[g1-simulacrum] unitree_sdk2_python not found at ${sdk}" >&2
        echo "  This image only pip-installs that SDK from the GEAR tree on this machine." >&2
        echo "  export GROOT_ROOT=/path/to/GR00T-WholeBodyControl" >&2
        exit 1
      fi
      echo "[g1-simulacrum] building $SONIC_IMAGE from ${sdk}"
      docker compose "${COMPOSE_FILES[@]}" build
    fi
    # Tag name stays g1-simulacrum:sonic across rebuilds; compare IDs.
    if is_running && [[ "$(container_image_id)" != "$(tagged_image_id "$SONIC_IMAGE")" ]]; then
      echo "[g1-simulacrum] recreating container with new $SONIC_IMAGE + host network"
      docker compose -f compose.yaml stop
    fi
    if ! is_running; then
      docker compose "${COMPOSE_FILES[@]}" up -d
      wait_running
    fi
    return
  fi

  if [[ "$build_flag" == "--build" ]]; then
    docker compose "${COMPOSE_FILES[@]}" up -d --build
    wait_running
    return
  fi
  if is_running; then
    return 0
  fi
  docker compose "${COMPOSE_FILES[@]}" up -d
  wait_running
}

# Start if needed, run the command, stop only if this invocation started it.
run_session() {
  local started_here=0
  if ! is_running; then
    started_here=1
    ensure_up
  elif [[ "$SONIC_MODE" == "1" && "$(container_image_id)" != "$(tagged_image_id "$SONIC_IMAGE")" ]]; then
    started_here=1
    ensure_up
  fi
  local st=0
  docker compose "${COMPOSE_FILES[@]}" exec sim "$@" || st=$?
  if [[ "$started_here" -eq 1 ]]; then
    docker compose "${COMPOSE_FILES[@]}" stop
  fi
  return "$st"
}

if [[ "${1:-}" == "stop" ]]; then
  exec docker compose "${COMPOSE_FILES[@]}" stop
fi

if [[ "${1:-}" == "start" ]]; then
  docker compose "${COMPOSE_FILES[@]}" start
  wait_running
  echo "container g1-simulacrum is running (will not auto-start on reboot).  ./run.sh stop when done"
  exit 0
fi

if [[ "${1:-}" == "up" ]]; then
  shift
  ensure_up "${1:-}"
  echo "container g1-simulacrum is up (no restart-on-boot).  ./run.sh stop when done"
  exit 0
fi

if [[ "${1:-}" == "exec" ]]; then
  shift
  if [[ $# -eq 0 ]]; then
    set -- bash
  fi
  run_session "$@"
  exit $?
fi

if [[ $# -eq 0 ]]; then
  set -- bash
fi
run_session "$@"
exit $?
