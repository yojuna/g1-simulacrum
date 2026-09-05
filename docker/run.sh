#!/usr/bin/env bash
# Dedicated GPU container — never `compose run` (that fights container_name).
#
#   ./run.sh              start if needed, bash, then graceful stop
#   ./run.sh <cmd...>     start if needed, run cmd, then graceful stop
#   ./run.sh exec <cmd>   same as above
#   ./run.sh up           start and leave running (until stop or host shutdown)
#   ./run.sh up --build   rebuild image then up
#   ./run.sh stop         SIGTERM + grace period (container object remains)
#   ./run.sh start        start an existing stopped container (leave running)
#
# No restart-on-boot. Bind-mounted /workspace keeps code and results.
set -euo pipefail
cd "$(dirname "$0")"

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export VIDEO_GID="$(getent group video | cut -d: -f3 || echo 44)"
export RENDER_GID="$(getent group render | cut -d: -f3 || echo 110)"

mkdir -p home

if [[ -n "${DISPLAY:-}" ]]; then
  xhost +SI:localuser:"$(id -un)" >/dev/null 2>&1 || xhost +local: >/dev/null 2>&1 || true
fi

is_running() {
  docker compose ps --status running --services 2>/dev/null | grep -qx sim
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
  docker compose logs --tail 80 sim >&2
  return 1
}

ensure_up() {
  local build_flag="${1:-}"
  if [[ "$build_flag" == "--build" ]]; then
    docker compose up -d --build
    wait_running
    return
  fi
  if is_running; then
    return 0
  fi
  docker compose up -d
  wait_running
}

# Start if needed, run the command, stop only if this invocation started it.
run_session() {
  local started_here=0
  if ! is_running; then
    started_here=1
    ensure_up
  fi
  local st=0
  docker compose exec sim "$@" || st=$?
  if [[ "$started_here" -eq 1 ]]; then
    docker compose stop
  fi
  return "$st"
}

if [[ "${1:-}" == "stop" ]]; then
  exec docker compose stop
fi

if [[ "${1:-}" == "start" ]]; then
  docker compose start
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
