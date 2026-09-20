"""Plant state file for ws_sonic_redux orchestration (optional).

When ws_sonic_redux is bind-mounted at ``docker/ws_sonic_redux``, health checks
read ``logs/.plant_state.json`` from that tree. Standalone ``./run.sh sonic``
uses ``/workspace/logs/.plant_state.json`` instead (writable bind mount).
"""

from __future__ import annotations

import json
import os
from pathlib import Path

REDUX_ROOT = Path("/workspace/docker/ws_sonic_redux")
REDUX_MARKER = REDUX_ROOT / "configs" / "sim.yaml"
REDUX_PLANT_STATE = REDUX_ROOT / "logs" / ".plant_state.json"
WORKSPACE_PLANT_STATE = Path("/workspace/logs/.plant_state.json")


def _dir_writable(path: Path) -> bool:
    try:
        path.mkdir(parents=True, exist_ok=True)
        probe = path / ".write_probe"
        probe.write_text("ok")
        probe.unlink()
        return True
    except OSError:
        return False


def resolve_plant_state_path() -> Path:
    """Pick a plant-state path that works with or without ws_sonic_redux mounted."""
    override = os.environ.get("PLANT_STATE_PATH", "").strip()
    if override:
        return Path(override)

    if REDUX_MARKER.is_file():
        candidates = (REDUX_PLANT_STATE, WORKSPACE_PLANT_STATE)
    else:
        # Empty docker/ws_sonic_redux mountpoint (nobody:nogroup) is common without sim_ctl.
        candidates = (WORKSPACE_PLANT_STATE, REDUX_PLANT_STATE)

    for path in candidates:
        if _dir_writable(path.parent):
            return path
    return candidates[0]


def write_plant_state(path: Path, payload: dict) -> bool:
    """Atomically write plant state; return False on permission / IO errors."""
    try:
        tmp = path.with_suffix(".tmp")
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp.write_text(json.dumps(payload))
        tmp.replace(path)
        return True
    except OSError:
        return False
