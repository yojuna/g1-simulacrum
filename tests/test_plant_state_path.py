"""Tests for sonic_dds plant state path resolution."""

from __future__ import annotations

from pathlib import Path

from extras.sonic_dds.plant_state import resolve_plant_state_path


def test_resolve_prefers_workspace_when_redux_not_mounted(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.chdir(tmp_path)
    ws_logs = tmp_path / "logs"
    ws_logs.mkdir()
    redux = tmp_path / "docker" / "ws_sonic_redux"
    redux.mkdir(parents=True)
    # Empty mountpoint — no configs/sim.yaml

    monkeypatch.setattr(
        "extras.sonic_dds.plant_state.REDUX_MARKER",
        redux / "configs" / "sim.yaml",
    )
    monkeypatch.setattr(
        "extras.sonic_dds.plant_state.WORKSPACE_PLANT_STATE",
        ws_logs / ".plant_state.json",
    )
    monkeypatch.setattr(
        "extras.sonic_dds.plant_state.REDUX_PLANT_STATE",
        redux / "logs" / ".plant_state.json",
    )

    assert resolve_plant_state_path() == ws_logs / ".plant_state.json"


def test_resolve_prefers_redux_when_mounted(monkeypatch, tmp_path: Path) -> None:
    monkeypatch.chdir(tmp_path)
    ws_logs = tmp_path / "logs"
    ws_logs.mkdir()
    redux = tmp_path / "docker" / "ws_sonic_redux"
    (redux / "configs").mkdir(parents=True)
    marker = redux / "configs" / "sim.yaml"
    marker.write_text("backend: g1_simulacrum\n")

    monkeypatch.setattr("extras.sonic_dds.plant_state.REDUX_MARKER", marker)
    monkeypatch.setattr(
        "extras.sonic_dds.plant_state.WORKSPACE_PLANT_STATE",
        ws_logs / ".plant_state.json",
    )
    redux_state = redux / "logs" / ".plant_state.json"
    monkeypatch.setattr("extras.sonic_dds.plant_state.REDUX_PLANT_STATE", redux_state)

    assert resolve_plant_state_path() == redux_state


def test_resolve_env_override(monkeypatch, tmp_path: Path) -> None:
    custom = tmp_path / "custom.json"
    monkeypatch.setenv("PLANT_STATE_PATH", str(custom))
    assert resolve_plant_state_path() == custom
