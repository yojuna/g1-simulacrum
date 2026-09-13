"""Bringup YAML keeps harness on until post-cmd settle (all scenes)."""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from extras.sonic_dds.wbc_config import GantryBringupConfig, SonicDdsConfig  # noqa: E402

def _redux_backend_dir() -> Path:
    here = Path(__file__).resolve()
    docker = Path("/workspace/ws_sonic_redux/configs/backends/g1_simulacrum")
    if docker.is_dir():
        return docker
    try:
        host = here.parents[4] / "ws_sonic_redux" / "configs" / "backends" / "g1_simulacrum"
    except IndexError:
        host = here.parents[1] / "ws_sonic_redux" / "configs" / "backends" / "g1_simulacrum"
    return host


REDUX = _redux_backend_dir()


class TestGantryBringupTiming(unittest.TestCase):
    def test_release_harness_on_lowcmd_default_off(self):
        cfg = GantryBringupConfig()
        self.assertFalse(cfg.release_harness_on_lowcmd)

    def test_bringup_yaml_keeps_harness_until_release(self):
        path = REDUX / "sonic_dds_bringup.yaml"
        if not path.is_file():
            self.skipTest("sonic_dds_bringup.yaml not in repo")
        cfg = SonicDdsConfig.from_yaml(path)
        self.assertFalse(cfg.gantry.bringup.release_harness_on_lowcmd)
        self.assertEqual(cfg.gantry.bringup.post_cmd_settle_s, 5.0)


if __name__ == "__main__":
    unittest.main()
