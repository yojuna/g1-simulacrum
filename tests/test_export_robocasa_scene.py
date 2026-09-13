"""Free-body pose bake for RoboCasa MJCF export (no robocasa import)."""

from __future__ import annotations

import importlib.util
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path
from types import SimpleNamespace

_SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "export_robocasa_scene.py"


def _load_export():
    spec = importlib.util.spec_from_file_location("export_robocasa_scene", _SCRIPT)
    assert spec is not None and spec.loader is not None
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class _FakeModel:
    def __init__(self) -> None:
        self._addr = {"obj_0_joint0": (7, 14)}

    def get_joint_qpos_addr(self, name: str):
        return self._addr[name]


class _FakeData:
    def __init__(self) -> None:
        self.qpos = [0.0] * 14
        self.qpos[7:14] = [1.25, -0.40, 0.95, 1.0, 0.0, 0.0, 0.0]


class TestBakeFreeBodyPoses(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.exp = _load_export()

    def test_bakes_freejoint_qpos_onto_body(self) -> None:
        root = ET.fromstring(
            """
            <mujoco>
              <worldbody>
                <body name="counter" pos="1 0 0.5"/>
                <body name="obj_0_main">
                  <joint name="obj_0_joint0" type="free"/>
                </body>
              </worldbody>
            </mujoco>
            """
        )
        env = SimpleNamespace(
            sim=SimpleNamespace(model=_FakeModel(), data=_FakeData())
        )
        n = self.exp._bake_free_body_poses(root, env)
        self.assertEqual(n, 1)
        obj = root.find(".//body[@name='obj_0_main']")
        assert obj is not None
        self.assertEqual(obj.get("pos"), "1.250000 -0.400000 0.950000")
        self.assertEqual(obj.get("quat"), "1.000000 0.000000 0.000000 0.000000")
        counter = root.find(".//body[@name='counter']")
        assert counter is not None
        self.assertEqual(counter.get("pos"), "1 0 0.5")

    def test_skips_when_sim_missing(self) -> None:
        root = ET.fromstring(
            '<mujoco><worldbody><body name="obj_0_main">'
            '<joint name="obj_0_joint0" type="free"/></body></worldbody></mujoco>'
        )
        self.assertEqual(self.exp._bake_free_body_poses(root, SimpleNamespace()), 0)

    def test_parse_g1_spawn(self) -> None:
        self.assertEqual(
            self.exp._parse_g1_spawn("2.0,-1.8,0.82"),
            (2.0, -1.8, 0.82),
        )
        self.assertEqual(
            self.exp._parse_g1_spawn("2.0, -1.8, 0.82"),
            (2.0, -1.8, 0.82),
        )
        self.assertIsNone(self.exp._parse_g1_spawn(None))


if __name__ == "__main__":
    unittest.main()
