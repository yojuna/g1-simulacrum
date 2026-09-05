"""Compile pinned MJCF with ``MjModel.from_xml_path`` only.

No ElementTree, no temp XML, no Menagerie hunt at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import mujoco

from .joints import (
    BODY_JOINT_NAMES,
    HAND_JOINT_NAMES,
    REQUIRED_CAMERAS,
    REQUIRED_SITES,
    xml_joint_name,
)

_MJCF_DIR = Path(__file__).resolve().parent / "mjcf"

_XML_BY_HANDS = {
    "dex3": "g1_sensorized.xml",
    "none": "g1_sensorized_none.xml",
}


@dataclass(frozen=True, slots=True)
class CompiledModel:
    model: mujoco.MjModel
    xml_path: Path
    body_joint_ids: dict[str, int]
    body_qposadr: dict[str, int]
    body_dofadr: dict[str, int]
    body_actuator_ids: dict[str, int]
    hand_joint_ids: dict[str, int]
    hand_qposadr: dict[str, int]
    hand_dofadr: dict[str, int]
    hand_actuator_ids: dict[str, int]


class ModelLoader:
    """Load ``g1_sensorized.xml`` (Dex3) or ``g1_sensorized_none.xml``."""

    def __init__(self, *, hands: str = "dex3", xml_path: str | Path | None = None) -> None:
        self._hands = hands
        self._xml_path = Path(xml_path) if xml_path is not None else None

    def resolve_xml_path(self) -> Path:
        if self._xml_path is not None:
            path = self._xml_path
        else:
            try:
                name = _XML_BY_HANDS[self._hands]
            except KeyError as exc:
                raise ValueError(
                    f"unknown hands kit {self._hands!r}; expected dex3|none"
                ) from exc
            path = _MJCF_DIR / name
        if not path.is_file():
            raise FileNotFoundError(f"MJCF not found: {path}")
        return path

    def build(self) -> CompiledModel:
        xml_path = self.resolve_xml_path()
        model = mujoco.MjModel.from_xml_path(str(xml_path))
        self._require_names(model)
        body_j, body_q, body_v, body_a = self._map_group(model, BODY_JOINT_NAMES, required=True)
        hand_j, hand_q, hand_v, hand_a = self._map_group(
            model, HAND_JOINT_NAMES, required=False
        )
        if self._hands == "dex3" and len(hand_j) != len(HAND_JOINT_NAMES):
            missing = [n for n in HAND_JOINT_NAMES if n not in hand_j]
            raise ValueError(f"Dex3 model missing finger joints: {missing}")
        if self._hands == "none" and hand_j:
            raise ValueError(f"hands=none must not have finger joints: {list(hand_j)}")
        return CompiledModel(
            model=model,
            xml_path=xml_path,
            body_joint_ids=body_j,
            body_qposadr=body_q,
            body_dofadr=body_v,
            body_actuator_ids=body_a,
            hand_joint_ids=hand_j,
            hand_qposadr=hand_q,
            hand_dofadr=hand_v,
            hand_actuator_ids=hand_a,
        )

    def _require_names(self, model: mujoco.MjModel) -> None:
        missing: list[str] = []
        for site in REQUIRED_SITES:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site) < 0:
                missing.append(f"site:{site}")
        for cam in REQUIRED_CAMERAS:
            if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_CAMERA, cam) < 0:
                missing.append(f"camera:{cam}")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "mid360_link") < 0:
            missing.append("body:mid360_link")
        if mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "torso_link") < 0:
            missing.append("body:torso_link")
        if missing:
            raise ValueError(
                f"pinned MJCF {self.resolve_xml_path()} missing required names: {missing}"
            )

    def _map_group(
        self,
        model: mujoco.MjModel,
        names: tuple[str, ...],
        *,
        required: bool,
    ) -> tuple[dict[str, int], dict[str, int], dict[str, int], dict[str, int]]:
        joints: dict[str, int] = {}
        qposadr: dict[str, int] = {}
        dofadr: dict[str, int] = {}
        actuators: dict[str, int] = {}
        missing: list[str] = []
        for name in names:
            xml_name = xml_joint_name(name)
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, xml_name)
            aid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, xml_name)
            if jid < 0 or aid < 0:
                if required:
                    missing.append(xml_name)
                continue
            joints[name] = int(jid)
            qposadr[name] = int(model.jnt_qposadr[jid])
            dofadr[name] = int(model.jnt_dofadr[jid])
            actuators[name] = int(aid)
        if missing:
            raise ValueError(f"missing required body joints/actuators: {missing}")
        return joints, qposadr, dofadr, actuators
