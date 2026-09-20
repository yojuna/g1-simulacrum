"""Apply immutable rigid hand payloads to named MJCF anchor bodies."""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import TYPE_CHECKING

import mujoco
import numpy as np

if TYPE_CHECKING:
    from ..config import HandPayloadConfig, PayloadsConfig

PAYLOAD_BODY_NAMES = {
    "left": "left_hand_payload_link",
    "right": "right_hand_payload_link",
}
PAYLOAD_GEOM_NAMES = {
    "left": "left_hand_payload_geom",
    "right": "right_hand_payload_geom",
}


@dataclass(frozen=True, slots=True)
class AppliedHandPayload:
    side: str
    body_name: str
    body_id: int
    geom_name: str
    geom_id: int
    profile: str
    mass_kg: float
    com_pos_wrist_m: tuple[float, float, float]
    size_m: tuple[float, float, float]
    quat_wxyz: tuple[float, float, float, float]
    inertia_kg_m2: tuple[float, float, float]
    collision: bool

    def to_dict(self) -> dict[str, object]:
        return asdict(self)


@dataclass(frozen=True, slots=True)
class AppliedPayloads:
    left: AppliedHandPayload
    right: AppliedHandPayload
    model_total_mass_kg: float

    def to_dict(self) -> dict[str, object]:
        return {
            "left": self.left.to_dict(),
            "right": self.right.to_dict(),
            "model_total_mass_kg": self.model_total_mass_kg,
        }


def cuboid_diagonal_inertia(
    mass_kg: float,
    size_m: tuple[float, float, float],
) -> tuple[float, float, float]:
    """Return principal inertia of a uniform full-size cuboid at its COM."""
    x, y, z = (float(value) for value in size_m)
    m = float(mass_kg)
    return (
        m * (y * y + z * z) / 12.0,
        m * (x * x + z * z) / 12.0,
        m * (x * x + y * y) / 12.0,
    )


def _validate_hand(side: str, spec: HandPayloadConfig) -> None:
    if spec.mass_kg < 0.0:
        raise ValueError(f"{side} payload mass must be nonnegative")
    if any(value <= 0.0 for value in spec.size_m):
        raise ValueError(f"{side} payload size must be strictly positive")
    quat = np.asarray(spec.quat_wxyz, dtype=np.float64)
    if not np.isfinite(quat).all() or abs(float(np.linalg.norm(quat)) - 1.0) > 1e-6:
        raise ValueError(f"{side} payload quat_wxyz must be finite and unit length")
    if not np.isfinite(np.asarray(spec.com_pos_wrist_m, dtype=np.float64)).all():
        raise ValueError(f"{side} payload COM must be finite")


def _apply_hand(
    model: mujoco.MjModel,
    side: str,
    spec: HandPayloadConfig,
) -> AppliedHandPayload:
    _validate_hand(side, spec)
    body_name = PAYLOAD_BODY_NAMES[side]
    geom_name = PAYLOAD_GEOM_NAMES[side]
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    geom_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, geom_name)
    if body_id < 0 or geom_id < 0:
        raise ValueError(
            f"pinned MJCF missing payload anchor for {side}: "
            f"body={body_name!r} id={body_id}, geom={geom_name!r} id={geom_id}"
        )

    inertia = cuboid_diagonal_inertia(spec.mass_kg, spec.size_m)
    model.body_pos[body_id] = spec.com_pos_wrist_m
    model.body_quat[body_id] = spec.quat_wxyz
    model.body_ipos[body_id] = 0.0
    model.body_iquat[body_id] = (1.0, 0.0, 0.0, 0.0)
    model.body_mass[body_id] = spec.mass_kg
    model.body_inertia[body_id] = inertia

    model.geom_pos[geom_id] = 0.0
    model.geom_quat[geom_id] = (1.0, 0.0, 0.0, 0.0)
    model.geom_size[geom_id] = np.asarray(spec.size_m, dtype=np.float64) / 2.0
    model.geom_contype[geom_id] = 1 if spec.collision else 0
    model.geom_conaffinity[geom_id] = 1 if spec.collision else 0
    model.geom_rgba[geom_id, 3] = 0.85 if spec.mass_kg > 0.0 else 0.0

    return AppliedHandPayload(
        side=side,
        body_name=body_name,
        body_id=int(body_id),
        geom_name=geom_name,
        geom_id=int(geom_id),
        profile=spec.profile,
        mass_kg=float(spec.mass_kg),
        com_pos_wrist_m=tuple(float(x) for x in spec.com_pos_wrist_m),
        size_m=tuple(float(x) for x in spec.size_m),
        quat_wxyz=tuple(float(x) for x in spec.quat_wxyz),
        inertia_kg_m2=tuple(float(x) for x in inertia),
        collision=bool(spec.collision),
    )


def apply_payloads(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    config: PayloadsConfig,
) -> AppliedPayloads:
    """Apply both payloads once, recompute constants, and return measured state."""
    left = _apply_hand(model, "left", config.left)
    right = _apply_hand(model, "right", config.right)
    mujoco.mj_setConst(model, data)
    mujoco.mj_forward(model, data)

    for applied in (left, right):
        if not np.isclose(
            float(model.body_mass[applied.body_id]),
            applied.mass_kg,
            rtol=0.0,
            atol=1e-12,
        ):
            raise RuntimeError(f"{applied.side} payload mass failed post-apply verification")

    return AppliedPayloads(
        left=left,
        right=right,
        model_total_mass_kg=float(mujoco.mj_getTotalmass(model)),
    )
