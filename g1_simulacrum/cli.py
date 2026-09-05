"""Tiny CLI: compile the pinned model and print maps."""

from __future__ import annotations

import argparse

from .config import G1SimulacrumConfig
from .simulacrum import G1Simulacrum


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="g1-simulacrum")
    parser.add_argument("--config", default="configs/default.yaml")
    args = parser.parse_args(argv)
    sim = G1Simulacrum.from_config(args.config)
    sim.build()
    c = sim.compiled
    print(f"xml {c.xml_path}")
    print(f"bodies {sim.model.nbody} joints {sim.model.njnt} actuators {sim.model.nu}")
    print(f"body map {len(c.body_joint_ids)} hand map {len(c.hand_joint_ids)}")
    print(f"control {sim.config.controller.control_hz} Hz physics {sim.config.controller.physics_hz} Hz")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
