# g1-simulacrum wiki

Compiled facts about the **Unitree G1** as this package models it: a 29-DoF
EDU body, **Dex3-1 hands by default**, Livox Mid-360 and Intel RealSense
D435i on `torso_link`, pelvis and torso IMUs, SDK2 low-level at 500 Hz.

This folder is the **fact book**. [`ARCHITECTURE.md`](../ARCHITECTURE.md) is
the **decision book**. If a number appears in both, they must match. If they
disagree, stop and fix one of them; do not invent a third value in code.

## Pages

| Page | Contents |
|------|----------|
| [Sources](sources.md) | Canonical URLs, pinned SHAs, how to update |
| [Platform](g1-platform.md) | Variants, DoF, size, mass, what this sim is |
| [Sensors](g1-sensors.md) | URDF mounts, Mid-360, D435i, four IMUs |
| [Control](g1-control.md) | `unitree_hg`, 500 Hz, joint indices, DDS topics |
| [Hands](g1-hands.md) | Wrist flange, Dex3-1, how kits swap |
| [Sim fidelity](sim-fidelity.md) | What MuJoCo can and cannot match |

## Rules for this wiki

1. Every number has a **source** (URL and, for git files, SHA).
2. Quote official values; do not round away digits that appear in URDF.
3. Degrees next to radians are convenience only. XML uses the radian form.
4. Marketing copy from unitree.com is labeled as product-page, not CAD.
5. When Unitree revises a URDF, add a dated row; bump the pin in
   Architecture and in the MJCF snapshot together.

Not a GitHub Wiki (separate git repo). Versioned here so PRs can review
citations with the rest of the project.
