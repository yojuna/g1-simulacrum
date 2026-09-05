# Simulation fidelity

Goal: kinematics, IMU **sites**, control rate, and sensor **geometry** match
the cited robot. Optics and noise will not be factory-identical. Label the
gap instead of hiding it.

## High (copy XML / URDF)

| Aspect | How |
|--------|-----|
| 29-DoF names, limits, inertias | Pinned `g1_29dof_rev_1_0` MJCF |
| Pelvis / torso IMU locations | Unitree sites; same numbers as URDF |
| Lidar and camera **mounts** | URDF `mid360_joint` / `d435_joint` at pin `7c40519` (not discoverse’s G1 site) |
| Dex3 kinematics | Unitree with-hand MJCF; flange `0.0415 ±0.003` m |
| Control rate | 500 Hz apply, 1000 Hz physics |
| Joint order | `G1JointIndex` 0–28; fingers named separately |

If a later Unitree URDF revises a pose, bump the pin; do not lerp in Python.

## Medium (same idea, not the same physics)

### Mid-360

- Rays from a site on `mid360_link` with Livox **non-repetitive** pattern
  (`LivoxGenerator` / mid360 in `mujoco-lidar`), 10 Hz, FOV 360° × (−7°, +52°).
- Not: real photonics, multi-return, 100 klx sunlight, IP67 housing, PTP.
- Range 1σ may start at Livox’s ≤ 2 cm @ 10 m; dropout is a **placeholder**
  until we have robot logs.
- CPU raycast first. YAML default is `backend: cpu`. GPU/Warp is an image
  extra, not the package default.
- Cosmetic lidar/camera housings are geom **group 4**; rays skip that group
  so they are not eaten by a 3 cm shell. Also `bodyexclude=torso_link`.

### D435i

- Two pinhole cameras (depth fovy 58°, RGB fovy 42°), 30 Hz. Cameras look
  along `d435i_link` **+X** (`xyaxes="0 -1 0 0 0 1"`), matching URDF
  camera-link, not MuJoCo’s default +Z (sky).
- On **MuJoCo 3.12**, `Renderer.enable_depth_rendering()` already returns
  **metric metres**. Do not apply the older OpenGL z-buffer formula again.
- Not: stereo matching, IR projector, rolling-shutter RGB, Intel’s <2% @ 2 m
  as a guaranteed sim error, 90 fps.
- Useful depth band in config: 0.3–3 m. Do not advertise 0.1–10 m as equally
  good.

### IMUs

- MuJoCo `<accelerometer>` / `<gyro>` on the four sites, sampled at the
  rates in [g1-sensors.md](g1-sensors.md).
- Noise: start from datasheet / Unitree MJCF `noise=` / `cutoff=` on the
  robot IMUs (`5e-4` gyro, `1e-2` accel in Unitree XML). Device IMUs get
  their own blocks (ICM-40609 vs BMI055). Not “one IMU for the whole head.”

## Low / out of scope (do not fake)

- Dex3 tactile arrays and finger tendons (Dex3 *geometry* is in; control is next)
- Discoverse G1 lidar site as CAD (`pos 0 0 0.405`, 180° about Y)
- DDS, CRC, `mode_machine`, motion-control service
- Jetson / NX video hub JPEG path
- Mid-360s hardware swap (post-April 2026 G1) until we pin that datasheet
- Calibrated noise from **this** robot’s bags

## Checklist before claiming “faithful”

1. Mount XML matches the URDF table to the last digit we copied.
2. Lidar rays are not emitted at `torso_link` origin; site frame is Unitree, not discoverse’s Y-flip.
3. Camera parent is `torso_link`, not a made-up `head_link`. Cameras look along body +X.
4. `control_hz: 500`, not 200.
5. Hands attach at Unitree palm joints; Dex3 is not folded into the 29.
6. README / Architecture / wiki numbers are the same.
