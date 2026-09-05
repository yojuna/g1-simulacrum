# G1 sensors

Mount poses are **URDF**, not guesses. Sensor FOV/range/noise are
**datasheets**. Robot IMU topics are **SDK2**. Device IMUs on the lidar and
camera are not the control IMUs.

## Frames on the robot

G1 has **no actuated head**. Official docs place Mid-360 “in the middle of
the head” and D435i on the head; both URDF joints parent to `torso_link`.

| Official name | Parent | Role |
|---------------|--------|------|
| `pelvis` | floating base | Primary robot IMU (`rt/lowstate` → `imu_state`) |
| `torso_link` | waist chain | Secondary robot IMU (`rt/secondary_imu`) |
| `mid360_link` | `torso_link` | Livox optical / point-cloud frame (`livox_frame` on DDS) |
| `d435_link` | `torso_link` | RealSense body (this package: `d435i_link`) |

Do not invent `head_link` as a camera parent.

## Pinned URDF poses

Source: [`g1_29dof_rev_1_0.urdf`](https://github.com/unitreerobotics/unitree_ros/blob/7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74/robots/g1_description/g1_29dof_rev_1_0.urdf)
at **`unitree_ros@7c40519e02d7`** (2026-06-16).

xyz metres, rpy radians (URDF XYZ fixed-axis: roll, pitch, yaw).

| Joint | xyz | rpy | ≈ degrees |
|-------|-----|-----|-----------|
| `mid360_joint` | `0.0002835 0.00003 0.428434` | `3.141592653589793 0.05112069379091391 0` | roll 180°, pitch 2.93°, yaw 0 |
| `d435_joint` | `0.0576235 0.01753 0.42987` | `0 0.8307767239493009 0` | pitch 47.60° |
| `imu_in_torso_joint` | `-0.03959 -0.00224 0.14792` | `0 0 0` | — |
| `imu_in_pelvis_joint` | `0.04525 0 -0.08339` | `0 0 0` | — |

MJCF `g1_29dof_rev_1_0.xml` already has sites `imu_in_pelvis` and
`imu_in_torso` at those positions. It does **not** include `mid360_link` or
`d435_link`. Those come from our mount includes.

### Mid-360 pose revision (do not mix)

| When | xyz z | rpy | SHA / note |
|------|-------|-----|------------|
| Until 2026-06-16 | `0.41618` | `0, 0.04014257, 0` | pre-fix; still common in local clones |
| **Current pin** | `0.428434` | `π, 0.05112069, 0` | `7c40519` “fix g1 mid360_joint transform” |

The 180° roll is a real Unitree change, not a formatting quirk. Point-cloud
+Z must follow the **pinned** frame.

## MuJoCo-LiDAR (discoverse) vs our mount

Library: [discoverse-dev/MuJoCo-LiDAR](https://github.com/discoverse-dev/MuJoCo-LiDAR).
G1 demo: [`examples/unitree_g1.py`](https://github.com/discoverse-dev/MuJoCo-LiDAR/blob/main/examples/unitree_g1.py)
+ [`models/unitree_g1/g1_mjx_feetonly.xml`](https://github.com/discoverse-dev/MuJoCo-LiDAR/blob/main/models/unitree_g1/g1_mjx_feetonly.xml).

How they configure it:

| Piece | Discoverse G1 demo | This package |
|-------|--------------------|--------------|
| Parent | `torso_link` | `torso_link` → body `mid360_link` |
| Optical object | site `lidar` | site `mid360` on `mid360_link` |
| Translation | `pos="0 0 0.405"` | URDF `0.0002835 0.00003 0.428434` |
| Rotation | `quat="0 0 1 0"` (MuJoCo wxyz = **180° about Y**) | URDF **roll π** + 2.93° pitch |
| Pattern | `LivoxGenerator("mid360")` | same |
| Wrapper | `MjLidarWrapper(..., site_name="lidar", args={bodyexclude: torso_link})` | same wrapper, `site_name="mid360"`, same `bodyexclude` |

Ray math in the library (site frame):

```
x = cos(φ) cos(θ)
y = cos(φ) sin(θ)
z = sin(φ)
```

Positive φ is **+Z of the site**. Livox vertical FOV is biased (+52° vs −7°),
so +Z of the site must point **down** on the standing robot or the fat lobe
covers the sky.

Both mounts flip site +Z downward. They are **not the same rotation**:

- Discoverse 180° about Y also flips site +X to torso **−X** (backward).
- Unitree roll π keeps site +X ≈ torso **+X** (forward), which matches
  Unitree’s “lidar X toward the front of the robot” wording.

There is **no library conflict**. `mujoco-lidar` only needs a site pose and
theta/phi. Copying discoverse’s site into our XML would fight the URDF pin
and yaw the optical frame 180°. Keep Unitree; the Livox flower will still
look like a Mid-360, with X-forward consistent with `livox_frame`.

Also copy discoverse’s **self-hit** practice: exclude `torso_link` (head
mesh lives there) so rays are not eaten by the skull.

## Four IMUs

| IMU | Location | Bus / topic | Rate | Chip (if known) |
|-----|----------|-------------|------|-----------------|
| Pelvis (primary) | `imu_in_pelvis` | `rt/lowstate` → `imu_state` | 500 Hz with LowState | robot IMU |
| Torso (secondary) | `imu_in_torso` | `rt/secondary_imu` (`IMUState_`) | same low-level path (example prints both at 1 Hz from a 500 Hz counter) | robot IMU |
| Mid-360 | inside lidar | `rt/utlidar/imu_livox_mid360` | **200 Hz** | Livox: ICM-40609 |
| D435i | inside camera | RealSense motion streams | gyro typically 200 Hz; accel 63–250 Hz class | Bosch BMI055 |

SDK2’s ankle-swing example labels `low_state.imu_state()` as **pelvis** and
`rt/secondary_imu` as **torso**. `LowState_` carries one IMU, not two.
[unitree_ros#121](https://github.com/unitreerobotics/unitree_ros/issues/121)
is the same confusion.

Mid-360 IMU relative to the lidar (Unitree lidar service page): translation
`(0.011, 0.02329, -0.04412)` m, **no rotation**. Use that offset for a
device-IMU site on `mid360_link`.

D435i IMU extrinsics are in the RealSense SDK (mechanical drawing, not
user-calibrated). The D435i IMU has **no factory calibration**; idle gyro
bias and |a| ≠ g are expected on hardware.

## Livox Mid-360

[Livox Mid-360 specs](https://www.livoxtech.com/mid-360/specs), retrieved
2026-09-05.

| Item | Value |
|------|--------|
| Model | MID-360 |
| Wavelength | 905 nm, Class 1 |
| FOV | **360° H**, **−7° to +52° V** (59° vertical span; Unitree marketing says “59°”) |
| Range @ 100 klx | 40 m @ 10% reflectivity; 70 m @ 80% |
| Blind / close | 0.1 m specified blind; 0.1–0.2 m detectable but precision not guaranteed |
| Range precision (1σ) | ≤ 2 cm @ 10 m; ≤ 3 cm @ 0.2 m (80% refl., 25 °C; see notes on datasheet) |
| Angular precision (1σ) | < 0.15° |
| Point rate | 200 000 pts/s (first return) |
| Frame rate | **10 Hz typical** (Unitree DDS cloud topic also 10 Hz) |
| Scan | non-repetitive (not a spinning HDL ring) |
| IMU | ICM-40609 |
| Mass / size | 265 g; 65 × 65 × 60 mm |
| Power | 6.5 W avg (up to 14 W self-heat below 0 °C) |
| IP | IP67 |

Unitree DDS (lidar service ≥ 1.0.0.5):

- Cloud: `rt/utlidar/cloud_livox_mid360`, 10 Hz, `frame_id` `"livox_frame"`
- IMU: `rt/utlidar/imu_livox_mid360`, 200 Hz

G1 units **produced after April 2026** may ship **Mid-360s** (Unitree lidar
page). This package still models the Mid-360 datasheet until we pin a
Mid-360s spec.

## Intel RealSense D435i

[Intel D435i product page](https://www.intelrealsense.com/depth-camera-d435i/),
retrieved via Intel’s published tech specs (2026-09-05).

| Item | Value |
|------|--------|
| Depth | stereoscopic, global shutter (D430 module) |
| Depth FOV | **87° × 58°** (H × V) |
| Depth resolution / rate | up to 1280 × 720, **up to 90 fps** |
| Min-Z at max res | ~28 cm |
| Ideal range | **0.3–3 m** |
| Depth accuracy (Intel) | < 2% at 2 m |
| RGB | 1920 × 1080, **30 fps**, rolling shutter |
| RGB FOV | **69° × 42°** |
| IMU | BMI055, timestamped to depth clock |
| Mass / size (module) | ~72 g; ~90 × 25 × 25 mm (Intel peripheral, not G1 housing) |

This package: RGB and depth at **30 Hz**, 640 × 480, to keep the sim cheap.
Hardware can do 90 fps depth; say so in config comments.

MuJoCo cameras are **pinhole**. D435i depth is **stereo**. Separate MJCF
cameras: depth `fovy="58"` (Intel V), RGB `fovy="42"`. Do not render both
from one camera. See [sim-fidelity.md](sim-fidelity.md).
