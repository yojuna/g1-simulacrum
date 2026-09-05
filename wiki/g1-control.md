# G1 control (SDK2)

G1 low-level is **`unitree_hg`**, not the Go2 `unitree_go` types. This
package does not speak DDS; the rates and joint order below are what the
sim must **match** so later stacks can compose.

Source of truth:
[`g1_ankle_swing_example.cpp`](https://github.com/unitreerobotics/unitree_sdk2/blob/main/example/g1/low_level/g1_ankle_swing_example.cpp)
on unitree_sdk2 `main` (read 2026-09-05).

## Rate

| Quantity | Value | Evidence |
|----------|--------|----------|
| Control / command period | **2 ms (500 Hz)** | `control_dt_ = 0.002`; recurrent threads `2000` µs |
| Physics in this sim | 1000 Hz | Architecture choice: two substeps per control step |
| Not 200 Hz | — | 200 Hz is common on other Unitree platforms / some Isaac graphs; it is **not** G1 `LowCmd` |

`G1_NUM_MOTOR = 29`. `LowCmd_` motor array is sized **35** in the IDL
(padding / reserved slots, e.g. arm-sdk weight on index 29). This package
exposes **29** named joints only.

## Topics (hardware)

| Topic | Type | Direction | Role |
|-------|------|-----------|------|
| `rt/lowcmd` | `unitree_hg::LowCmd_` | to robot | `q, dq, tau, kp, kd`, `mode_pr`, `mode_machine`, CRC |
| `rt/lowstate` | `unitree_hg::LowState_` | from robot | 29 motor states + **pelvis** `imu_state` |
| `rt/secondary_imu` | `IMUState_` | from robot | **torso** IMU |
| `rt/utlidar/cloud_livox_mid360` | PointCloud2 | from robot | 10 Hz |
| `rt/utlidar/imu_livox_mid360` | Imu | from robot | 200 Hz |

CRC is required on LowCmd/LowState. Irrelevant inside MuJoCo; relevant if
a later extra speaks DDS.

## `G1JointIndex` (canonical 0–28)

Same names as Menagerie / this package. Comments are from the SDK example.

| i | Name | Notes |
|---|------|--------|
| 0–5 | `left_hip_pitch` … `left_ankle_roll` | ankle pitch/roll also called ankle B/A in AB mode |
| 6–11 | `right_hip_pitch` … `right_ankle_roll` | same |
| 12 | `waist_yaw` | |
| 13 | `waist_roll` | invalid on 23-DoF and waist-locked 29-DoF |
| 14 | `waist_pitch` | same |
| 15–21 | `left_shoulder_pitch` … `left_wrist_yaw` | wrist pitch/yaw invalid on 23-DoF |
| 22–28 | `right_shoulder_pitch` … `right_wrist_yaw` | same |

Never treat `qpos[7:36]` as the public contract. Map **names → MuJoCo ids**
at compile time. Freejoint base stays `qpos[0:7]` / `qvel[0:6]`.

## Ankle PR vs AB

SDK `mode_pr`: `0` = series pitch/roll, `1` = parallel A/B. Same two motors,
different command meaning. This package’s PD loop commands **named joints**
in Menagerie (pitch/roll). An AB-mode adapter is not core.

## Example PD tables (SDK sample, not our tuned gains)

From the same example (`Kp` / `Kd` arrays). Copy into `gains.py` only if we
deliberately adopt them; they are a **cited starting point**, not measured
on a specific robot.

Legs: Kp `60, 60, 60, 100, 40, 40` (each side); Kd `1, 1, 1, 2, 1, 1`.
Waist: Kp `60, 40, 40`; Kd `1, 1, 1`.
Arms: Kp `40` × 7 per arm; Kd `1` × 7.

## What this package implements

| Type | Input | Output |
|------|--------|--------|
| `pd` | `(29,)` position targets, canonical order | PD torques |
| `passthrough` | `(29,)` torques | `data.ctrl` |

No `sonic` controller type. No DDS client in core.
