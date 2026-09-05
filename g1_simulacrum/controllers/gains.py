"""Per-joint PD gains for the G1 29 body DoF.

Cited starting point from unitree_sdk2 ``g1_ankle_swing_example.cpp``
(wiki/g1-control.md). These are **not** measured on a specific robot.

Legs: Kp 60, 60, 60, 100, 40, 40 (each side); Kd 1, 1, 1, 2, 1, 1.
Waist: Kp 60, 40, 40; Kd 1, 1, 1.
Arms: Kp 40 × 7 per arm; Kd 1 × 7.
"""

from __future__ import annotations

from ..model.joints import BODY_JOINT_NAMES as _BODY

# fmt: off
G1_PD_GAINS: dict[str, dict[str, float]] = {
    "left_hip_pitch": {"kp": 60.0, "kd": 1.0},
    "left_hip_roll": {"kp": 60.0, "kd": 1.0},
    "left_hip_yaw": {"kp": 60.0, "kd": 1.0},
    "left_knee": {"kp": 100.0, "kd": 2.0},
    "left_ankle_pitch": {"kp": 40.0, "kd": 1.0},
    "left_ankle_roll": {"kp": 40.0, "kd": 1.0},
    "right_hip_pitch": {"kp": 60.0, "kd": 1.0},
    "right_hip_roll": {"kp": 60.0, "kd": 1.0},
    "right_hip_yaw": {"kp": 60.0, "kd": 1.0},
    "right_knee": {"kp": 100.0, "kd": 2.0},
    "right_ankle_pitch": {"kp": 40.0, "kd": 1.0},
    "right_ankle_roll": {"kp": 40.0, "kd": 1.0},
    "waist_yaw": {"kp": 60.0, "kd": 1.0},
    "waist_roll": {"kp": 40.0, "kd": 1.0},
    "waist_pitch": {"kp": 40.0, "kd": 1.0},
    "left_shoulder_pitch": {"kp": 40.0, "kd": 1.0},
    "left_shoulder_roll": {"kp": 40.0, "kd": 1.0},
    "left_shoulder_yaw": {"kp": 40.0, "kd": 1.0},
    "left_elbow": {"kp": 40.0, "kd": 1.0},
    "left_wrist_roll": {"kp": 40.0, "kd": 1.0},
    "left_wrist_pitch": {"kp": 40.0, "kd": 1.0},
    "left_wrist_yaw": {"kp": 40.0, "kd": 1.0},
    "right_shoulder_pitch": {"kp": 40.0, "kd": 1.0},
    "right_shoulder_roll": {"kp": 40.0, "kd": 1.0},
    "right_shoulder_yaw": {"kp": 40.0, "kd": 1.0},
    "right_elbow": {"kp": 40.0, "kd": 1.0},
    "right_wrist_roll": {"kp": 40.0, "kd": 1.0},
    "right_wrist_pitch": {"kp": 40.0, "kd": 1.0},
    "right_wrist_yaw": {"kp": 40.0, "kd": 1.0},
}
# fmt: on

# Hold Dex3 at reset qpos. Not a HandController; just so fingers do not flop.
FINGER_HOLD_KP = 2.0
FINGER_HOLD_KD = 0.05

assert tuple(G1_PD_GAINS) == _BODY
