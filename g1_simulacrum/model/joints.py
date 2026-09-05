"""Canonical G1 joint names (SDK2 G1JointIndex 0–28) and Dex3 fingers.

XML joints/actuators use a ``_joint`` suffix. Maps in the loader use these
canonical names without the suffix. Never slice ``qpos[7:36]``.
"""

from __future__ import annotations

# SDK2 G1JointIndex 0–28 / wiki/g1-control.md
BODY_JOINT_NAMES: tuple[str, ...] = (
    "left_hip_pitch",
    "left_hip_roll",
    "left_hip_yaw",
    "left_knee",
    "left_ankle_pitch",
    "left_ankle_roll",
    "right_hip_pitch",
    "right_hip_roll",
    "right_hip_yaw",
    "right_knee",
    "right_ankle_pitch",
    "right_ankle_roll",
    "waist_yaw",
    "waist_roll",
    "waist_pitch",
    "left_shoulder_pitch",
    "left_shoulder_roll",
    "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",
    "left_wrist_pitch",
    "left_wrist_yaw",
    "right_shoulder_pitch",
    "right_shoulder_roll",
    "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",
    "right_wrist_pitch",
    "right_wrist_yaw",
)

# Dex3-1: thumb, middle, index (Unitree IDL / with-hand MJCF)
HAND_JOINT_NAMES: tuple[str, ...] = (
    "left_hand_thumb_0",
    "left_hand_thumb_1",
    "left_hand_thumb_2",
    "left_hand_middle_0",
    "left_hand_middle_1",
    "left_hand_index_0",
    "left_hand_index_1",
    "right_hand_thumb_0",
    "right_hand_thumb_1",
    "right_hand_thumb_2",
    "right_hand_middle_0",
    "right_hand_middle_1",
    "right_hand_index_0",
    "right_hand_index_1",
)

NUM_BODY_JOINTS = len(BODY_JOINT_NAMES)
NUM_HAND_JOINTS = len(HAND_JOINT_NAMES)

REQUIRED_SITES: tuple[str, ...] = (
    "imu_in_pelvis",
    "imu_in_torso",
    "mid360",
)

REQUIRED_CAMERAS: tuple[str, ...] = (
    "d435i_depth",
    "d435i_rgb",
)


def xml_joint_name(canonical: str) -> str:
    return f"{canonical}_joint"
