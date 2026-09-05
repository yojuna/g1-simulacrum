"""Per-joint PD gains for the Unitree G1 (29 DOF).

These are calibrated to match the GEAR-SONIC deployment configuration.
Joint ordering follows the SONIC convention.

Sources:
    - GR00T-WholeBodyControl/gear_sonic_deploy/policy/*/observation_config.yaml
    - SONIC paper Table S1
"""

# fmt: off
G1_JOINT_NAMES: list[str] = [
    # Legs (12)
    "left_hip_pitch",    "left_hip_roll",     "left_hip_yaw",
    "left_knee",         "left_ankle_pitch",  "left_ankle_roll",
    "right_hip_pitch",   "right_hip_roll",    "right_hip_yaw",
    "right_knee",        "right_ankle_pitch", "right_ankle_roll",
    # Waist (3)
    "waist_yaw",         "waist_roll",        "waist_pitch",
    # Left arm (7)
    "left_shoulder_pitch",  "left_shoulder_roll",  "left_shoulder_yaw",
    "left_elbow",
    "left_wrist_roll",     "left_wrist_pitch",    "left_wrist_yaw",
    # Right arm (7)
    "right_shoulder_pitch", "right_shoulder_roll", "right_shoulder_yaw",
    "right_elbow",
    "right_wrist_roll",    "right_wrist_pitch",   "right_wrist_yaw",
]

# PD gains per joint index, matching SONIC deployment.
# Legs have high stiffness; wrists are more compliant.
G1_PD_GAINS: dict[int, dict[str, float]] = {
    # Left leg
    0:  {"kp": 200.0, "kd": 5.0},   # left_hip_pitch
    1:  {"kp": 200.0, "kd": 5.0},   # left_hip_roll
    2:  {"kp": 100.0, "kd": 2.0},   # left_hip_yaw
    3:  {"kp": 200.0, "kd": 5.0},   # left_knee
    4:  {"kp": 40.0,  "kd": 2.0},   # left_ankle_pitch
    5:  {"kp": 40.0,  "kd": 2.0},   # left_ankle_roll
    # Right leg
    6:  {"kp": 200.0, "kd": 5.0},   # right_hip_pitch
    7:  {"kp": 200.0, "kd": 5.0},   # right_hip_roll
    8:  {"kp": 100.0, "kd": 2.0},   # right_hip_yaw
    9:  {"kp": 200.0, "kd": 5.0},   # right_knee
    10: {"kp": 40.0,  "kd": 2.0},   # right_ankle_pitch
    11: {"kp": 40.0,  "kd": 2.0},   # right_ankle_roll
    # Waist
    12: {"kp": 400.0, "kd": 5.0},   # waist_yaw
    13: {"kp": 400.0, "kd": 5.0},   # waist_roll
    14: {"kp": 400.0, "kd": 5.0},   # waist_pitch
    # Left arm
    15: {"kp": 100.0, "kd": 2.0},   # left_shoulder_pitch
    16: {"kp": 100.0, "kd": 2.0},   # left_shoulder_roll
    17: {"kp": 50.0,  "kd": 1.0},   # left_shoulder_yaw
    18: {"kp": 100.0, "kd": 2.0},   # left_elbow
    19: {"kp": 10.0,  "kd": 0.2},   # left_wrist_roll
    20: {"kp": 10.0,  "kd": 0.2},   # left_wrist_pitch
    21: {"kp": 4.0,   "kd": 0.1},   # left_wrist_yaw
    # Right arm
    22: {"kp": 100.0, "kd": 2.0},   # right_shoulder_pitch
    23: {"kp": 100.0, "kd": 2.0},   # right_shoulder_roll
    24: {"kp": 50.0,  "kd": 1.0},   # right_shoulder_yaw
    25: {"kp": 100.0, "kd": 2.0},   # right_elbow
    26: {"kp": 10.0,  "kd": 0.2},   # right_wrist_roll
    27: {"kp": 10.0,  "kd": 0.2},   # right_wrist_pitch
    28: {"kp": 4.0,   "kd": 0.1},   # right_wrist_yaw
}
# fmt: on

# Convenience arrays
KP_ARRAY = [G1_PD_GAINS[i]["kp"] for i in range(29)]
KD_ARRAY = [G1_PD_GAINS[i]["kd"] for i in range(29)]
