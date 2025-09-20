#!/usr/bin/env python3

# Head motor configuration
HEAD_MOTORS = {
    1: {"name": "Head Roll", "min": -0.45, "max": 0.45, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 5.0,
        "cur": 2.0, "zero_pos": 0.0},
    2: {"name": "Head Pitch", "min": -0.43, "max": 0.43, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 5.0,
        "cur": 2.0, "zero_pos": 0.0},
    3: {"name": "Head Yaw", "min": -1.57, "max": 1.57, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 5.0,
        "cur": 2.0, "zero_pos": 0.0}
}

ARM_MOTORS = {
    11: {"name": "Left Shoulder Pitch", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 12.0,
        "cur": 6.0, "zero_pos": -0.016},
    12: {"name": "Left Shoulder Roll", "min": -0.26, "max": 2.61, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
        "cur": 5.0, "zero_pos": 0.098},
    13: {"name": "Left Shoulder Yaw", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 8.0,
        "cur": 4.0, "zero_pos": -0.142},
    14: {"name": "Left Elbow Pitch", "min": -2.61, "max": 0.26, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 8.0,
        "cur": 4.0, "zero_pos": -0.347},
    15: {"name": "Left Wrist Yaw", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 6.0,
        "cur": 3.0, "zero_pos": 0.03},
    16: {"name": "Left Wrist Pitch", "min": -0.78, "max": 1.04, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 2.0, "zero_pos": 0.0},
    17: {"name": "Left Wrist Roll", "min": -1.65, "max": 1.3, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 2.0, "zero_pos": 0.0},
    21: {"name": "Right Shoulder Pitch", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 12.0,
        "cur": 6.0, "zero_pos": -0.041},
    22: {"name": "Right Shoulder Roll", "min": -2.61, "max": 0.26, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
        "cur": 5.0, "zero_pos": -0.129},
    23: {"name": "Right Shoulder Yaw", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 8.0,
        "cur": 4.0, "zero_pos": 0.168},
    24: {"name": "Right Elbow Pitch", "min": -2.61, "max": 0.26, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 8.0,
        "cur": 4.0, "zero_pos": -0.386},
    25: {"name": "Right Wrist Yaw", "min": -2.96, "max": 2.96, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 6.0,
        "cur": 3.0, "zero_pos": -0.035},
    26: {"name": "Right Wrist Pitch", "min": -0.78, "max": 1.04, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 2.0, "zero_pos": 0.0},
    27: {"name": "Right Wrist Roll", "min": -1.65, "max": 1.3, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 2.0, "zero_pos": 0.0}
}

WAIST_MOTORS = {
    31: {"name": "Waist Yaw", "min": -2.96, "max": 2.96, "velocity": 0.1, "max_velocity": 0.3, "max_cur": 12.0,
        "cur": 6.0, "zero_pos": 0.0},
}

LEG_MOTORS = {
    51: {"name": "Left Hip Roll", "min": -0.78, "max": 0.78, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 16.0,
        "cur": 8.0, "zero_pos": -0.023},
    52: {"name": "Left Hip Pitch", "min": -2.79, "max": 2.09, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 20.0,
        "cur": 10.0, "zero_pos": -0.50},
    53: {"name": "Left Hip Yaw", "min": -1.04, "max": 1.04, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 16.0,
        "cur": 8.0, "zero_pos": 0.0},
    54: {"name": "Left Knee Pitch", "min": 0.0, "max": 2.39, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 20.0,
        "cur": 10.0, "zero_pos": 0.985},
    55: {"name": "Left Wrist Pitch", "min": -1.22, "max": 0.52, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
        "cur": 5.0, "zero_pos": -0.473},
    56: {"name": "Left Wrist Roll", "min": -1.04, "max": 0.0, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
        "cur": 5.0, "zero_pos": -0.52},
    61: {"name": "Right Hip Roll", "min": -0.78, "max": 0.78, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 16.0,
         "cur": 8.0, "zero_pos": 0.023},
    62: {"name": "Right Hip Pitch", "min": -2.79, "max": 2.09, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 20.0,
         "cur": 10.0, "zero_pos": -0.50},
    63: {"name": "Right Hip Yaw", "min": -1.04, "max": 1.04, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 16.0,
         "cur": 8.0, "zero_pos": 0.0},
    64: {"name": "Right Knee Pitch", "min": 0.0, "max": 2.39, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 20.0,
         "cur": 10.0, "zero_pos": 0.985},
    65: {"name": "Right Wrist Pitch", "min": -1.22, "max": 0.52, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
         "cur": 5.0, "zero_pos": -0.473},
    66: {"name": "Right Wrist Roll", "min": -1.04, "max": 0.0, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 10.0,
         "cur": 5.0, "zero_pos": -0.52}
}

# Add configurations for other body parts here
# For example:
# ARM_MOTORS = { ... }
# LEG_MOTORS = { ... }

# Body parts to ROS topics mapping
BODY_PARTS_TOPICS = {
    "head": {
        "cmd_topic": "/head/cmd_pos",
        "status_topic": "/head/status",
        "motors": HEAD_MOTORS
    },
    "arms": {
        "cmd_topic": "/arm/cmd_pos",
        "status_topic": "/arm/status",
        "motors": ARM_MOTORS
    },
    "waist": {
        "cmd_topic": "/waist/cmd_pos",
        "status_topic": "/waist/status",
        "motors": WAIST_MOTORS
    },
    "legs": {
        "cmd_topic": "/leg/cmd_pos",
        "status_topic": "/leg/status",
        "motors": LEG_MOTORS
    }
}