from constants import *
from multiprocessing import Value, Manager

# manager = None
#
# def initialize_multiprocessing():
#     global manager
#     manager = Manager()

def generate_control_map():
    return {
        CTRL_R_TRIGGER: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_RIGHT_TRIGGER, CTRL_NETWORK_LABEL: NWK_R_TRIGGER},
        CTRL_L_TRIGGER: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_LEFT_TRIGGER, CTRL_NETWORK_LABEL: NWK_L_TRIGGER},
        CTRL_L_THUMB_Y: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_LEFT_THUMB_Y, CTRL_NETWORK_LABEL: NWK_L_THUMB_Y},
        CTRL_L_THUMB_X: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_LEFT_THUMB_X, CTRL_NETWORK_LABEL: NWK_L_THUMB_X},
        CTRL_R_THUMB_Y: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_RIGHT_THUMB_Y, CTRL_NETWORK_LABEL: NWK_R_THUMB_Y},
        CTRL_R_THUMB_X: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: INPUT_RIGHT_THUMB_X, CTRL_NETWORK_LABEL: NWK_R_THUMB_X},

        CTRL_BUTTON_DPAD_UP: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 1, CTRL_NETWORK_LABEL: NWK_DPAD_UP},
        CTRL_BUTTON_DPAD_DOWN: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 2, CTRL_NETWORK_LABEL: NWK_DPAD_DOWN},
        CTRL_BUTTON_DPAD_LEFT: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 3, CTRL_NETWORK_LABEL: NWK_DPAD_LEFT},
        CTRL_BUTTON_DPAD_RIGHT: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 4, CTRL_NETWORK_LABEL: NWK_DPAD_RIGHT},

        CTRL_BUTTON_A: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 13, CTRL_NETWORK_LABEL: NWK_BUTTON_A},
        CTRL_BUTTON_B: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 14, CTRL_NETWORK_LABEL: NWK_BUTTON_B},
        CTRL_BUTTON_X: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 15, CTRL_NETWORK_LABEL: NWK_BUTTON_X},
        CTRL_BUTTON_Y: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 16, CTRL_NETWORK_LABEL: NWK_BUTTON_Y},

        CTRL_BUTTON_L1: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 9, CTRL_NETWORK_LABEL: NWK_BUTTON_L1},
        CTRL_BUTTON_R1: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 10, CTRL_NETWORK_LABEL: NWK_BUTTON_R1},
        CTRL_BUTTON_L3: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 7, CTRL_NETWORK_LABEL: NWK_BUTTON_L3},
        CTRL_BUTTON_R3: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 8, CTRL_NETWORK_LABEL: NWK_BUTTON_R3},

        CTRL_BUTTON_START: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 5, CTRL_NETWORK_LABEL: NWK_BUTTON_START},
        CTRL_BUTTON_BACK: {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 6, CTRL_NETWORK_LABEL: NWK_BUTTON_BACK}
    }

def generate_multiprocess_control_map2(manager):
    return manager.dict(generate_control_map())

def generate_multiprocess_state_map2(manager):
    return manager.dict(generate_state_map())


# def generate_multiprocess_state_map2(manager):
#     d = manager.dict()
#     state_map = generate_state_map()
#     for key in state_map:
#         d[key] = state_map[key]
#     return d

def generate_multiprocess_control_map():
    return {
        CTRL_R_TRIGGER: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_RIGHT_TRIGGER, CTRL_NETWORK_LABEL: NWK_R_TRIGGER},
        CTRL_L_TRIGGER: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_LEFT_TRIGGER, CTRL_NETWORK_LABEL: NWK_L_TRIGGER},
        CTRL_L_THUMB_Y: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_LEFT_THUMB_Y, CTRL_NETWORK_LABEL: NWK_L_THUMB_Y},
        CTRL_L_THUMB_X: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_LEFT_THUMB_X, CTRL_NETWORK_LABEL: NWK_L_THUMB_X},
        CTRL_R_THUMB_Y: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_RIGHT_THUMB_Y, CTRL_NETWORK_LABEL: NWK_L_THUMB_Y},
        CTRL_R_THUMB_X: {CTRL_VALUE: manager.Value('d', 0.0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: INPUT_RIGHT_THUMB_X, CTRL_NETWORK_LABEL: NWK_L_THUMB_X},

        CTRL_BUTTON_DPAD_UP: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 1, CTRL_NETWORK_LABEL: NWK_DPAD_UP},
        CTRL_BUTTON_DPAD_DOWN: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 2, CTRL_NETWORK_LABEL: NWK_DPAD_DOWN},
        CTRL_BUTTON_DPAD_LEFT: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 3, CTRL_NETWORK_LABEL: NWK_DPAD_LEFT},
        CTRL_BUTTON_DPAD_RIGHT: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 4, CTRL_NETWORK_LABEL: NWK_DPAD_RIGHT},

        CTRL_BUTTON_A: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 13, CTRL_NETWORK_LABEL: NWK_BUTTON_A},
        CTRL_BUTTON_B: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 14, CTRL_NETWORK_LABEL: NWK_BUTTON_B},
        CTRL_BUTTON_X: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 15, CTRL_NETWORK_LABEL: NWK_BUTTON_X},
        CTRL_BUTTON_Y: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 16, CTRL_NETWORK_LABEL: NWK_BUTTON_Y},

        CTRL_BUTTON_L1: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 9, CTRL_NETWORK_LABEL: NWK_BUTTON_L1},
        CTRL_BUTTON_R1: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 10, CTRL_NETWORK_LABEL: NWK_BUTTON_R1},
        CTRL_BUTTON_L3: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 7, CTRL_NETWORK_LABEL: NWK_BUTTON_L3},
        CTRL_BUTTON_R3: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 8, CTRL_NETWORK_LABEL: NWK_BUTTON_R3},

        CTRL_BUTTON_START: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 5, CTRL_NETWORK_LABEL: NWK_BUTTON_START},
        CTRL_BUTTON_BACK: {CTRL_VALUE: manager.Value('i', 0), CTRL_FLAG: manager.Value('i', 0), CTRL_JOYSTICK_FIELD: 6, CTRL_NETWORK_LABEL: NWK_BUTTON_BACK}
    }

def generate_state_map():
    return {
        STATE_HEAD_LIGHTS: 0,
        STATE_INDICATOR_MODE: INDICATOR_STATE_OFF,
        STATE_STEERING_TARGET: 0.0,  # Arduino analog input 1
        STATE_THROTTLE_REMOTE: 0.0,  # Arduino analog input 2
        STATE_GEAR: GEAR_PARK,  # Arduino serial bits 1 and 2 (MSB)
        STATE_HORN: 0,  # Arduino serial bit 3
        STATE_OTHER_LIGHTS: 0,  # Arduino serial bit 4
        STATE_DRIVING_MODE: DRIVING_MODE_REMOTE,  # Arduino serial bit 5,
        STATE_BRAKE_REMOTE: 0.0,
        STATE_STOP_THROTTLE: 0,
        STATE_FRONT_RIGHT_RPM: 0.0,
        STATE_FRONT_LEFT_RPM: 0.0,
        STATE_REAR_RIGHT_RPM: 0.0,
        STATE_REAR_LEFT_RPM: 0.0,
        # Arduino analog outputs
        STATE_STEERING_CURRENT: 0.0,
        STATE_THROTTLE_CURRENT: 0.0,
        STATE_BRAKE_CURRENT: 0.0,
        STATE_CUSTOM_INPUT: 0.0,

        STATE_IMU_READING_HEADING: 0.0,
        STATE_IMU_READING_PITCH: 0.0,
        STATE_IMU_READING_ROLL: 0.0
    }


def generate_multiprocess_state_map():
    return {
        STATE_HEAD_LIGHTS: manager.Value('i', 0),
        STATE_INDICATOR_MODE: manager.Value('i', INDICATOR_STATE_OFF),
        STATE_STEERING_TARGET: manager.Value('d', 0.0),  # Arduino analog input 1
        STATE_THROTTLE_REMOTE: manager.Value('d', 0.0),  # Arduino analog input 2
        STATE_GEAR: manager.Value('i', GEAR_PARK),  # Arduino serial bits 1 and 2 (MSB)
        STATE_HORN: manager.Value('i', 0),  # Arduino serial bit 3
        STATE_OTHER_LIGHTS: manager.Value('i', 0),  # Arduino serial bit 4
        STATE_DRIVING_MODE: manager.Value('i', DRIVING_MODE_REMOTE),  # Arduino serial bit 5,
        STATE_BRAKE_REMOTE: manager.Value('d', 0.0),
        STATE_STOP_THROTTLE: manager.Value('i', 0),
        STATE_FRONT_RIGHT_RPM: manager.Value('d', 0.0),
        STATE_FRONT_LEFT_RPM: manager.Value('d', 0.0),
        STATE_REAR_RIGHT_RPM: manager.Value('d', 0.0),
        STATE_REAR_LEFT_RPM: manager.Value('d', 0.0),
        # Arduino analog outputs
        STATE_STEERING_CURRENT: manager.Value('d', 0.0),
        STATE_THROTTLE_CURRENT: manager.Value('d', 0.0),
        STATE_BRAKE_CURRENT: manager.Value('d', 0.0),
        STATE_CUSTOM_INPUT: manager.Value('d',0.0),

        STATE_IMU_READING_HEADING: manager.Value('d', 0.0),
        STATE_IMU_READING_PITCH: manager.Value('d', 0.0),
        STATE_IMU_READING_ROLL: manager.Value('d', 0.0)
    }


def convert_to_local_map(map):
    new_map = {}
    for key in map:
        new_map[key] = map[key].value
