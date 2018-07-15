# Remote control constants
CTRL_BUTTON_BACK = 'button_back'
CTRL_BUTTON_START = 'button_start'
CTRL_BUTTON_R3 = 'button_r3'
CTRL_BUTTON_L3 = 'button_l3'
CTRL_BUTTON_R1 = 'button_r1'
CTRL_BUTTON_L1 = 'button_l1'
CTRL_BUTTON_Y = 'button_y'
CTRL_BUTTON_X = 'button_x'
CTRL_BUTTON_B = 'button_b'
CTRL_BUTTON_A = 'button_a'
CTRL_BUTTON_DPAD_RIGHT = 'button_dpad_right'
CTRL_BUTTON_DPAD_LEFT = 'button_dpad_left'
CTRL_BUTTON_DPAD_DOWN = 'button_dpad_down'
CTRL_BUTTON_DPAD_UP = 'button_dpad_up'
CTRL_R_THUMB_X = 'r_thumb_x'
CTRL_R_THUMB_Y = 'r_thumb_y'
CTRL_L_THUMB_X = 'l_thumb_x'
CTRL_L_THUMB_Y = 'l_thumb_y'
CTRL_L_TRIGGER = 'l_trigger'
CTRL_R_TRIGGER = 'r_trigger'

CTRL_FLAG = 'flag'
CTRL_JOYSTICK_FIELD = 'joystick_field'
CTRL_NETWORK_LABEL = 'nw_label'
CTRL_VALUE = 'value'
CTRL_PIN = 'pin'

# Network constants
NWK_BUTTON_BACK = 'bback'
NWK_BUTTON_START = 'bstart'
NWK_BUTTON_R3 = 'br3'
NWK_BUTTON_L3 = 'bl3'
NWK_BUTTON_R1 = 'br1'
NWK_BUTTON_L1 = 'bl1'
NWK_BUTTON_Y = 'by'
NWK_BUTTON_X = 'bx'
NWK_BUTTON_B = 'bb'
NWK_BUTTON_A = 'ba'
NWK_DPAD_RIGHT = 'bdr'
NWK_DPAD_LEFT = 'bdl'
NWK_DPAD_DOWN = 'bdd'
NWK_DPAD_UP = 'bdu'
NWK_L_THUMB_X = 'lx'
NWK_L_THUMB_Y = 'ly'
NWK_R_THUMB_X = 'rx'
NWK_R_THUMB_Y = 'ry'
NWK_L_TRIGGER = 'l'
NWK_R_TRIGGER = 'r'

# Joystick constants
INPUT_RIGHT_THUMB_Y = 'r_thumb_y'
INPUT_RIGHT_THUMB_X = 'r_thumb_x'
INPUT_LEFT_THUMB_Y = 'l_thumb_y'
INPUT_LEFT_THUMB_X = 'l_thumb_x'
INPUT_RIGHT_TRIGGER = 'right_trigger'
INPUT_LEFT_TRIGGER = 'left_trigger'
INPUT_BUTTONS = 'buttons'

# Local input constants
LINPUT_REMOTE_DRIVE = 'remote_drive'
LINPUT_OTHER_LIGHTS = 'other_lights'
LINPUT_HORN = 'horn'
LINPUT_LEFT_INDICATOR = 'left_indicator'
LINPUT_RIGHT_INDICATOR = 'right_indicator'
LINPUT_PARK_GEAR = 'park_gear'
LINPUT_REVERSE_GEAR = 'reverse_gear'
LINPUT_DRIVE_GEAR = 'drive_gear'

# State constants
STATE_BRAKE_CURRENT = 'brake_current'
STATE_THROTTLE_CURRENT = 'throttle_current'
STATE_STEERING_CURRENT = 'steering_current'
STATE_REAR_LEFT_RPM = 'rl_rpm'
STATE_REAR_RIGHT_RPM = 'rr_rpm'
STATE_FRONT_LEFT_RPM = 'fl_rpm'
STATE_FRONT_RIGHT_RPM = 'fr_rpm'
STATE_STOP_THROTTLE = 'stop_throttle'
STATE_BRAKE_REMOTE = 'brake_remote'
STATE_DRIVING_MODE = 'driving_mode'
STATE_OTHER_LIGHTS = 'other_lights'
STATE_HORN = 'horn'
STATE_GEAR = 'gear'
STATE_THROTTLE_REMOTE = 'throttle_remote'
STATE_STEERING_TARGET = 'steering_target'
STATE_INDICATOR_MODE = 'indicator_mode'
STATE_HEAD_LIGHTS = 'head_lights'
STATE_CUSTOM_INPUT = 'custom_input'
STATE_IMU_READING_HEADING = 'imu_reading_heading'
STATE_IMU_READING_PITCH = 'imu_reading_pitch'
STATE_IMU_READING_ROLL = 'imu_reading_roll'

# Driving mode constants
DRIVING_MODE_LOCAL = 'local'
DRIVING_MODE_REMOTE = 'remote'

MAP_DRIVING_MODE = {
    DRIVING_MODE_LOCAL: 'Local',
    DRIVING_MODE_REMOTE: 'Remote'
}

# Gear constants
GEAR_DRIVE = 'drive'
GEAR_NEUTRAL = 'neutral'
GEAR_REVERSE = 'reverse'
GEAR_PARK = 'park'

MAP_GEAR = {
    GEAR_DRIVE: 'Drive',
    GEAR_NEUTRAL: 'Neutral',
    GEAR_REVERSE: 'Reverse',
    GEAR_PARK: 'Park'
}

# Indicator constants
INDICATOR_STATE_RIGHT = 'right'
INDICATOR_STATE_LEFT = 'left'
INDICATOR_STATE_OFF = 'off'
INDICATOR_STATE_WARN = 'warn'

MAP_INDICATOR = {
    INDICATOR_STATE_RIGHT: 'Right',
    INDICATOR_STATE_LEFT: 'Left',
    INDICATOR_STATE_OFF: 'Off',
    INDICATOR_STATE_WARN: 'Warn'
}

# Servo Constants
SERVO_PIN_RL = 3
SERVO_PIN_RR = 2
SERVO_PIN_FL = 1
SERVO_PIN_FR = 0
SERVO_PIN_CAM_X = 4
SERVO_PIN_CAM_Y = 5