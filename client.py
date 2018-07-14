import sys
import socket
import time
import pigpio
import os
from threading import Thread
import subprocess
import serial
import logging
import coloredlogs
import json

import Adafruit_PCA9685

from SC.util.control import *
from SC.light import light_patterns, apa102
from SC.net.serialpacket import *
from SC.servo import servo
from SC.arduino import arduino_serial
from SC.imu import imu

LOG_LEVEL = 'INFO'
STOP_THROTTLE_THRESHOLD = 0.05

PWM_I2C_ADDRESS = 0x40
PWM_FREQUENCY = 50

DISABLE_SERVO = True

# Controller values
ctrl_map = generate_control_map()

local_ctrl_map = {
    LINPUT_DRIVE_GEAR: {CTRL_FLAG: False, CTRL_PIN: 18},
    LINPUT_REVERSE_GEAR: {CTRL_FLAG: False, CTRL_PIN: 23},
    LINPUT_PARK_GEAR: {CTRL_FLAG: False, CTRL_PIN: 24},
    LINPUT_RIGHT_INDICATOR: {CTRL_FLAG: False, CTRL_PIN: 25},
    LINPUT_LEFT_INDICATOR: {CTRL_FLAG: False, CTRL_PIN: 7},
    LINPUT_HORN: {CTRL_FLAG: False, CTRL_PIN: 12},
    LINPUT_OTHER_LIGHTS: {CTRL_FLAG: False, CTRL_PIN: 16},
    LINPUT_REMOTE_DRIVE: {CTRL_FLAG: False, CTRL_PIN: 20}
}

state_map = {
    STATE_HEAD_LIGHTS: 0,
    STATE_INDICATOR_MODE: INDICATOR_STATE_OFF,
    STATE_STEERING_TARGET: 0.0,     # Arduino analog input 1
    STATE_THROTTLE_REMOTE: 0.0,     # Arduino analog input 2
    STATE_GEAR: GEAR_PARK,             # Arduino serial bits 1 and 2 (MSB)
    STATE_HORN: 0,                  # Arduino serial bit 3
    STATE_OTHER_LIGHTS: 0,          # Arduino serial bit 4
    STATE_DRIVING_MODE: DRIVING_MODE_REMOTE,   # Arduino serial bit 5,
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
    STATE_IMU_DATA: None
}

# Thread stop signal
stop_signal = 0

# Initialize the library and the strip
led_strip = apa102.APA102(num_led=144, global_brightness=20, mosi = 10, sclk = 11,
                                  order='rgb', max_speed_hz=4000000)

led_threads = [{'thread': None, 'stop_signal': 0}]

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flushInput()

imu_module = imu.IMU()

if not DISABLE_SERVO:
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685(PWM_I2C_ADDRESS)
    # Set frequency to 50hz, good for servos.
    pwm.set_pwm_freq(PWM_FREQUENCY)


class CarClient():
    def __init__(self):
        self.INPUT_PINS = [5, 6, 13, 19, 18, 23, 24, 25, 7, 12, 16, 20]
        self.OUTPUT_PINS = []
        self.ALT0_PINS = []

        self.RPM_MAP = {
            5: STATE_FRONT_RIGHT_RPM,
            6: STATE_FRONT_LEFT_RPM,
            13: STATE_REAR_RIGHT_RPM,
            19: STATE_REAR_LEFT_RPM
        }

        self.pi = pigpio.pi()

        # Car state values
        self.throttle = 0
        self.reverse = 0
        self.bias = 0
        self.left = 0
        self.right = 0

        # RPM values
        self.last = [None] * 32
        self.callbacks = []

        self.lrpm = 0
        self.rrpm = 0
        self.lastrpm = 0

        # Thread Variables
        self.threads = []

        # Serial Variables
        #self.serial_client = None
        #self.serial_port = '/dev/ttyACM0'
        #self.serial_baudrate = 115200

        # Network Variables
        self.server_socket = None
        self.server_host = '192.168.1.75'  # The remote host
        self.server_port = 50008  # The same port as used by the server

        # Video Streaming Variables
        self.video_process = None
        self.video_port = 4200

    def initialize(self):
        global led_strip
        global imu_module
        global local_ctrl_map

        # Clear LED strip
        led_strip.clear_strip()

        light_patterns.default_lights(led_strip, 1)

        if not self.pi.connected:
            # Check that the PiGPIO Daemon is running
            #TODO: Replace with better handling
            exit()

        # Pin Setup
        for pin in self.INPUT_PINS:
            self.pi.set_mode(pin, pigpio.INPUT)
        for pin in self.OUTPUT_PINS:
            self.pi.set_mode(pin, pigpio.OUTPUT)
        for pin in self.ALT0_PINS:
            self.pi.set_mode(pin, pigpio.ALT0)
        for pin in self.RPM_MAP:
            self.callbacks.append(self.pi.callback(pin, pigpio.RISING_EDGE, self.rpm_callback))
        for key in local_ctrl_map:
            self.callbacks.append(self.pi.callback(local_ctrl_map[key][CTRL_PIN], pigpio.EITHER_EDGE, self.pin_callback))

        # Serial (Arduino) Setup
        #self.serial_client = serial.Serial(self.serial_port, self.serial_baudrate)
        #self.serial_client.flushInput()

        # IMU Setup
        imu_module.initialize()
        imu_module.register_callback(imu_callback)

        # Network Setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.connect((self.server_host, self.server_port))
        logging.info(str(self.server_socket))

        # def thread_2( threadName ):
        #   os.system('raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 2000000 -o udp://192.168.1.71:4200')

        # Video Streaming Setup
        #TODO: FIX H264 STREAMING
        #self.video_process = subprocess.Popen(['raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 10000000 -o -'
        #                                       ' |  gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! gdppay'
        #                                       ' ! udpsink host='+self.server_host+' port='+str(self.video_port)], shell=True)
        os.system('killall raspivid')
        self.video_process = subprocess.Popen(['raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 10000000 -o udp://'+self.server_host+':'+str(self.video_port)], shell=True)

        # Processes Setup
        self.threads.append(Thread(target=run_controller_thread, args=()))
        self.threads.append(Thread(target=run_arduino_thread, args=()))
        self.threads.append(Thread(target=imu_module.run, args=()))

    def run(self):
        try:
            for t in self.threads:
                t.start()
            self.run_network_thread()
        except Exception, e:
            logging.error(e)
            self.shutdown()
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        global stop_signal
        global led_strip
        global ser

        logging.info("Tidying up")

        logging.info("Clearing Light Strip...")
        led_strip.clear_strip()
        led_strip.cleanup()

        # Disable PWM controller.
        logging.info("Disabling PWM Controllers...")

        # Ground digital output pins
        logging.info("Grounding output pins...")
        for pin in self.OUTPUT_PINS:
            self.pi.write(pin, 0)

        logging.info("Closing Serial port...")
        ser.close()

        # Remove all pin callbacks
        logging.info("Removing all pin callbacks...")
        for c in self.callbacks:
            c.cancel()

        # Stopping IMU thread
        logging.info("Stopping IMU...")
        imu_module.stop()

        # Stop PiGPIO daemon
        logging.info("Stopping PiGPIO daemon...")
        self.pi.stop()

        # Terminate video streaming process
        logging.info("Terminating video streaming process...")
        self.video_process.terminate()

        # Stop all threads
        logging.info("Stopping all threads...")
        stop_signal = 1
        for t in self.threads:
            if t.isAlive():
                t.join()
                logging.info('thread '+repr(t)+' joined.')

    def rpm_callback(self, pin, level, tick):
        if self.last[pin] is not None:
            diff = time.time() - self.last[pin]
            #TODO: Check Diff value
            if diff > 0.01 and level == 1 and pin in self.RPM_MAP:
                value = 3 / diff
                #setattr(self, self.RPM_MAP[pin], value)
                state_map[self.RPM_MAP[pin]] = value
                logging.debug("PIN={} LEVEL={} TIME_DIFF={} {}={}".format(pin, level, diff, self.RPM_MAP[pin], value))

        self.last[pin] = time.time()

    def pin_callback(self, pin, level, tick):
        global local_ctrl_map

        for key in local_ctrl_map:
            if local_ctrl_map[key][CTRL_PIN] == pin:
                local_ctrl_map[key][CTRL_VALUE] = level
                local_ctrl_map[key][CTRL_FLAG] = 1
                logging.debug(key + ' value updated to '+str(level)+' from source: '+str(pin))
                if key == LINPUT_REMOTE_DRIVE:
                    state_map[STATE_DRIVING_MODE] = DRIVING_MODE_LOCAL if level == 0 else DRIVING_MODE_REMOTE
                else:
                    if state_map[STATE_DRIVING_MODE] == DRIVING_MODE_LOCAL:
                        if key == LINPUT_DRIVE_GEAR:
                            state_map[STATE_GEAR] = GEAR_DRIVE if level == 1 else GEAR_NEUTRAL
                            if level == 1:
                                logging.info('Setting Gear to Drive')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == LINPUT_REVERSE_GEAR:
                            state_map[STATE_GEAR] = GEAR_REVERSE if level == 1 else GEAR_NEUTRAL
                            if level == 1:
                                logging.info('Setting Gear to Reverse')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == LINPUT_PARK_GEAR:
                            state_map[STATE_GEAR] = GEAR_PARK if level == 1 else GEAR_NEUTRAL
                            if level == 1:
                                logging.info('Setting Gear to Park')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == LINPUT_RIGHT_INDICATOR:
                            logging.info('Toggling Right Indicator')
                            handle_indicator_event(INDICATOR_STATE_RIGHT)
                        elif key == LINPUT_LEFT_INDICATOR:
                            logging.info('Toggling Left Indicator')
                            handle_indicator_event(INDICATOR_STATE_LEFT)
                    if key == LINPUT_OTHER_LIGHTS:
                        state_map[STATE_OTHER_LIGHTS] = int(level)
                    if key == LINPUT_HORN:
                        state_map[STATE_HORN] = int(level)

    def run_network_thread(self):
        global ctrl_map
        global state_map
        global stop_signal

        logging.info("Ready to accept data.")

        while stop_signal == 0:
            packet = read_packet(self.server_socket)
            #logging.info('Received a packet.')
            data = packet.data
            type = packet.type
            if type == TYPE_CMD_UPDATE:
                write_packet(self.server_socket, Packet(TYPE_ACK, ''))
                for reading in data.split('\n'):
                    if reading != "":
                        id, value = reading.split("_")
                        for key in ctrl_map:
                            if ctrl_map[key][CTRL_NETWORK_LABEL] == id:
                                ctrl_map[key][CTRL_VALUE] = float(value)
                                ctrl_map[key][CTRL_FLAG] = True
                        logging.debug('Processed: '+repr(reading))
            elif type == TYPE_CMD_READ:
                write_packet(self.server_socket, Packet(TYPE_VALUE, json.dumps(state_map)))
                logging.debug('Processed READ Command')
            else:
                write_packet(self.server_socket, Packet(TYPE_ACK, ''))


def handle_indicator_event(mode):
    global led_strip
    global led_threads
    state_map[STATE_INDICATOR_MODE] = mode if state_map[STATE_INDICATOR_MODE] != mode else INDICATOR_STATE_OFF
    led_threads[0]['stop_signal'] = 1
    led_threads.pop(0)
    led_threads.append({'stop_signal': 0})
    if state_map[STATE_INDICATOR_MODE] == mode:
        logging.info(mode+': ON')
        led_threads[0]['thread'] = Thread(target=light_patterns.indicator_lights,
                                          args=(led_strip, mode, 'single', led_threads[0]))
        led_threads[0]['thread'].start()


def imu_callback(data):
    global state_map
    state_map[STATE_IMU_DATA] = data
    logging.debug('IMU Data:' + str(data))
    time.sleep(0.25)


def set_arduino_flags(gear, horn, other_lights, driving_mode):
    reverse_flags = (0b11 if gear == GEAR_REVERSE else 0) << 6
    horn_flag = (0b1 if horn == 1 else 0) << 5
    other_lights_flag = (0b1 if other_lights == 1 else 0) << 4
    driving_mode_flag = (0b1 if driving_mode == DRIVING_MODE_REMOTE else 0) << 3
    byte_input = reverse_flags | horn_flag | other_lights_flag | driving_mode_flag
    return byte_input


def run_arduino_thread():
    global state_map
    global stop_signal
    global ser
    ser.flushInput()
    while stop_signal == 0:
        try:
            # Write to Arduino
            steering_target = float((state_map[STATE_STEERING_TARGET]+0.5)*1023)
            throttle_remote = float(state_map[STATE_THROTTLE_REMOTE]*1023)
            byte_input = set_arduino_flags(state_map[STATE_GEAR], state_map[STATE_HORN], state_map[STATE_OTHER_LIGHTS], state_map[STATE_DRIVING_MODE])
            arduino_serial.writeFrame(ser, steering_target, throttle_remote, byte_input)
            # Read from Arduino
            steering, throttle, brake, custom_input = arduino_serial.readFrame(ser)
            state_map[STATE_STEERING_CURRENT] = float((steering * 1.0 / 1023) - 0.5)
            state_map[STATE_THROTTLE_CURRENT] = float(throttle * 1.0 / 1023) if state_map[STATE_STOP_THROTTLE] == 0 else 0.0
            state_map[STATE_BRAKE_CURRENT] = float(brake * 1.0 / 1023)
            state_map[STATE_CUSTOM_INPUT] = float(custom_input * 1.0 / 1023)
        except Exception, e:
            logging.warning(e.message)
            time.sleep(0.1)


def run_controller_thread():
    global ctrl_map
    global state_map
    global stop_signal
    global led_strip
    global led_threads

    while stop_signal == 0:
        #TODO: Check effect of this thread on timing
        #logging.debug("Control loop.")
        try:
            time.sleep(0.1)

            for key in ctrl_map:
                if ctrl_map[key][CTRL_FLAG]:
                    value = ctrl_map[key][CTRL_VALUE]

                    # Camera Pan Servo
                    if key == CTRL_R_THUMB_X:
                        if not DISABLE_SERVO:
                            servo_value = int((-value*204)+307)
                            servo.set_servo_pulse(pwm, 5, int(servo_value))

                    # Camera Tilt Servo
                    elif key == CTRL_R_THUMB_Y:
                        if not DISABLE_SERVO:
                            servo_value = int((-value*204)+307)
                            servo.set_servo_pulse(pwm, 4, int(servo_value))

                    if state_map[STATE_DRIVING_MODE] == DRIVING_MODE_REMOTE:

                        # Brake Command
                        if key == CTRL_L_TRIGGER:
                            #make sure to prevent any throttling during braking
                            if not DISABLE_SERVO:
                                state_map[STATE_BRAKE_REMOTE] = value
                                if value > STOP_THROTTLE_THRESHOLD:
                                    state_map[STATE_STOP_THROTTLE] = 1
                                else:
                                    state_map[STATE_STOP_THROTTLE] = 0
                                servo_value = int((value * 204) + 307)
                                servo.set_servo_pulse(pwm, 0, servo_value)
                                servo.set_servo_pulse(pwm, 1, servo_value)
                                servo.set_servo_pulse(pwm, 2, int(servo_value * 0.8))
                                servo.set_servo_pulse(pwm, 3, int(servo_value * 0.8))

                        # Throttle Command
                        elif key == CTRL_R_TRIGGER:
                            state_map[STATE_THROTTLE_REMOTE] = value

                        # Steering Command
                        elif key == CTRL_R_THUMB_X:
                            state_map[STATE_STEERING_TARGET] = value

                        # Headlights on
                        elif key == CTRL_BUTTON_Y:
                            if value == 1:
                                logging.info('Toggling Headlights')
                                state_map[STATE_HEAD_LIGHTS] = int(1 - state_map[STATE_HEAD_LIGHTS])
                                logging.info('Head Lights ' + ('on' if state_map[STATE_HEAD_LIGHTS] == 1 else 'off') + '.')
                                light_patterns.head_lights(led_strip, state_map[STATE_HEAD_LIGHTS])

                        # Other Lights on
                        elif key == CTRL_BUTTON_B:
                            if value == 1:
                                logging.info('Toggling Other Lights')
                                state_map[STATE_OTHER_LIGHTS] = int(1 - state_map[STATE_OTHER_LIGHTS])
                                logging.info('Other Lights ' + ('on' if state_map[STATE_OTHER_LIGHTS] == 1 else 'off') + '.')
                                light_patterns.head_lights(led_strip, state_map[STATE_OTHER_LIGHTS])

                        # Horn (Hold)
                        elif key == CTRL_BUTTON_X:
                            state_map[STATE_HORN] = value
                            logging.info('Horn ' + ('on' if state_map[STATE_HORN] == 1 else 'off') + '.')

                        # Park Gear
                        elif key == CTRL_BUTTON_DPAD_LEFT:
                            if value == 1:
                                logging.info('Setting Gear to Park')
                                state_map[STATE_GEAR] = GEAR_PARK
                                light_patterns.reverse_lights(led_strip, 0)

                        # Neutral Gear
                        elif key == CTRL_BUTTON_DPAD_RIGHT:
                            if value == 1:
                                logging.info('Setting Gear to Neutral')
                                state_map[STATE_GEAR] = GEAR_NEUTRAL
                                light_patterns.reverse_lights(led_strip, 0)

                        # Drive Gear
                        elif key == CTRL_BUTTON_DPAD_UP:
                            if value == 1:
                                logging.info('Setting Gear to Drive')
                                state_map[STATE_GEAR] = GEAR_DRIVE
                                light_patterns.reverse_lights(led_strip, 0)

                        # Reverse Gear
                        elif key == CTRL_BUTTON_DPAD_DOWN:
                            if value == 1:
                                logging.info('Setting Gear to Reverse')
                                state_map[STATE_GEAR] = GEAR_REVERSE
                                light_patterns.reverse_lights(led_strip, 1)

                        #TODO: Detect hold mode (or not)
                        # Left Indicator Button Input
                        elif key == CTRL_BUTTON_L1:
                            if value == 1:
                                logging.info('Toggling Left Indicator')
                                handle_indicator_event(INDICATOR_STATE_LEFT)
                        # Right Indicator Button Input
                        elif key == CTRL_BUTTON_R1:
                            if value == 1:
                                logging.info('Toggling Right Indicator')
                                handle_indicator_event(INDICATOR_STATE_RIGHT)

                        # Warn Indicator Button Input
                        elif key == CTRL_BUTTON_BACK:
                            if value == 1:
                                logging.info('Toggling Warn Indicator')
                                handle_indicator_event(INDICATOR_STATE_WARN)
                        else:
                            pass

                    ctrl_map[key][CTRL_FLAG] = False
        except Exception, e:
            logging.error(e)
    led_threads[0]['stop_signal'] = 1
    if led_threads[0]['thread'] is not None:
        led_threads[0]['thread'].join()

if __name__ == "__main__":
    for arg in sys.argv[1:]:
        key, val = arg.split('=')
        if key == 'LOG_LEVEL':
            LOG_LEVEL = val

    coloredlogs.install(level=LOG_LEVEL, fmt='%(asctime)s %(hostname)s %(name)s[%(process)d] [%(threadName)s] %(levelname)s %(message)s')
    client = CarClient()
    client.initialize()
    client.run()
