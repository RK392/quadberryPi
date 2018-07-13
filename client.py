import sys

sys.path.insert(0, './APA102_Pi')

import socket
import time
import pigpio
import os
#import PID
from threading import Thread
import subprocess
import serial
import light_patterns
from serialpacket import *
import servo
from imu import IMU
import logging
import coloredlogs
import apa102
import arduino_serial
import json
import imu
import Adafruit_PCA9685

#imu = IMU()
#imu.initialize()
#thread.start(imu.run)

LOG_LEVEL = 'INFO'
STOP_THROTTLE_THRESHOLD = 0.05

PWM_I2C_ADDRESS = 0x40
PWM_FREQUENCY = 50

DISABLE_SERVO = False

CTRL_FLAG = 'flag'
CTRL_JOYSTICK_FIELD = 'joystick_field'
CTRL_NETWORK_LABEL = 'nw_label'
CTRL_VALUE = 'value'

CTRL_PIN = 'pin'


# Controller values
ctrl_map = {
    'r_trigger': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'right_trigger', CTRL_NETWORK_LABEL: 'r'},
    'l_trigger': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'left_trigger', CTRL_NETWORK_LABEL: 'l'},
    'l_thumb_y': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'l_thumb_y', CTRL_NETWORK_LABEL: 'ly'},
    'l_thumb_x': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'l_thumb_x', CTRL_NETWORK_LABEL: 'lx'},
    'r_thumb_y': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'r_thumb_y', CTRL_NETWORK_LABEL: 'y'},
    'r_thumb_x': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 'r_thumb_x', CTRL_NETWORK_LABEL: 'x'},

    'button_dpad_up': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 1, CTRL_NETWORK_LABEL: 'bdu'},
    'button_dpad_down': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 2, CTRL_NETWORK_LABEL: 'bdd'},
    'button_dpad_left': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 3, CTRL_NETWORK_LABEL: 'bdl'},
    'button_dpad_right': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 4, CTRL_NETWORK_LABEL: 'bdr'},

    'button_a':  {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 13, CTRL_NETWORK_LABEL: 'ba'},
    'button_b':  {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 14, CTRL_NETWORK_LABEL: 'bb'},
    'button_x':  {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 15, CTRL_NETWORK_LABEL: 'bx'},
    'button_y':  {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 16, CTRL_NETWORK_LABEL: 'by'},

    'button_l1': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 9, CTRL_NETWORK_LABEL: 'bl1'},
    'button_r1': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 10, CTRL_NETWORK_LABEL: 'br1'},
    'button_l3': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 7, CTRL_NETWORK_LABEL: 'bl3'},
    'button_r3': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 8, CTRL_NETWORK_LABEL: 'br3'},

    'button_start': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 5, CTRL_NETWORK_LABEL: 'bstart'},
    'button_back': {CTRL_FLAG: False, CTRL_JOYSTICK_FIELD: 6, CTRL_NETWORK_LABEL: 'bback'}
}

local_ctrl_map = {
    'drive_gear': {CTRL_FLAG: False, CTRL_PIN: 18},
    'reverse_gear': {CTRL_FLAG: False, CTRL_PIN: 23},
    'park_gear': {CTRL_FLAG: False, CTRL_PIN: 24},
    'right_indicator': {CTRL_FLAG: False, CTRL_PIN: 25},
    'left_indicator': {CTRL_FLAG: False, CTRL_PIN: 7},
    'horn': {CTRL_FLAG: False, CTRL_PIN: 12},
    'other_lights': {CTRL_FLAG: False, CTRL_PIN: 16},
    'remote_drive': {CTRL_FLAG: False, CTRL_PIN: 20}
}

state_map = {
    'head_lights': 0,
    'indicator_mode': 'off',
    'steering_target': 0.0,     # Arduino analog input 1
    'throttle_remote': 0.0,     # Arduino analog input 2
    'gear': 'park',             # Arduino serial bits 1 and 2 (MSB)
    'horn': 0,                  # Arduino serial bit 3
    'other_lights': 0,          # Arduino serial bit 4
    'driving_mode': 'remote',   # Arduino serial bit 5,
    'brake_remote': 0.0,
    'stop_throttle': 0,
    'fr_rpm': 0.0,
    'fl_rpm': 0.0,
    'rr_rpm': 0.0,
    'rl_rpm': 0.0,
    # Arduino analog outputs
    'steering_current': 0.0,
    'throttle_current': 0.0,
    'brake_current': 0.0,
    'imu_data': None
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
    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(PWM_FREQUENCY)

class CarClient():
    def __init__(self):
        self.INPUT_PINS = [5, 6, 13, 19, 18, 23, 24, 25, 7, 12, 16, 20]
        self.OUTPUT_PINS = []
        self.ALT0_PINS = []

        self.RPM_MAP = {
            5: 'fr_rpm',
            6: 'fl_rpm',
            13: 'rr_rpm',
            19: 'rl_rpm'
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
        self.server_host = '192.168.1.66'  # The remote host
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
        #TODO: FIX H264 STREAaMING
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
                if key == 'remote_drive':
                    state_map['driving_mode'] = 'local' if level == 0 else 'remote'
                else:
                    if state_map['driving_mode'] == 'local':
                        if key == 'drive_gear':
                            state_map['gear'] = 'drive' if level == 1 else 'neutral'
                            if level == 1:
                                logging.info('Setting Gear to Drive')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == 'reverse_gear':
                            state_map['gear'] = 'reverse' if level == 1 else 'neutral'
                            if level == 1:
                                logging.info('Setting Gear to Reverse')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == 'park_gear':
                            state_map['gear'] = 'park' if level == 1 else 'neutral'
                            if level == 1:
                                logging.info('Setting Gear to Park')
                            else:
                                logging.info('Setting Gear to Neutral')
                        elif key == 'right_indicator':
                            logging.info('Toggling Right Indicator')
                            handle_indicator_event('right')
                        elif key == 'left_indicator':
                            logging.info('Toggling Left Indicator')
                            handle_indicator_event('left')
                    if key == 'other_lights':
                        state_map['other_lights'] = int(level)
                    if key == 'horn':
                        state_map['horn'] = int(level)

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
    state_map['indicator_mode'] = mode if state_map['indicator_mode'] != mode else 'off'
    led_threads[0]['stop_signal'] = 1
    led_threads.pop(0)
    led_threads.append({'stop_signal': 0})
    if state_map['indicator_mode'] == mode:
        logging.info(mode+': ON')
        led_threads[0]['thread'] = Thread(target=light_patterns.indicator_lights,
                                          args=(led_strip, mode, 'single', led_threads[0]))
        led_threads[0]['thread'].start()


def imu_callback(data):
    global state_map
    state_map['imu_data'] = data
    logging.debug('IMU Data:' + str(data))
    time.sleep(0.25)


def set_arduino_flags(gear, horn, other_lights, driving_mode):
    reverse_flags = (0b11 if gear == 'reverse' else 0) << 6
    horn_flag = (0b1 if horn == 1 else 0) << 5
    other_lights_flag = (0b1 if other_lights == 1 else 0) << 4
    driving_mode_flag = (0b1 if driving_mode == 'remote' else 0) << 3
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
            steering_target = float((state_map['steering_target']+0.5)*1023)
            throttle_remote = float(state_map['throttle_remote']*1023)
            byte_input = set_arduino_flags(state_map['gear'], state_map['horn'], state_map['other_lights'], state_map['driving_mode'])
            arduino_serial.writeFrame(ser, steering_target, throttle_remote, byte_input)
            # Read from Arduino
            steering, throttle, brake, custom_input = arduino_serial.readFrame(ser)
            state_map['steering_current'] = float((steering * 1.0 / 1023) - 0.5)
            state_map['throttle_current'] = float(throttle * 1.0 / 1023) if state_map['stop_throttle'] == 0 else 0.0
            state_map['brake_current'] = float(brake * 1.0 / 1023)
            state_map['custom_input'] = float(custom_input * 1.0 / 1023)
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
                    if key == 'r_thumb_x':
                        if not DISABLE_SERVO:
                            servo_value = int((-value*204)+307)
                            servo.set_servo_pulse(pwm, 5, int(servo_value))

                    # Camera Tilt Servo
                    elif key == 'r_thumb_y':
                        if not DISABLE_SERVO:
                            servo_value = int((-value*204)+307)
                            servo.set_servo_pulse(pwm, 4, int(servo_value))

                    if state_map['driving_mode'] == 'remote':

                        # Brake Command
                        if key == 'l_trigger':
                            #make sure to prevent any throttling during braking
                            if not DISABLE_SERVO:
                                state_map['brake_remote'] = value
                                if value > STOP_THROTTLE_THRESHOLD:
                                    state_map['stop_throttle'] = 1
                                else:
                                    state_map['stop_throttle'] = 0
                                servo_value = int((value * 204) + 307)
                                servo.set_servo_pulse(pwm, 0, servo_value)
                                servo.set_servo_pulse(pwm, 1, servo_value)
                                servo.set_servo_pulse(pwm, 2, int(servo_value*0.8))
                                servo.set_servo_pulse(pwm, 3, int(servo_value*0.8))

                        # Throttle Command
                        elif key == 'r_trigger':
                            state_map['throttle_remote'] = value

                        # Steering Command
                        elif key == 'l_thumb_x':
                            state_map['steering_target'] = value

                        # Headlights on
                        elif key == 'button_y':
                            if value == 1:
                                logging.info('Toggling Headlights')
                                state_map['head_lights'] = int(1 - state_map['head_lights'])
                                logging.info('Head Lights ' + ('on' if state_map['head_lights'] == 1 else 'off') + '.')
                                light_patterns.head_lights(led_strip, state_map['head_lights'])

                        # Other Lights on
                        elif key == 'button_b':
                            if value == 1:
                                logging.info('Toggling Other Lights')
                                state_map['other_lights'] = int(1 - state_map['other_lights'])
                                logging.info('Other Lights ' + ('on' if state_map['other_lights'] == 1 else 'off') + '.')
                                light_patterns.head_lights(led_strip, state_map['other_lights'])

                        # Horn (Hold)
                        elif key == 'button_x':
                            state_map['horn'] = value
                            logging.info('Horn ' + ('on' if state_map['horn'] == 1 else 'off') + '.')

                        # Park Gear
                        elif key == 'button_dpad_left':
                            if value == 1:
                                logging.info('Setting Gear to Park')
                                state_map['gear'] = 'park'
                                light_patterns.reverse_lights(led_strip, 0)

                        # Neutral Gear
                        elif key == 'button_dpad_right':
                            if value == 1:
                                logging.info('Setting Gear to Neutral')
                                state_map['gear'] = 'neutral'
                                light_patterns.reverse_lights(led_strip, 0)

                        # Drive Gear
                        elif key == 'button_dpad_up':
                            if value == 1:
                                logging.info('Setting Gear to Drive')
                                state_map['gear'] = 'drive'
                                light_patterns.reverse_lights(led_strip, 0)

                        # Reverse Gear
                        elif key == 'button_dpad_down':
                            if value == 1:
                                logging.info('Setting Gear to Reverse')
                                state_map['gear'] = 'reverse'
                                light_patterns.reverse_lights(led_strip, 1)

                        #TODO: Detect hold mode (or not)
                        # Left Indicator Button Input
                        elif key == 'button_l1':
                            if value == 1:
                                logging.info('Toggling Left Indicator')
                                handle_indicator_event('left')
                        # Right Indicator Button Input
                        elif key == 'button_r1':
                            if value == 1:
                                logging.info('Toggling Right Indicator')
                                handle_indicator_event('right')

                        # Warn Indicator Button Input
                        elif key == 'button_back':
                            if value == 1:
                                logging.info('Toggling Warn Indicator')
                                handle_indicator_event('warn')
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
    #imu_module.initialize()
    #imu_module.register_callback(imu_callback)
    #imu_module.run()
