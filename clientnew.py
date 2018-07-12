import socket
import sys
import time
import pigpio
import os
import PID
from threading import Thread
import subprocess
import serial
from serialpacket import *
import servo
from imu import IMU
import logging
import coloredlogs

#imu = IMU()
#imu.initialize()
#thread.start(imu.run)

#set_servo_pulse(channel, pulse)

# Controller values
r_trigger = 0.0
l_trigger = 0.0
l_thumb_y = 0.0
l_thumb_x = 0.0
r_thumb_y = 0.0
r_thumb_x = 0.0

# Value change flags
r_trigger_f = 0
l_trigger_f = 0
l_thumb_y_f = 0
l_thumb_x_f = 0
r_thumb_y_f = 0
r_thumb_x_f = 0

# Thread stop signal
stop_signal = 0

class CarClient():
    def __init__(self):
        self.INPUT_PINS = [5, 6, 13, 19] #TODO: Add remaining 6 pins
        self.OUTPUT_PINS = [5, 6, 16, 20, 23, 24]
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
        self.serial_client = None
        self.serial_port = '/dev/ttyACM0'
        self.serial_baudrate = 115200

        # Network Variables
        self.server_socket = None
        self.server_host = '192.168.1.66'  # The remote host
        self.server_port = 50008  # The same port as used by the server

        # Video Streaming Variables
        self.video_process = None
        self.video_port = 4200

    def initialize(self):
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
            self.callbacks.append(self.pi.callback(pin, pigpio.RISING_EDGE, self.pin_callback))

        # Serial (Arduino) Setup
        self.serial_client = serial.Serial(self.serial_port, self.serial_baudrate)
        self.serial_client.flushInput()

        # Network Setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.connect((self.server_host, self.server_port))

        # def thread_2( threadName ):
        #   os.system('raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 2000000 -o udp://192.168.1.71:4200')

        # Video Streaming Setup
        #TODO: FIX H264 STREAaMING
        #self.video_process = subprocess.Popen(['raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 10000000 -o -'
        #                                       ' |  gst-launch-1.0 fdsrc ! h264parse ! rtph264pay config-interval=1 pt=96 ! gdppay'
        #                                       ' ! udpsink host='+self.server_host+' port='+str(self.video_port)], shell=True)
        os.system('killall raspivid')
        self.video_process = subprocess.Popen(['raspivid -t 0 -h 1080 -w 1920 -fps 30 -hf -vf -b 3000000 -o udp://'+self.server_host+':'+str(self.video_port)], shell=True)

        # Processes Setup
        self.threads.append(Thread(target=run_controller_thread, args=()))

    def run(self):
        try:
            for t in self.threads:
                t.start()
            self.run_network_thread()
        except Exception, e:
            logging.error(e)
            self.shutdown()
            raise e
        except KeyboardInterrupt:
            self.shutdown()

    def shutdown(self):
        global stop_signal

        logging.info("Tidying up")

        # Disable PWM controller.
        logging.info("Disabling PWM Controllers...")

        # Ground digital output pins
        logging.info("Grounding output pins...")
        for pin in self.OUTPUT_PINS:
            self.pi.write(pin, 0)

        # Remove all pin callbacks
        logging.info("Removing all pin callbacks...")
        for c in self.callbacks:
            c.cancel()

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

    def pin_callback(self, pin, level, tick):
        if self.last[pin] is not None:
            diff = time.time() - self.last[pin]
            #TODO: Check Diff value
            if diff > 0.01 and level == 1 and pin in self.RPM_MAP:
                value = 3 / diff
                setattr(self, self.RPM_MAP[pin], value)
                logging.debug("PIN={} LEVEL={} TIME_DIFF={} {}={}".format(pin, level, diff, self.RPM_MAP[pin], value))

        self.last[pin] = time.time()

    def run_network_thread(self):
        global r_trigger, l_trigger, l_thumb_y, l_thumb_x, r_thumb_y, r_thumb_x
        global r_trigger_f, l_trigger_f, l_thumb_y_f, l_thumb_x_f, r_thumb_y_f, r_thumb_x_f
        global stop_signal

        logging.info("Ready to accept data.")

        while stop_signal == 0:
            packet = read_packet(self.server_socket)
            write_packet(self.server_socket, Packet(TYPE_ACK, ''))
            logging.debug('Packet Acknowledgement Sent')
            logging.debug('Received a packet.')
            logging.info('Packet Received: ' + str(packet))
            data = packet.data
            logging.debug('Received: ' + repr(data))
            for command in data.split('\n'):
                if command != "":
                    id, value = command.split("_")
                    if id == "l":
                        self.l_trigger = float(value)
                        self.l_trigger_f = 1
                    elif id == "r":
                        self.r_trigger = float(value)
                        self.r_trigger_f = 1
                    elif id == "y":
                        self.r_thumb_y = float(value)
                        self.r_thumb_y_f = 1
                    elif id == "x":
                        self.r_thumb_x = float(value)
                        self.r_thumb_x_f = 1
                    elif id == "lx":
                        self.l_thumb_x = float(value)
                        self.l_thumb_x_f = 1
                    elif id == "ly":
                        self.l_thumb_y = float(value)
                        self.l_thumb_y_f = 1
                    logging.debug('Processed: '+repr(command))


def run_controller_thread():
    global r_trigger, l_trigger, l_thumb_y, l_thumb_x, r_thumb_y, r_thumb_x
    global r_trigger_f, l_trigger_f, l_thumb_y_f, l_thumb_x_f, r_thumb_y_f, r_thumb_x_f
    global stop_signal

    while stop_signal == 0:
        logging.debug("Control loop.")
        time.sleep(5)
        if l_trigger_f == 1:
            l_trigger_f = 0

        if r_trigger_f == 1:
            r_trigger_f = 0

        if l_thumb_x_f == 1:
            l_thumb_x_f = 0

        if l_thumb_y_f == 1:
            l_thumb_y_f = 0

        if r_thumb_x_f == 1:
            r_thumb_x_f = 0

        if r_thumb_y_f == 1:
            r_thumb_y_f = 0


if __name__ == "__main__":
    coloredlogs.install(level='INFO', fmt='%(asctime)s %(hostname)s %(name)s[%(process)d] [%(threadName)s] %(levelname)s %(message)s')
    client = CarClient()
    client.initialize()
    client.run()
