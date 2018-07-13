#!/usr/bin/env python

"""
A module for getting input from Microsoft XBox 360 controllers via the XInput library on Windows.

Adapted from Jason R. Coombs' code here:
http://pydoc.net/Python/jaraco.input/1.0.1/jaraco.input.win32.xinput/
under the MIT licence terms

Upgraded to Python 3
Modified to add deadzones, reduce noise, and support vibration
Only req is Pyglet 1.2alpha1 or higher:
pip install --upgrade http://pyglet.googlecode.com/archive/tip.zip 
"""

import ctypes
import sys
import time
import threading
import socket
from operator import itemgetter, attrgetter
from itertools import count, starmap
from serialpacket import *

import errno
from pyglet import event
import subprocess
import logging
import json
import coloredlogs

DISABLE_UI_FLAG = False
LOG_LEVEL = 'INFO'

CTRL_FLAG = 'flag'
CTRL_JOYSTICK_FIELD = 'joystick_field'
CTRL_NETWORK_LABEL = 'nw_label'
CTRL_VALUE = 'value'

REFRESH_INTERVAL = 0.25

VIDEO_STREAMING_PROGRAM = "A:/gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe"

app = None

# 1 Dpad Up
# 2 Dpad Down
# 3 Dpad Left
# 4 Dpad Right
# 5 Start
# 6 Back
# 7 L3
# 8 R3
# 9 L1
# 10 R1
# 13 A
# 14 B
# 15 X
# 16 Y

#controller values
ctrl_map = {
    "r_trigger": {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'right_trigger', CTRL_NETWORK_LABEL: 'r'},
    'l_trigger': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'left_trigger', CTRL_NETWORK_LABEL: 'l'},
    'l_thumb_y': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'l_thumb_y', CTRL_NETWORK_LABEL: 'ly'},
    'l_thumb_x': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'l_thumb_x', CTRL_NETWORK_LABEL: 'lx'},
    'r_thumb_y': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'r_thumb_y', CTRL_NETWORK_LABEL: 'y'},
    'r_thumb_x': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 'r_thumb_x', CTRL_NETWORK_LABEL: 'x'},

    'button_dpad_up': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 1, CTRL_NETWORK_LABEL: 'bdu'},
    'button_dpad_down': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 2, CTRL_NETWORK_LABEL: 'bdd'},
    'button_dpad_left': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 3, CTRL_NETWORK_LABEL: 'bdl'},
    'button_dpad_right': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 4, CTRL_NETWORK_LABEL: 'bdr'},

    'button_a':  {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 13, CTRL_NETWORK_LABEL: 'ba'},
    'button_b':  {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 14, CTRL_NETWORK_LABEL: 'bb'},
    'button_x':  {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 15, CTRL_NETWORK_LABEL: 'bx'},
    'button_y':  {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 16, CTRL_NETWORK_LABEL: 'by'},

    'button_l1': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 9, CTRL_NETWORK_LABEL: 'bl1'},
    'button_r1': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 10, CTRL_NETWORK_LABEL: 'br1'},
    'button_l3': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 7, CTRL_NETWORK_LABEL: 'bl3'},
    'button_r3': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 8, CTRL_NETWORK_LABEL: 'br3'},

    'button_start': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 5, CTRL_NETWORK_LABEL: 'bstart'},
    'button_back': {CTRL_FLAG:False, CTRL_JOYSTICK_FIELD: 6, CTRL_NETWORK_LABEL: 'bback'}
}

state_map = {}

# structs according to
# http://msdn.microsoft.com/en-gb/library/windows/desktop/ee417001%28v=vs.85%29.aspx

class XINPUT_GAMEPAD(ctypes.Structure):
    _fields_ = [
        ('buttons', ctypes.c_ushort),  # wButtons
        ('left_trigger', ctypes.c_ubyte),  # bLeftTrigger
        ('right_trigger', ctypes.c_ubyte),  # bLeftTrigger
        ('l_thumb_x', ctypes.c_short),  # sThumbLX
        ('l_thumb_y', ctypes.c_short),  # sThumbLY
        ('r_thumb_x', ctypes.c_short),  # sThumbRx
        ('r_thumb_y', ctypes.c_short),  # sThumbRy
    ]


class XINPUT_STATE(ctypes.Structure):
    _fields_ = [
        ('packet_number', ctypes.c_ulong),  # dwPacketNumber
        ('gamepad', XINPUT_GAMEPAD),  # Gamepad
    ]


class XINPUT_VIBRATION(ctypes.Structure):
    _fields_ = [("wLeftMotorSpeed", ctypes.c_ushort),
                ("wRightMotorSpeed", ctypes.c_ushort)]

class XINPUT_BATTERY_INFORMATION(ctypes.Structure):
    _fields_ = [("BatteryType", ctypes.c_ubyte),
                ("BatteryLevel", ctypes.c_ubyte)]

xinput = ctypes.windll.xinput1_4
#xinput = ctypes.windll.xinput9_1_0  # this is the Win 8 version ?
# xinput1_2, xinput1_1 (32-bit Vista SP1)
# xinput1_3 (64-bit Vista SP1)


def struct_dict(struct):
    """
    take a ctypes.Structure and return its field/value pairs
    as a dict.

    >>> 'buttons' in struct_dict(XINPUT_GAMEPAD)
    True
    >>> struct_dict(XINPUT_GAMEPAD)['buttons'].__class__.__name__
    'CField'
    """
    get_pair = lambda field_type: (
        field_type[0], getattr(struct, field_type[0]))
    return dict(list(map(get_pair, struct._fields_)))


def get_bit_values(number, size=32):
    """
    Get bit values as a list for a given number

    >>> get_bit_values(1) == [0]*31 + [1]
    True

    >>> get_bit_values(0xDEADBEEF)
    [1L, 1L, 0L, 1L, 1L, 1L, 1L, 0L, 1L, 0L, 1L, 0L, 1L, 1L, 0L, 1L, 1L, 0L, 1L, 1L, 1L, 1L, 1L, 0L, 1L, 1L, 1L, 0L, 1L, 1L, 1L, 1L]

    You may override the default word size of 32-bits to match your actual
    application.
    >>> get_bit_values(0x3, 2)
    [1L, 1L]

    >>> get_bit_values(0x3, 4)
    [0L, 0L, 1L, 1L]
    """
    res = list(gen_bit_values(number))
    res.reverse()
    # 0-pad the most significant bit
    res = [0] * (size - len(res)) + res
    return res


def gen_bit_values(number):
    """
    Return a zero or one for each bit of a numeric value up to the most
    significant 1 bit, beginning with the least significant bit.
    """
    number = int(number)
    while number:
        yield number & 0x1
        number >>= 1

ERROR_DEVICE_NOT_CONNECTED = 1167
ERROR_SUCCESS = 0


class XInputJoystick(event.EventDispatcher):

    """
    XInputJoystick

    A stateful wrapper, using pyglet event model, that binds to one
    XInput device and dispatches events when states change.

    Example:
    controller_one = XInputJoystick(0)
    """
    max_devices = 4

    def __init__(self, device_number, normalize_axes=True):
        values = vars()
        del values['self']
        self.__dict__.update(values)

        super(XInputJoystick, self).__init__()

        self._last_state = self.get_state()
        self.received_packets = 0
        self.missed_packets = 0

        # Set the method that will be called to normalize
        #  the values for analog axis.
        choices = [self.translate_identity, self.translate_using_data_size]
        self.translate = choices[normalize_axes]

    def translate_using_data_size(self, value, data_size):
        # normalizes analog data to [0,1] for unsigned data
        #  and [-0.5,0.5] for signed data
        data_bits = 8 * data_size
        return float(value) / (2 ** data_bits - 1)

    def translate_identity(self, value, data_size=None):
        return value

    def get_state(self):
        "Get the state of the controller represented by this object"
        state = XINPUT_STATE()
        res = xinput.XInputGetState(self.device_number, ctypes.byref(state))
        if res == ERROR_SUCCESS:
            return state
        if res != ERROR_DEVICE_NOT_CONNECTED:
            raise RuntimeError(
                "Unknown error %d attempting to get state of device %d" % (res, self.device_number))
        # else return None (device is not connected)

    def is_connected(self):
        return self._last_state is not None

    @staticmethod
    def enumerate_devices():
        "Returns the devices that are connected"
        devices = list(
            map(XInputJoystick, list(range(XInputJoystick.max_devices))))
        return [d for d in devices if d.is_connected()]

    def set_vibration(self, left_motor, right_motor):
        "Control the speed of both motors seperately"
        # Set up function argument types and return type
        XInputSetState = xinput.XInputSetState
        XInputSetState.argtypes = [ctypes.c_uint, ctypes.POINTER(XINPUT_VIBRATION)]
        XInputSetState.restype = ctypes.c_uint

        vibration = XINPUT_VIBRATION(
            int(left_motor * 65535), int(right_motor * 65535))
        XInputSetState(self.device_number, ctypes.byref(vibration))

    def get_battery_information(self):
        "Get battery type & charge level"
        BATTERY_DEVTYPE_GAMEPAD = 0x00
        BATTERY_DEVTYPE_HEADSET = 0x01
        # Set up function argument types and return type
        XInputGetBatteryInformation = xinput.XInputGetBatteryInformation
        XInputGetBatteryInformation.argtypes = [ctypes.c_uint, ctypes.c_ubyte, ctypes.POINTER(XINPUT_BATTERY_INFORMATION)]
        XInputGetBatteryInformation.restype = ctypes.c_uint 

        battery = XINPUT_BATTERY_INFORMATION(0,0)
        XInputGetBatteryInformation(self.device_number, BATTERY_DEVTYPE_GAMEPAD, ctypes.byref(battery))

        #define BATTERY_TYPE_DISCONNECTED       0x00
        #define BATTERY_TYPE_WIRED              0x01
        #define BATTERY_TYPE_ALKALINE           0x02
        #define BATTERY_TYPE_NIMH               0x03
        #define BATTERY_TYPE_UNKNOWN            0xFF
        #define BATTERY_LEVEL_EMPTY             0x00
        #define BATTERY_LEVEL_LOW               0x01
        #define BATTERY_LEVEL_MEDIUM            0x02
        #define BATTERY_LEVEL_FULL              0x03
        batt_type = "Unknown" if battery.BatteryType == 0xFF else ["Disconnected", "Wired", "Alkaline","Nimh"][battery.BatteryType]
        level = ["Empty", "Low", "Medium", "Full"][battery.BatteryLevel]
        return batt_type, level

    def dispatch_events(self):
        "The main event loop for a joystick"
        state = self.get_state()
        if not state:
            raise RuntimeError(
                "Joystick %d is not connected" % self.device_number)
        if state.packet_number != self._last_state.packet_number:
            # state has changed, handle the change
            self.update_packet_count(state)
            self.handle_changed_state(state)
        self._last_state = state

    def update_packet_count(self, state):
        "Keep track of received and missed packets for performance tuning"
        self.received_packets += 1
        missed_packets = state.packet_number - \
            self._last_state.packet_number - 1
        if missed_packets:
            self.dispatch_event('on_missed_packet', missed_packets)
        self.missed_packets += missed_packets

    def handle_changed_state(self, state):
        "Dispatch various events as a result of the state changing"
        self.dispatch_event('on_state_changed', state)
        self.dispatch_axis_events(state)
        self.dispatch_button_events(state)

    def dispatch_axis_events(self, state):
        # axis fields are everything but the buttons
        axis_fields = dict(XINPUT_GAMEPAD._fields_)
        axis_fields.pop('buttons')
        for axis, type in list(axis_fields.items()):
            old_val = getattr(self._last_state.gamepad, axis)
            new_val = getattr(state.gamepad, axis)
            data_size = ctypes.sizeof(type)
            old_val = self.translate(old_val, data_size)
            new_val = self.translate(new_val, data_size)

            # an attempt to add deadzones and dampen noise
            # done by feel rather than following http://msdn.microsoft.com/en-gb/library/windows/desktop/ee417001%28v=vs.85%29.aspx#dead_zone
            # ags, 2014-07-01
            if ((old_val != new_val and abs(old_val - new_val) > 0.00400000000) or
               (axis == 'right_trigger' or axis == 'left_trigger') and new_val == 0 and abs(old_val - new_val) > 0.00000000500000000):
                self.dispatch_event('on_axis', axis, new_val)

    def dispatch_button_events(self, state):
        changed = state.gamepad.buttons ^ self._last_state.gamepad.buttons
        changed = get_bit_values(changed, 16)
        buttons_state = get_bit_values(state.gamepad.buttons, 16)
        changed.reverse()
        buttons_state.reverse()
        button_numbers = count(1)
        changed_buttons = list(
            filter(itemgetter(0), list(zip(changed, button_numbers, buttons_state))))
        tuple(starmap(self.dispatch_button_event, changed_buttons))

    def dispatch_button_event(self, changed, number, pressed):
        self.dispatch_event('on_button', number, pressed)

    # stub methods for event handlers
    def on_state_changed(self, state):
        pass

    def on_axis(self, axis, value):
        pass

    def on_button(self, button, pressed):
        pass

    def on_missed_packet(self, number):
        pass

list(map(XInputJoystick.register_event_type, [
    'on_state_changed',
    'on_axis',
    'on_button',
    'on_missed_packet',
]))


def determine_optimal_sample_rate(joystick=None):
    """
    Poll the joystick slowly (beginning at 1 sample per second)
    and monitor the packet stream for missed packets, indicating
    that the sample rate is too slow to avoid missing packets.
    Missed packets will translate to a lost information about the
    joystick state.
    As missed packets are registered, increase the sample rate until
    the target reliability is reached.
    """
    # in my experience, you want to probe at 200-2000Hz for optimal
    #  performance
    if joystick is None:
        joystick = XInputJoystick.enumerate_devices()[0]

    j = joystick

    print("Move the joystick or generate button events characteristic of your app")
    print("Hit Ctrl-C or press button 6 (<, Back) to quit.")

    # here I use the joystick object to store some state data that
    #  would otherwise not be in scope in the event handlers

    # begin at 1Hz and work up until missed messages are eliminated
    j.probe_frequency = 1  # Hz
    j.quit = False
    j.target_reliability = .99  # okay to lose 1 in 100 messages

    @j.event
    def on_button(button, pressed):
        # flag the process to quit if the < button ('back') is pressed.
        j.quit = (button == 6 and pressed)

    @j.event
    def on_missed_packet(number):
        print('missed %(number)d packets' % vars())
        total = j.received_packets + j.missed_packets
        reliability = j.received_packets / float(total)
        if reliability < j.target_reliability:
            j.missed_packets = j.received_packets = 0
            j.probe_frequency *= 1.5

    while not j.quit:
        j.dispatch_events()
        time.sleep(1.0 / j.probe_frequency)
    print("final probe frequency was %s Hz" % j.probe_frequency)

def handle_joystick(conn, addr, signal_list):
    """
    Grab 1st available gamepad, logging changes to the screen.
    L & R analogue triggers set the vibration motor speed.
    """

    joysticks = XInputJoystick.enumerate_devices()
    device_numbers = list(map(attrgetter('device_number'), joysticks))

    logging.debug('found %d devices: %s' % (len(joysticks), device_numbers))

    if not joysticks:
        sys.exit(0)

    j = joysticks[0]
    print('using %d' % j.device_number)

    battery = j.get_battery_information()
    print(battery)

    @j.event
    def on_button(button, pressed):
        global ctrl_map
        logging.debug('button '+str(button)+': '+str(pressed))
        for key in ctrl_map:
            if ctrl_map[key][CTRL_JOYSTICK_FIELD] == button:
                ctrl_map[key][CTRL_VALUE] = pressed
                ctrl_map[key][CTRL_FLAG] = True

    left_speed = 0
    right_speed = 0

    @j.event
    def on_axis(axis, value):
        global ctrl_map

        logging.debug('axis '+str(axis)+': '+str(value))
        for key in ctrl_map:
            if ctrl_map[key][CTRL_JOYSTICK_FIELD] == axis:
                ctrl_map[key][CTRL_VALUE] = value
                ctrl_map[key][CTRL_FLAG] = True
        # j.set_vibration(left_speed, right_speed)

    try:
        while True:
            if(signal_list[0]):
                raise Exception("Kill signal sent")
            #logging.debug("Send Loop")
            #conn.send("")
            j.dispatch_events()
            time.sleep(.1)
    except Exception as e:
        logging.error(e)
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")


def handle_socket(conn, addr, signal_list):
    global ctrl_map
    global state_map
    global app
    prev_read = time.time()
    try:
        while(True):
            try:
                if(signal_list[0]):
                    raise Exception("Kill signal sent")

                # if global state is updated
                #     send update to pi
                try:
                    for key in ctrl_map:
                        if ctrl_map[key][CTRL_FLAG]:
                            ctrl_map[key][CTRL_FLAG] = False
                            nwk_str_val = ctrl_map[key][CTRL_NETWORK_LABEL]+'_'+str(ctrl_map[key][CTRL_VALUE])
                            logging.info('Sending Command: ' + nwk_str_val)
                            command_packet = Packet(TYPE_CMD_UPDATE, nwk_str_val)
                            response = send_command(conn, command_packet)
                            #logging.info("Received:" + repr(response))
                            if response.type != TYPE_ACK:
                                raise PacketException("Invalid type: "+response.type)
                except PacketException, e:
                    logging.warn("Invalid packet dropped")
                    logging.warn(e)

                try:
                    # Read state of car
                    time_diff = time.time() - prev_read
                    if time_diff > REFRESH_INTERVAL:
                        prev_read = time.time()
                        command_packet = Packet(TYPE_CMD_READ, '')
                        response = send_command(conn, command_packet)
                        #logging.info("Received:" + repr(response))
                        if response.type == TYPE_VALUE:
                            state_map = json.loads(response.data)
                            logging.debug('State Map: ' + str(response.data))
                            if not DISABLE_UI_FLAG and app.main_screen is not None:
                                app.main_screen.update_data(state_map)
                        else:
                            raise PacketException("Invalid type: "+response.type)
                except PacketException, e:
                    logging.warn("Invalid packet dropped")
                    logging.warn(e)
                # read sensor data from pi
                #response = send_command(conn,Packet(...))
                #packet = Packet(TYPETOREQUESTDATA, WHICHSENSOR?)
                #global variable = send_command(conn,Packet(REQUEST HERE)))
                #data = conn.recv(1024)
                #TODO: Check if this line needs to be added to serialpacket.py
                #if not data: break
                #Handle input here
            except socket.error, e:
                if e.args[0] == errno.EWOULDBLOCK:
                    print 'EWOULDBLOCK'
                    time.sleep(0.2)  # short delay, no tight loops
    except Exception as e:
        logging.error(e)
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")

def run_server():
    global app

    threads = []
    signal = False
    signal_list = [signal]

    if not DISABLE_UI_FLAG:
        import server_ui
        app = server_ui.DashboardApp()

    try:
        HOST = ''                 # Symbolic name meaning all available interfaces
        PORT = 50008              # Arbitrary non-privileged port
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.settimeout(0.2)  # timeout for listening
        #s.setblocking(0)

        #TODO: Fix this
        #proc = subprocess.Popen(
        #    [program, "udpsrc", "port=4200", "!", "gdpdepay", "!", "rtph264depay", "!", "avdec_h264",
        #     "!", "fpsdisplaysink", "sync=false", "text-overlay=false"], shell=True)
        proc = subprocess.Popen(
            [VIDEO_STREAMING_PROGRAM, "udpsrc", "port=4200", "!", "h264parse", "!", "avdec_h264", "!", "textoverlay",
             "text=quadberrypi",
             "!", "autovideosink", "sync=false", "text-overlay=false"], shell=True)

        s.bind((HOST, PORT))
        s.listen(1)
        c_count = 1
        while(True):
            try:
                conn, addr = s.accept()
                logging.info("Connection made on "+str(addr))
                t1 = threading.Thread(name="handle_joystick_"+str(c_count), target=handle_joystick, args=(conn, addr, signal_list))
                t2 = threading.Thread(name="handle_socket_"+str(c_count), target=handle_socket, args=(conn, addr, signal_list))
                if not DISABLE_UI_FLAG:
                    t3 = threading.Thread(target=app.run, args=())
                    t3.start()
                t1.start()
                t2.start()


                threads.append(t1)
                threads.append(t2)
                if not DISABLE_UI_FLAG:
                    threads.append(t3)
                c_count = c_count + 1
                #while(t1.is_alive() or t2.is_alive()):
                    #t1.join(5)
                    #t2.join(5)
            except socket.timeout:
                pass
    except Exception as e:
        logging.error(e)
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")
    finally:
        logging.info("Terminating Camera")
        proc.terminate()
        logging.info("Tidying up")
        signal_list[0] = True
        if not DISABLE_UI_FLAG:
            app.stop()
        for c_thread in threads:
            c_thread.join()


if __name__ == "__main__":
    for arg in sys.argv[1:]:
        key, val = arg.split('=')
        if key == 'LOG_LEVEL':
            LOG_LEVEL = val
        elif key == 'UI':
            DISABLE_UI_FLAG = (val == '0')
    if DISABLE_UI_FLAG:
        coloredlogs.install(level=LOG_LEVEL, fmt='%(asctime)s %(hostname)s %(name)s[%(process)d] [%(threadName)s] %(levelname)s %(message)s')
    run_server()
    #sample_first_joystick()
    # determine_optimal_sample_rate()

#data = conn.recv(1024)
#if not data: data = 0