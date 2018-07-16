import sys
import time
import threading
import multiprocessing
import socket
import traceback
from operator import attrgetter

import errno
import subprocess
import logging
import json
import coloredlogs

from SC.net.serialpacket import *
from SC.util.common import *
from SC.util.constants import *
from SC.input.joystick import XInputJoystick


DISABLE_UI_FLAG = False
LOG_LEVEL = 'INFO'
REFRESH_INTERVAL = 0.1
VIDEO_STREAMING_PROGRAM = "A:/gstreamer/1.0/x86_64/bin/gst-launch-1.0.exe"

STOP_SIGNAL = Value('i',0)

app = None

ctrl_map = None
state_map = None

def handle_joystick(ctrl_map):
    """
    Grab 1st available gamepad, logging changes to the screen.
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
        logging.debug('button '+str(button)+': '+str(pressed))
        for key in ctrl_map.keys():
            if ctrl_map[key][CTRL_JOYSTICK_FIELD] == button:
                newval = ctrl_map[key]
                newval[CTRL_VALUE] = pressed
                newval[CTRL_FLAG] = 1
                ctrl_map[key] = newval

    left_speed = 0
    right_speed = 0

    @j.event
    def on_axis(axis, value):

        logging.debug('axis '+str(axis)+': '+str(value))
        for key in ctrl_map.keys():
            if ctrl_map[key][CTRL_JOYSTICK_FIELD] == axis:
                newval = ctrl_map[key]
                newval[CTRL_VALUE] = value
                newval[CTRL_FLAG] = 1
                ctrl_map[key] = newval
        # j.set_vibration(left_speed, right_speed)

    try:
        while True:
            if(STOP_SIGNAL.value == 1):
                raise Exception("Kill signal sent")
            # logging.debug("Send Loop")
            # conn.send("")
            j.dispatch_events()
            #time.sleep(.1)
    except Exception as e:
        logging.error(e)
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")


def handle_socket(conn, addr):
    global ctrl_map
    global state_map
    global app
    prev_read = time.time()
    try:
        while(True):
            try:
                if(STOP_SIGNAL.value == 1):
                    raise Exception("Kill signal sent")

                # if global state is updated
                #     send update to pi
                try:
                    for key in ctrl_map.keys():
                        if ctrl_map[key][CTRL_FLAG] == 1:
                            newval = ctrl_map[key]
                            newval[CTRL_FLAG] = 0
                            ctrl_map[key] = newval
                            nwk_str_val = ctrl_map[key][CTRL_NETWORK_LABEL] + '_' + str(ctrl_map[key][CTRL_VALUE])
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
                            new_state_map = json.loads(response.data)
                            for key in new_state_map:
                                state_map[key] = new_state_map[key]
                            logging.debug('State Map: ' + str(response.data))
                            if not DISABLE_UI_FLAG and app.main_screen is not None:
                                app.main_screen.update_data(state_map)
                        else:
                            raise PacketException("Invalid type: "+response.type)
                except PacketException, e:
                    logging.warn("Invalid packet dropped")
                    logging.warn(e)
            except socket.error, e:
                if e.args[0] == errno.EWOULDBLOCK:
                    print 'EWOULDBLOCK'
                    time.sleep(0.2)  # short delay, no tight loops
    except Exception as e:
        logging.error(e)
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")

def run_server(state_map, ctrl_map):
    global app

    threads = []
    # signal = Value('i', 0)
    # p]o\ = [signal]

    if not DISABLE_UI_FLAG:
        from SC.ui import dashboard
        app = dashboard.DashboardApp()

    try:
        HOST = ''                 # Symbolic name meaning all available interfaces
        PORT = 9012              # Arbitrary non-privileged port
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

        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind((HOST, PORT))
        s.listen(1)
        c_count = 1
        while(True):
            try:
                conn, addr = s.accept()
                logging.info("Connection made on "+str(addr))
                t1 = multiprocessing.Process(name="handle_joystick_"+str(c_count), target=handle_joystick, args=(ctrl_map,))
                t2 = threading.Thread(name="handle_socket_"+str(c_count), target=handle_socket, args=(conn, addr))
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
            except socket.timeout:
                pass
    except Exception as e:
        logging.error(e)
        logging.error(traceback.format_exc())
    except KeyboardInterrupt:
        logging.debug("Keyboard Interrupt")
    finally:
        logging.info("Tidying up")
        logging.info("Closing port")
        s.close()
        logging.info("Terminating Camera")
        proc.terminate()
        logging.info("Closing other threads/processes")
        STOP_SIGNAL.value = 1
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

    manager = multiprocessing.Manager()

    ctrl_map = generate_multiprocess_control_map2(manager)
    state_map = generate_multiprocess_state_map2(manager)

    run_server(state_map, ctrl_map)