import time
import logging

HEAD_LIGHT_WIDTH = 36
BRAKE_LIGHT_WIDTH = 18
REVERSE_LIGHT_WIDTH = 9
BLINKER_LIGHT_WIDTH = 18
BLINKER_LIGHT_OFFSET = 18

BLINKER_REPEAT = 10

QUARTER_WIDTH = 36
LED_COUNT = 144

COLOUR_WHITE = 0xFFFFFF
COLOUR_HALF_WHITE = 0x7F7F7F
COLOUR_RED = 0xFF0000
COLOUR_HALF_RED = 0x7F0000
COLOUR_YELLOW = 0xFFFF00
COLOUR_OFF = 0x000000

sections = [
    {'min': 35, 'max': 0},
    {'min': 36, 'max': 71},
    {'min': 107, 'max': 72},
    {'min': 108, 'max': 143}
]

led_prev_states = [0] * 144
led_curr_states = [0] * 144

def print_lights():
    global led_prev_states, led_curr_states
    str_lights = ''
    for led in led_curr_states:
        str_lights = str_lights + ('R' if led == COLOUR_RED else
                                   'r' if led == COLOUR_HALF_RED else 'W' if led == COLOUR_WHITE else
                                   'w' if led == COLOUR_HALF_WHITE else 'Y' if led == COLOUR_YELLOW else 'o')
    logging.debug("lights: " + str_lights)

def default_lights(led_strip, value):
    global led_prev_states, led_curr_states
    width = QUARTER_WIDTH
    for sec_id in [0, 1, 2, 3]:
        section = sections[sec_id]
        i_min = section['min']
        reverse = section['min'] > section['max']
        i_max = (i_min + width) if not reverse else (i_min - width)
        if value == 1:
            if sec_id in [0, 1]:
                colour = COLOUR_HALF_WHITE
            else:
                colour = COLOUR_HALF_RED
        else:
            colour = COLOUR_OFF
        for i in range(i_min,i_max,1 if not reverse else -1):
            led_prev_states[i] = led_curr_states[i]
            led_curr_states[i] = colour
            led_strip.set_pixel_rgb(i,colour)
    print_lights()
    led_strip.show()

def head_lights(led_strip, value):
    global led_prev_states, led_curr_states
    width = HEAD_LIGHT_WIDTH
    for sec_id in [0, 1]:
        section = sections[sec_id]
        i_min = section['min']
        reverse = section['min'] > section['max']
        i_max = (i_min + width) if not reverse else (i_min-width)
        if value == 1:
            colour = COLOUR_WHITE
        else:
            colour = COLOUR_HALF_WHITE
        for i in range(i_min,i_max,1 if not reverse else -1):
            led_prev_states[i] = led_curr_states[i]
            led_curr_states[i] = colour
            led_strip.set_pixel_rgb(i,colour)
    print_lights()
    led_strip.show()

def brake_lights(led_strip, value):
    global led_prev_states, led_curr_states
    width = BRAKE_LIGHT_WIDTH
    for sec_id in [2, 3]:
        section = sections[sec_id]
        i_min = section['min']
        reverse = section['min'] > section['max']
        i_max = (i_min + width) if not reverse else (i_min - width)
        if value > 0.1:
            colour = COLOUR_RED
        else:
            colour = COLOUR_HALF_RED
        for i in range(i_min, i_max, 1 if not reverse else -1):
            led_prev_states[i] = led_curr_states[i]
            led_curr_states[i] = colour
            led_strip.set_pixel_rgb(i, colour)
    print_lights()
    led_strip.show()

def reverse_lights(led_strip, value):
    global led_prev_states, led_curr_states
    width = REVERSE_LIGHT_WIDTH
    for sec_id in [2, 3]:
        section = sections[sec_id]
        i_min = section['min']
        reverse = section['min'] > section['max']
        i_max = (i_min + width) if not reverse else (i_min - width)
        if value == 1:
            colour = COLOUR_WHITE
        else:
            colour = COLOUR_HALF_RED
        for i in range(i_min, i_max, 1 if not reverse else -1):
            led_prev_states[i] = led_curr_states[i]
            led_curr_states[i] = colour
            led_strip.set_pixel_rgb(i, colour)
    print_lights()
    led_strip.show()

def indicator_lights(led_strip, indicator, mode, thread_info):
    global led_prev_states, led_curr_states
    width = BLINKER_LIGHT_WIDTH
    sec_ids = [0, 3] if indicator == 'left' else [1, 2] if indicator == 'right' else [0, 1, 2, 3]
    rep = 0
    if mode != 'off':
        while rep < BLINKER_REPEAT and thread_info['stop_signal'] == 0:
            for sec_id in sec_ids:
                section = sections[sec_id]
                reverse = section['min'] > section['max']
                i_min = (section['min'] + BLINKER_LIGHT_OFFSET) if not reverse else (section['min'] - BLINKER_LIGHT_OFFSET)
                i_max = (i_min + width) if not reverse else (i_min - width)

                colour = COLOUR_YELLOW

                for i in range(i_min, i_max, 1 if not reverse else -1):
                    led_strip.set_pixel_rgb(i, colour)

            print_lights()
            led_strip.show()
            time.sleep(0.5)

            for sec_id in sec_ids:
                section = sections[sec_id]
                reverse = section['min'] > section['max']
                i_min = (section['min'] + BLINKER_LIGHT_OFFSET) if not reverse else (section['min'] - BLINKER_LIGHT_OFFSET)
                i_max = (i_min + width) if not reverse else (i_min - width)

                colour = COLOUR_YELLOW

                for i in range(i_min, i_max, 1 if not reverse else -1):
                    led_strip.set_pixel_rgb(i, led_curr_states[i])

            print_lights()
            led_strip.show()
            time.sleep(0.5)

            if mode == 'single':
                rep = rep + 1