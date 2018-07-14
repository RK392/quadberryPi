# Simple demo of of the PCA9685 PWM servo/LED controller library.
# This will move channel 0 from min to max position repeatedly.
# Author: Tony DiCola
# License: Public Domain
from __future__ import division
import time
import logging

# Import the PCA9685 module.
import Adafruit_PCA9685

# Uncomment to enable debug output.
#import logging
#logging.basicConfig(level=logging.DEBUG)

# Alternatively specify a different address and/or bus:
#pwm = Adafruit_PCA9685.PCA9685(address=0x41, busnum=2)

MAX_SERVO_SIGNAL = 4096

# Configure min and max servo pulse lengths
servo_min = 205  # Min pulse length out of 4096
servo_max = 409  # Max pulse length out of 4096

# Helper function to make setting a servo pulse width simpler.
def set_servo_pulse(pwm, channel, pulse):
    pwm.set_pwm(channel, 0, pulse)
    logging.debug('Setting PWM Channel '+str(channel)+' to '+str(pulse))



if __name__ == "__main__":
    # Initialise the PCA9685 using the default address (0x40).
    pwm = Adafruit_PCA9685.PCA9685(0x41)
    # Set frequency to 60hz, good for servos.
    pwm.set_pwm_freq(50)
    logging.basicConfig(level=logging.DEBUG)
    set_servo_pulse(5, 0)
    time.sleep(1)
    set_servo_pulse(pwm, 5, servo_min)
    time.sleep(1)
    set_servo_pulse(pwm, 5, servo_max)
    time.sleep(1)
    set_servo_pulse(pwm, 5, 0)
