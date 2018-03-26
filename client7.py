# Echo client program
import socket
import sys
#import RPi.GPIO as GPIO
import time
import pigpio
import os

os.system('gpio mode 26 pwm')
os.system('gpio mode 23 pwm')
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#GPIO.setup(18, GPIO.OUT)
#pwm = GPIO.PWM(18, 50)
#pwm.start(5)
pi = pigpio.pi()

pi.hardware_PWM(18, 20000, 70000)
pi.hardware_PWM(19, 20000, 70000)
pi.set_mode(6, pigpio.OUTPUT)
pi.set_mode(5, pigpio.OUTPUT)
pi.set_mode(20, pigpio.OUTPUT)
pi.set_mode(16, pigpio.OUTPUT)
pi.write(6, 1)
pi.write(5, 0)
pi.write(20, 1)
pi.write(16, 0)

HOST = '192.168.1.71'    # The remote host
PORT = 50008              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while 1:
        s.sendall('Hello, world')
        data = s.recv(1024)
        #s.close()
        print 'Received', repr(data)
        id, value = data.split("_")
        if id == "y":
                servo = 1-float(value)
                pi.hardware_PWM(13,20000,30000+80000*servo)
        elif id == "x":
                servo = 1-float(value)
                pi.hardware_PWM(12,20000,25000+88000*servo)
        elif id == "l":
                pi.write(16, 0)
                pi.write(20, 1)
                value = float(value)
                pi.set_PWM_dutycycle(12, value*255)
        elif id == "r":
                pi.write(5, 0)
                pi.write(6, 1)
                value = float(value)
                pi.set_PWM_dutycycle(13, value*255)


