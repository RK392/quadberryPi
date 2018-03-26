# Echo client program
import socket
import sys
#import RPi.GPIO as GPIO
import time
import pigpio
import os

#os.system('gpio mode 26 pwm')
#os.system('gpio mode 23 pwm')
#GPIO.setmode(GPIO.BCM)
#GPIO.setwarnings(False)
#GPIO.setup(18, GPIO.OUT)
#pwm = GPIO.PWM(18, 50)
#pwm.start(5)
pi = pigpio.pi()

pi.set_mode(6, pigpio.OUTPUT)
pi.set_mode(5, pigpio.OUTPUT)
pi.set_mode(20, pigpio.OUTPUT)
pi.set_mode(16, pigpio.OUTPUT)
pi.set_mode(12, pigpio.ALT0)
pi.set_mode(13, pigpio.ALT0)
pi.hardware_PWM(12, 1000, 0)
pi.hardware_PWM(13, 1000, 0)

pi.write(6, 1)
pi.write(5, 0)
pi.write(20, 1)
pi.write(16, 0)

value = 0

last = [None]*32
cb = []
lrpm = 0
rrpm = 0

def cbf(GPIO, level, tick):
   global lrpm, rrpm
   if last[GPIO] is not None:
      diff = pigpio.tickDiff(last[GPIO], tick)
      if (level == 1) & (GPIO == 26):
         lrpm = 1500000/diff
         print("G={} l={} d={} lrpm={}".format(GPIO, level, diff, lrpm))
      if (level == 1) & (GPIO == 19):
		 rrpm = 1500000/diff
		 print("G={} l={} d={} rrpm={}".format(GPIO, level, diff, rrpm))
   last[GPIO] = tick

pi = pigpio.pi()

if not pi.connected:
   exit()

if len(sys.argv) == 1:
   G = range(0, 32)
else:
   G = []
   for a in sys.argv[1:]:
      G.append(int(a))
   
for g in G:
   cb.append(pi.callback(g, pigpio.RISING_EDGE, cbf))

HOST = '192.168.1.71'    # The remote host
PORT = 50008              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))

try:
   while True:
      s.sendall('Hello, world')
      data = s.recv(1024)
      #s.close()
      #print 'Received', repr(data)
      id, value = data.split("_")
      if id == "l":
              pi.write(16, 0)
              pi.write(20, 1)
              value = float(value)
              pi.hardware_PWM(12, 1000, value*500000)
      elif id == "r":
              pi.write(5, 0)
              pi.write(6, 1)
              value = float(value)
              pi.hardware_PWM(13, 1000, value*500000)
except KeyboardInterrupt:
   print("\nTidying up")
   for c in cb:
      c.cancel()

pi.stop()


