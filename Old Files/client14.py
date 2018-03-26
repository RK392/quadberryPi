# Echo client program
import socket
import sys
#import RPi.GPIO as GPIO
import time
import pigpio
import os
import PID
import thread

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
pi.set_mode(23, pigpio.OUTPUT)
pi.set_mode(24, pigpio.OUTPUT)
pi.set_mode(12, pigpio.ALT0)
pi.set_mode(13, pigpio.ALT0)

pi.hardware_PWM(12, 1000, 0)
pi.hardware_PWM(13, 1000, 0)
pi.set_PWM_frequency(23, 50)
pi.set_PWM_frequency(24, 50)
pi.set_PWM_dutycycle(23, 0)
pi.set_PWM_dutycycle(24, 0)

pi.write(6, 1)
pi.write(5, 0)
pi.write(20, 1)
pi.write(16, 0)

value = 0
throttle = 0
reverse = 0
bias = 0
left = 0
right = 0

last = [None]*32
cb = []
lrpm = 0
rrpm = 0
lastrpm = 0

pid = PID.PID(0.5, 0.9, 0)
pid.SetPoint=0.0
#pid.SetPoint = 50
pid.setSampleTime(0.01)
feedback = 0
output = pid.output

def cbf(GPIO, level, tick):
   global lrpm, rrpm, lastrpm
   if last[GPIO] is not None:
      diff = time.time() - last[GPIO]
      if(diff > 0.01):
          if (level == 1) & (GPIO == 26):
             lrpm = 3/diff
             print("G={} l={} d={} lrpm={}".format(GPIO, level, diff, lrpm))
          if (level == 1) & (GPIO == 19):
             rrpm = 3/diff
             print("G={} l={} d={} rrpm={}".format(GPIO, level, diff, rrpm))
             #pid.update(rrpm/100)
             output = pid.output
             print("PID output={} rrpm={}".format(output, rrpm))
             lastrpm = time.time()

   last[GPIO] = time.time()

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

def update_pid( threadName ):
   global rrpm, lastrpm
   while True:
       time.sleep(0.01)
       if (time.time() - lastrpm) > 2:
         rrpm = 1
         #print "%s: Updating pid with value: %s" % ( threadName, rrpm/200 )
         pid.update(rrpm/200)
         #lastrpm = time.time()
       else:
         pid.update(rrpm/200)
       output = pid.output
       if output > 1:
         output = 1
       elif output < 0:
         output = 0
       #pi.hardware_PWM(13, 1000, output*500000)

def thread_2( threadName ):
   os.system('raspivid -o - -t 0 -hf -vf -w 1920 -h 1080 -b 1500000 -fps 30 | gst-launch-1.0 -e fdsrc ! h264parse ! rtph264pay pt=96 config-interval=10 ! queue max-size-bytes=0 max-size-buffers=0 max-size-time=5000000000 ! udpsink force-ipv4=true host=192.168.1.71 port=4200')

# Create two threads as follows
#try:
thread.start_new_thread( update_pid, ("PID-Thread",) )
thread.start_new_thread( thread_2, ("PID-Thread2",) )

#except Exception e:
#   print e
   
try:
   while True:
      s.sendall('Hello, world')
      data = s.recv(1024)
      #s.close()
      print 'Received', repr(data)
      id, value = data.split("_")
      if id == "l":
              pi.write(5, 1)
              pi.write(6, 0)
              pi.write(16, 1)
              pi.write(20, 0)
              reverse = float(value)
              if bias > 0:
                right = (reverse-bias)
                left = reverse
              elif bias < 0:
                left = (reverse+bias)
                right = reverse
              elif bias == 0:
                left = reverse
                right = reverse
              if left < 0 :
                left = 0
              if right < 0:
                right = 0
              pi.hardware_PWM(12, 1000, left*1000000)              
              pi.hardware_PWM(13, 1000, right*1000000)
      elif id == "r":
              #pid.SetPoint = value
              pi.write(5, 0)
              pi.write(6, 1)
              pi.write(16, 0)
              pi.write(20, 1)
              throttle = float(value)
              if bias > 0:
                right = (throttle-bias)
                left = throttle
              elif bias < 0:
                left = (throttle+bias)
                right = throttle
              elif bias == 0:
                left = throttle
                right = throttle
              if left < 0 :
                left = 0
              if right < 0:
                right = 0
              print "left %s    right %s" % ( left, right)
              pi.hardware_PWM(12, 1000, left*1000000)              
              pi.hardware_PWM(13, 1000, right*1000000)
      elif id == "y":
              pi.set_PWM_dutycycle(23, (-2*float(value)*0.0445+0.0805)*255)
              print "%s: Updating pid with value: %s" % ( id, -2*float(value)*0.0445+0.0805)
      elif id == "x":
              pi.set_PWM_dutycycle(24, (-2*float(value)*0.0425+0.0775)*255)
              print "%s: Updating pid with value: %s" % ( id, -2*float(value)*0.0445+0.0805)
      elif id == "lx":
              bias = float(value)
              if (bias > 0) & (reverse == 0):
                right = (throttle-bias)
                left = throttle
              elif (bias < 0) & (reverse == 0):
                left = (throttle+bias)
                right = throttle
              elif (bias > 0) & (throttle == 0):
                right = (reverse-bias)
                left = reverse
              elif (bias < 0) & (throttle == 0):
                left = (reverse+bias)
                right = reverse
              if left < 0 :
                left = 0
              if right < 0:
                right = 0
              print "left %s    right %s" % ( left, right)
              pi.hardware_PWM(12, 1000, left*1000000)              
              pi.hardware_PWM(13, 1000, right*1000000)


except KeyboardInterrupt:
    print("\nTidying up")
    pi.hardware_PWM(12, 1000, 0)
    pi.hardware_PWM(13, 1000, 0)
    pi.set_PWM_dutycycle(23, 0)
    pi.set_PWM_dutycycle(24, 0)
    pi.write(6, 0)
    pi.write(5, 0)
    pi.write(20, 0)
    pi.write(16, 0)
    for c in cb:
      c.cancel()

pi.stop()


