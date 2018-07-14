from time import sleep
import serial
ser = serial.Serial('/dev/ttyACM0', 115200) # Establish the connection on a specific port
ser.flushInput()
sleep(5)
ardIn = 0.0 # Below 32 everything in ASCII is gibberish
ardOut = 48
throttle = 0.0
while True:
     #ardIn = (int)(throttle * 255.999)
     #print ardIn
     #if (ardIn>=0) & (ardIn<=207):
     #   ardOut = ardIn + 48
     #elif (ardIn>=208) & (ardIn <=255):
     #   ardOut = ardIn - 208
     #else:
     #   ardOut = 48
     #print ardOut
     #ser.write(str('a'+chr(int(ardOut)))) # Convert the decimal number to ASCII then send it to the Arduino
     ser.write('123456') # Convert the decimal number to ASCII then send it to the Arduino
     read_ser=ser.readline()
     print(read_ser)  # Read the newest ardOut from the Arduino
     sleep(.05) # Delay for one tenth of a second
     #throttle +=0.00390625
     #if throttle > 1:
     #   throttle = 0
