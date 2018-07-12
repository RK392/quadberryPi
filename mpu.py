import serial
import struct

ser = serial.Serial('/dev/ttyS0', 9600)
ser.flushInput()
ser.flushOutput()

def readFrame():
#    start = ''
#    print("Scanning until frame start")
#    while start != 's':
#        start = ser.read(1)
#    print("Potential start found")

    read_ser = ser.readline()
    #read_ser = ser.read(32)
    reading = read_ser

    print 'Recieved from Uno: ', (read_ser)
    print 'Recieved from Uno: ', repr(read_ser)
    print 'Recieved from Uno (hex): ', ("".join("{:02x}".format(ord(c)) for c in read_ser))

    result = struct.unpack('<Bfffffff', reading)
    print("Parsed Result: " + str(result))

def writeFrame():
    echo = struct.pack('<Bfffffff',115,1.23,4.56,7.89,10.11,12.13,14.15,16.17)

    #echo = struct.pack('26c', *bytes("Simba says 'hello world'.."))

    #Send Serial here

    #sendt = 123.321
    print 'Sending to Uno: ', repr(echo)
#    print 'Sending to Uno (hex): ', (''.join('{:02x}'.format(ord(c)) for c in repr(echo)))

    result = ser.write(echo)

 #   print 'Wrote ' + str(result) + ' bytes'

if __name__ == "__main__":
    readFrame()
    #writeFrame()
    while True:
        readFrame()
#        writeFrame()



