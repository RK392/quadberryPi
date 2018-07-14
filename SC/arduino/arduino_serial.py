import serial
import struct
import logging


def readFrame(ser):
    start = ''
    valid = False
    result = None
    while not valid:
        while start != 's':
            start = ser.read(1)

        #print("start: " + start)
        #print("Potential start found")
        read_ser = start+ser.read(17)
        reading = read_ser

        logging.debug('Received from Uno: '+str(read_ser))
        logging.debug('Received from Uno (hex): '+"".join("{:02x}".format(ord(c)) for c in read_ser))

        result = struct.unpack('<BffffB', reading)

        logging.debug('Parsed Result: ' + str(result))

        if chr(result[5]) != 'e':
            logging.debug('Invalid packet dropped.')
        else:
            valid = True
    return result[1:-1]


def writeFrame(ser, steering, throttle, flags):
    frame = struct.pack('<BffBB', 115, steering, throttle, flags, 101)

    logging.debug('Sending to Uno: ' + repr(frame))
    logging.debug('Sending to Uno (hex): ' + ''.join('{:02x}'.format(ord(c)) for c in frame))

    result = ser.write(frame)

    logging.debug('Wrote ' + str(result) + ' bytes')

    return result


if __name__ == "__main__":
    ser_test = serial.Serial('/dev/ttyACM0', 9600)
    ser_test.flushInput()
    readFrame(ser_test)
    writeFrame(ser_test, 0.5, 0.6, 0b10101010)
    while True:
        readFrame(ser_test)
        writeFrame(ser_test, 0.5, 0.6, 0b10101010)
