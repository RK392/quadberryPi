# Echo client program
import socket
#import RPi.GPIO as GPIO
from SC.net.serialpacket import *

HOST = '192.168.1.71'    # The remote host
PORT = 50007              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
while True:
    packet = read_packet(s)
    #print("Packet Received: ")
    #print(packet)
    print(packet.data)
    #print(repr(packet))
    #s.send("strtb") #bad packet example
    write_packet(s, Packet(TYPE_ACK, ''))

