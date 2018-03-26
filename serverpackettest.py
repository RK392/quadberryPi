# Echo server program
import socket

from serialpacket import *

HOST = ''                 # Symbolic name meaning all available interfaces
PORT = 50007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((HOST, PORT))
s.listen(1)
conn, addr = s.accept()
print 'Connected by', addr
num = 1
while 1:
    try:
        num += 1
        packet = Packet(TYPE_CMD_UPDATE, 'test'+str(num))
        response = send_command(conn, packet)
        #print('Received Response: ')
        print(response)
        #time.sleep(3)
    except PacketException:
        print("Invalid packet dropped")
conn.close()