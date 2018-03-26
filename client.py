import socket
import sys

def read(port):
        s = socket.socket()
        host = '192.168.1.73' #(IP address of PC (server))
        s.connect((host,port))
        try:
                msg = s.recv(1024)
                s.close()
        except socket.error, msg:
            sys.stderr.write('error %s'%msg[1])
            s.close()
            print 'close'
            sys.exit(2)
        return msg

if __name__ == '__main__':
    port = 50008
    while True:
            print 'hey, checking TCP socket'
            data = read(port)
            print '%s' % data
            #print 'port num is: %d' % port






