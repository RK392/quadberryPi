import socket
import time

def send(data, port):
    s = socket.socket()
    s.bind(('', port))
    s.listen(5)
    c, addr = s.accept()
    print 'Got connection from',addr
    c.send(data)
    c.close()

if __name__ == '__main__':
    port = 1025
    num = 1
while True:
            print 'hey, sending data'
            for words in range(32,180):
				words = words/6.5
				data = str(words)
				print 'send data: %s' % data
				send(data,port)
#				time.sleep(0.01)
#            words = words + 1
#			if words == 50:
#				words = 5
#				
#
#			num = num + 1
