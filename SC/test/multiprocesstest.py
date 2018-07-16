from multiprocessing import Process, Manager
from SC.util.common import *

def f(d):
    d[1] = d[1] + [4]
    d['l_thumb_y']['joystick_field'] = 'x'
    d['l_thumb_x'] = 'z'
    print d

if __name__ == '__main__':
    manager = Manager() # create only 1 mgr
    d = manager.dict(generate_control_map()) # create only 1 dict
    d[1] = [3]
    p = Process(target=f,args=(d,)) # say to 'f', in which 'd' it should append
    p.start()
    p.join()
    print d
