#!/usr/bin/env python
'''
  This Python script uses Socket module available in Python to establish
  communication with the Raspberry Pi and receive commands and send data.
'''

import socket, sys, pickle
import thread, time
from ev3_diff_drive import EV3DiffDrive

HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8888 # Arbitrary non-privileged port
 
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    print 'Socket created'
except socket.error, msg :
    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

try:
    s.bind((HOST, PORT))
except socket.error , msg:
    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
print 'Socket bind complete'

ev3 = EV3DiffDrive(invert=False) # set invert=True if you want to change notion of left/right
ev3.reset()

#moves = {'str':a.move_forward,'rev':a.move_backward,'lft':a.turn_left,'rgt':a.turn_right,'stp':a.brake}

def log_encoder(delay):
    while 1:
        print ev3.encoder()
        time.sleep(delay)

try:
    thread.start_new_thread( log_encoder, (0.01,))
except:
    print "ERROR: unable to start thread ",e

while 1:
    # receive data from client (data, addr)
    data,addr = s.recvfrom(1024)

    if not data: 
        break
    
    cmd,arg = pickle.loads(data)
    
    if cmd in handleCmd:
        reply = handleCmd[cmd](arg)
    else:
        reply = default(arg)
    
    s.sendto(pickle.dumps(reply) , addr)
    print 'Message[' + addr[0] + ':' + str(addr[1]) + '] - ' + str(type(reply)) + ' :: ' + str(reply)

s.close()
