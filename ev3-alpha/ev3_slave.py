#!/usr/bin/env python
'''
  This Python script uses Socket module available in Python to establish
  communication with the Raspberry Pi and receive commands and send data.
'''

import socket, sys, pickle
import thread, time
from ev3_diff_drive import EV3DiffDrive
import time

HOST = ''   # Symbolic name meaning all available interfaces
PORT = 8888 # Arbitrary non-privileged port
 
try :
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#    print 'Socket created'
except socket.error, msg :
#    print 'Failed to create socket. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()

try:
    s.bind((HOST, PORT))
except socket.error , msg:
#    print 'Bind failed. Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
    sys.exit()
     
#print 'Socket bind complete'

ev3 = EV3DiffDrive(invert=False) # set invert=True if you want to change notion of left/right
ev3.reset()

logger = []
def log_encoder():
     while 1:
         logger.append(ev3.encoder())

try:
    thread.start_new_thread( log_encoder,())
except:
    print "ERROR: unable to start thread ",e

handleCmd = {
    'fwd': ev3.forward,
    'bck': ev3.backward,
    'trn': ev3.turn,
    'stp': ev3.brake,}
raw_input()
s.close()
print logger
while 0:

    # receive data from client (data, addr)
    data,addr = s.recvfrom(1024)

    if not data: 
        break
    
    cmd,arg = pickle.loads(data)
    
    if cmd in handleCmd:
        reply = handleCmd[cmd](arg)
    else:
        reply = 'Unknown command'
    
    s.sendto(pickle.dumps(reply) , addr)
#    print 'Message[' + addr[0] + ':' + str(addr[1]) + '] - ' + str(type(reply)) + ' :: ' + str(reply)

s.close()

