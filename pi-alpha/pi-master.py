#!/usr/bin/env python
'''
  This Python script uses Socket module available in Python to establish
  communication with the Rover and send commands and receive data.
'''

import socket, sys, pickle
from ev3settings import *

try:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
except socket.error:
    print 'Failed to create socket. Terminating.'
    sys.exit()

key_map = {
    '1':('bck',200),
    '2':('bck',400),
    '3':('bck',600),
    '4':('trn',-1),
    '5':('stp',0),
    '6':('trn',1),
    '7':('fwd',200),
    '8':('fwd',400),
    '9':('fwd',600),
}

while True:
    try :
        try:
            key = str(raw_input())
            msg = key_map[key]
        except:
            msg = (key,0)
        print 'Sending: ' + str(msg)

        s.sendto(pickle.dumps(msg), (ev3_host, ev3_port))
        data,addr = s.recvfrom(1024)
        reply = pickle.loads(data)
         
        print 'Server reply : ' + str(reply)
     
    except socket.error, msg:
        print 'Error Code : ' + str(msg[0]) + ' Message ' + msg[1]
        sys.exit()
