#!/usr/bin/env python
from ev3.ev3dev import Motor
import time

class EV3DiffDrive():
    def __init__(self, invert=False, *args, **kwargs):
        if invert:
            self.dl = Motor(port=Motor.PORT.B)
            self.dr = Motor(port=Motor.PORT.C)
        else:
            self.dl = Motor(port=Motor.PORT.C)
            self.dr = Motor(port=Motor.PORT.B)            

    def reset(self):
        self.dl.reset()
        self.dr.reset()

    def forward(self, speed=200):
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = speed

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = speed

        self.dl.start()
        self.dr.start()

    def backward(self, speed=200):
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = -speed

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = -speed

        self.dl.start()
        self.dr.start()

    def turn(self, speed_left=-200, speed_right=200):
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = speed_left

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = speed_right

        self.dl.start()
        self.dr.start()
        
    def brake(self):
        self.dl.stop()
        self.dr.stop()
        
    def encoder(self):
        return (self.dl.position, self.dr.position)


if __name__ == '__main__':
    r = EV3DiffDrive()
    r.reset()
    inp = get_input('>>')
    moves = {'w':r.forward,'s':r.backward,'a':r.turn,'d':r.turn,'b':r.brake} # Right turn is not possible in this test implementation
    while inp!='.':
        moves[inp]()
        inp = get_input('>>')
