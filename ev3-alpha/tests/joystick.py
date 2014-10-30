from ev3.ev3dev import Motor
import time
from util import get_input

class DriveMotor():
    def __init__(self,*args, **kwargs):
        self.dl = Motor(port=Motor.PORT.B)
        self.dr = Motor(port=Motor.PORT.C)
        self.dm = Motor(port=Motor.PORT.A)

    def reset(self):
        self.dl.reset()
        self.dr.reset()
        self.dm.reset()

    def move_forward(self):
        
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = 200
        self.dl.start()

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = 200
        self.dr.start()
        
        print 'move_forward'

    def move_backward(self):
        
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = -200
        self.dl.start()

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = -200
        self.dr.start()
        
        print 'move_backward'

    def turn_left(self):
        
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = -200
        self.dl.start()

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = 200
        self.dr.start()
        
        print 'turn_left'

    def turn_right(self):
        
        self.dl.run_mode = 'forever'
        self.dl.regulation_mode = True
        self.dl.pulses_per_second_sp = 200
        self.dl.start()

        self.dr.run_mode = 'forever'
        self.dr.regulation_mode = True
        self.dr.pulses_per_second_sp = -200
        self.dr.start()
        
        print 'turn_right'

    def brake(self):
        
        self.dl.stop()
        self.dr.stop()
        
        print 'brake'

if __name__ == '__main__':
    a = DriveMotor()
    a.reset()
    inp = get_input('>>')
    moves = {'w':a.move_forward,'s':a.move_backward,'a':a.turn_left,'d':a.turn_right,'b':a.brake}
    while inp!='.':
        moves[inp]()
        inp = get_input('>>')
