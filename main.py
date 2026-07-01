#Python imports
from rover import *
from ir_control import *
import time

#global variables
volacanoPeriod = 0

#main loop
while True:
    #drive control
    #drive(joystick.x, joystick.y)
    test_mode()
    
    time.sleep(0.05) #loop speed controller
