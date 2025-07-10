#Python imports
from rover import *
from ir_control import *
import time

#setup and start joystick
joystick.setMacAddress(b'd0547b4c4b89')

#global variables
volacanoPeriod = 0

#main loop
while True:
    #drive control
    drive(joystick.x, joystick.y)
    
    time.sleep(0.05) #loop speed controller
