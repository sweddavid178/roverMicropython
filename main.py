#Python imports
from rover import *
from ir_control import *
import time

#global variables
volcanoPeriod = 0

#main loop
while True:
    #drive control
    #drive(remote.right_x, remote.right_y)
    test_mode()
    
    time.sleep(0.05) #loop speed controller
