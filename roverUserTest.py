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

    #if trigger is held
    if joystick.trigger:
        setServoAngle(0)
        #IR_send_message(20,44)
    else:
        setServoAngle(45)  

    #volcano challenge
    if joystick.btnA == 1:
        volacanoPeriod = getLightSensorPeriod()
        print("volcano period: " + str(volacanoPeriod))
    if joystick.btnXPressed() == True:
        IR_send_message(200,volacanoPeriod) 
        
    #ir challenge
    irData, irAddress = IR_get_last_rx_message()
    if irAddress != -1:
        print(irData)
        decodedData = irData
        if decodedData == 0:
            set_internal_led(100,0,0)
        elif decodedData == 1:
            set_internal_led(0,100,0)
        elif decodedData == 2:
            print("setting blue")
            set_internal_led(0,0,100)
    
    #pressed is only true once
    if joystick.btnBPressed() == True:
        IR_send_message(100,100)
    
    
    time.sleep(0.05) #loop speed controller

