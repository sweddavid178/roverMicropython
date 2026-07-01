#Python imports
from rover import *
from ir_control import *
import time

#global variables
volacanoPeriod = 0

#main loop
while True:
    #drive control
    drive(remote.right_x, remote.right_y)

    #if trigger is held
    if remote.right_trigger_btn_held():
        setServoAngle(0)
        #IR_send_message(20,44)
    else:
        setServoAngle(45)  

    #volcano challenge
    if remote.left_down_btn_held() == 1:
        volacanoPeriod = getLightSensorPeriod()
        print("volcano period: " + str(volacanoPeriod))
    if remote.right_middle_btn_pressed() == True:
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
    if remote.left_trigger_btn_pressed() == True:
        IR_send_message(100,100)
    
    
    time.sleep(0.05) #loop speed controller

