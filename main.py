#Python imports
from rover import joystick, drive, setServoAngle, set_internal_led, getLightSensorPeriod
from ir_control import IR_send_message, IR_get_last_rx_message
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
    if joystick.trigger == True:
        setServoAngle(0)
        set_internal_led(100,100,100)
        IR_send_message(20,44)
    else:
        setServoAngle(45)
        set_internal_led(0,0,0)   

    #volcano challenge
    if joystick.btnA == True:
        volacanoPeriod = getLightSensorPeriod()
    if joystick.btnAPressed() == True:
        IR_send_message(20,volacanoPeriod) 
        
    #ir challenge
    irData, irAddress = IR_get_last_rx_message()
    if irAddress != -1:
        decodedData = irData % 3
        if decodedData == 0:
            set_external_led(100,0,0)
        elif decodedData == 1:
            set_external_led(0,100,0)
        elif decodedData == 2:
            set_external_led(0,0,100)
    
    #pressed is only true once
    if joystick.btnBPressed() == 0:
        IR_send_message(20,44)
    
    
    time.sleep(0.05) #loop speed controller
