#Python imports
from rover import set_internal_led, getLightSensorPeriod, set_external_led
from ir_control import IR_send_message, IR_get_last_rx_message
import time

IR_send_message(100,100)
#main loop
while True:
    
    
    
    #ir challenge
    irData, irAddress = IR_get_last_rx_message()
    
    if irAddress != -1:
        print("got message")
        decodedData = irData % 3
        if decodedData == 0:
            set_internal_led(100,0,0)
        elif decodedData == 1:
            set_internal_led(0,100,0)
        elif decodedData == 2:
            set_internal_led(0,0,100)
            
        if irAddress == 0:
            IR_send_message(100,irData+5)
        elif irAddress == 1:
            IR_send_message(100,irData-7)
        elif irAddress == 2:
            IR_send_message(100,irData+28)
        elif irAddress == 3:
            IR_send_message(100,irData-50)
        elif irAddress == 4:
            IR_send_message(100,irData+64)
    

    
    
    
    time.sleep(0.5) #loop speed controller
