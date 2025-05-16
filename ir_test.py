#Python imports
from rover import set_internal_led, getLightSensorPeriod
from ir_control import IR_send_message, IR_get_last_rx_message
import time


#main loop
while True:
        
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
    

    IR_send_message(20,44)
    
    
    time.sleep(0.5) #loop speed controller
