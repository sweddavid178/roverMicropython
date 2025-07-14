from rover import set_internal_led, set_external_led, setServoAngle, drive
from time import sleep
from ir_control import IR_get_last_rx_message, irTransmit, IR_send_message

def test_internal_led():
    for x in range(2):
        set_internal_led(100,0,0)
        sleep(.5)
        set_internal_led(0,100,0)
        sleep(.5)
        set_internal_led(0,0,100)
        sleep(.5)
       
def test_external_led():
    for x in range(2):
        set_external_led(100,0,0)
        sleep(.5)
        set_external_led(0,100,0)
        sleep(.5)
        set_external_led(0,0,100)
        sleep(.5)
       
def test_servo():
    setServoAngle(0)
    sleep(.5)
    setServoAngle(90)
    sleep(.5)
    setServoAngle(0)
   
def test_drive_motors():
    for x in range(5):
        drive(0,100)
        sleep(0.2)
    sleep(2)
    for x in range(10):
        drive(127,127)
        sleep(0.2)

def test_ir_tx_rx():
    for x in range(5):
        IR_send_message(20,50)
        sleep(.5)
 

   
   
#test_internal_led()
#test_external_led()
#test_ir_tx_rx()
test_servo()
test_drive_motors()

sleep(1)

# Turn off leds to indicate the test is done
set_internal_led(0,0,0)
set_external_led(100,100,100)