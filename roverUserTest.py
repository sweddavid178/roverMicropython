from rover import joystick, drive, setServoAngle, set_internal_led
from ir_control import IR_send_message, IR_get_last_rx_message
import time

while True:
    drive(joystick.x, joystick.y)
    if joystick.trigger == 0:
        setServoAngle(45)
        set_internal_led(0,0,0)
        
    else:
        setServoAngle(0)
        set_internal_led(100,100,100)
        IR_send_message(20,44)
    time.sleep(0.05)