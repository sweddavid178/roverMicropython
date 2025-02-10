from machine import Pin, PWM
import time
import math
import pyb

U16 = 65535
print("motor test")
#left pins 13, 5

def to_u16(value) -> int:
    return min(max(math.floor(value / 100 * U16), 0), U16)

lPin = PWM(Pin(5))
lPin.init(freq=1_000, duty_u16=0)
lPin.duty_u16(to_u16(50))
statusPin = Pin(22)
statusPin.init(mode=Pin.OUT, value=0)
servo1 = pyb.Servo(23)

while(True):
    print("test")
    servo1.angle(-45)
    statusPin.value(1)
    time.sleep(1)
    servo1.angle(60)
    statusPin(0)
    time.sleep(1)
    