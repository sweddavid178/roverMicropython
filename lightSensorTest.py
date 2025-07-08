from machine import Pin
from machine import Timer
import time

count = 0
lightPeriod = 0
lightSensor = Pin(17, Pin.IN)
lastSensor = 0

# Callback function for the timer
def measure_light(timer):
    global count, lightPeriod, lightSensor, lastSensor
    currentValue = lightSensor.value()
    if (currentValue == 1):
        count += 1
    
    if lastSensor == 1 and currentValue == 0:
        if lightPeriod == 0:
            lightPeriod = count
        else:
            lightPeriod = round(count * 0.3 + lightPeriod * 0.7) #filter results for more consistancy
    if currentValue == 0:
        count = 0
    lastSensor = currentValue

# Create a periodic timer
light_timer = Timer(1)
light_timer.init(mode=Timer.PERIODIC, period=100, callback=measure_light)  # Timer repeats every half second

# create an output pin on pin #0


# reconfigure pin #0 in input mode with a pull down resistor
#p0.init(p0.IN, p0.PULL_DOWN)
while True:
    print(lightSensor.value())
    
    print("count: ", lightPeriod)
    time.sleep(0.1)