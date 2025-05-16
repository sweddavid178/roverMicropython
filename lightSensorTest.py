from machine import Pin
# create an output pin on pin #0
p0 = Pin(17, Pin.IN, Pin.PULL_DOWN)

# reconfigure pin #0 in input mode with a pull down resistor
#p0.init(p0.IN, p0.PULL_DOWN)
while True:
    print(p0.value())