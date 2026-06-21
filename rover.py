import bluetooth
import struct
import time
from micropython import const
from ubinascii import hexlify
from machine import Pin, PWM, Timer, UART
import math
import network
import espnow

count = 0
lightPeriod = 0
lightSensor = Pin(17, Pin.IN)
lastSensor = 0
internalR = 0
internalG = 0
internalB = 0
ledCount = 0
ledState = False

#motor stuff
U16 = 65535
def to_u16(value) -> int:
    return min(max(math.floor(value / 127 * U16), 0), U16)
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)

#pwm led stuff
#int_red_pin = Pin(25)
int_red_pwm = PWM(Pin(16), freq=1_000, duty_u16=U16)
int_green_pwm = PWM(Pin(15), freq=1_000, duty_u16=U16)
int_blue_pwm = PWM(Pin(7), freq=1_000, duty_u16=U16)

def m_set_internal_led(red,green,blue):
    r = U16 - min(max(math.floor(red / 100 * U16), 0), U16)
    int_red_pwm.duty_u16(r)
    g = U16 - min(max(math.floor(green / 100 * U16), 0), U16)
    int_green_pwm.duty_u16(g)
    b = U16 - min(max(math.floor(blue / 100 * U16), 0), U16)
    int_blue_pwm.duty_u16(b)

def blinkControl():
    global internalR, internalG, internalB, ledCount, ledState
    period = 0
    ledCount += 1
    if internalR == 0 and internalG == 0 and internalB == 0:
        m_set_internal_led(0, 0, 0)  # Red
    if internalR >= internalG and internalR >= internalB:
        period = 10
    elif internalG >= internalR and internalG >= internalB:
        period = 5
    elif internalB >= internalR and internalB >= internalG:
        period = 2
    else:
        period = 0
    if ledCount % period == 0:
        ledState = not ledState
    if ledState:
        m_set_internal_led(internalR, internalG, internalB)
    else:
        m_set_internal_led(0, 0, 0)

# Callback function for the timer
def measure_light(timer):
    global count, lightPeriod, lightSensor, lastSensor, internalR, internalG, internalB, ledCount
    currentValue = lightSensor.value()
    if (currentValue == 1):
        count += 5 #increment count by 50ms
    
    if lastSensor == 1 and currentValue == 0:
        if lightPeriod == 0:
            lightPeriod = count
        else:
            lightPeriod = round(count*2 * 0.5 + lightPeriod * 0.5) #filter results for more consistancy
    if currentValue == 0:
        count = 0
    
    lastSensor = currentValue
    blinkControl()  # Call the blink control function
    
# Create a periodic timer
light_timer = Timer(1)
light_timer.init(mode=Timer.PERIODIC, period=50, callback=measure_light)  # Timer repeats every half second

# Get MAC address
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
mac = wlan.config('mac')
mac_str = ':'.join('{:02x}'.format(b) for b in mac)

def send_mac_address(uart, mac_str):
    """Send the MAC address over UART."""
    uart.write('MAC: {}\n'.format(mac_str))
    #print('Sent MAC address:', mac_str)

# Create a periodic timer for sending mac address
# UART setup (change pins and baudrate as needed)
uart = UART(1, baudrate=9600,tx=43,rx=44)  # Example pins for ESP32
mac_timer = Timer(2)
mac_timer.init(mode=Timer.PERIODIC, period=1000, callback=lambda t: send_mac_address(uart, mac_str))




left_LPin = PWM(Pin(13), freq=1_000, duty_u16=0)
left_RPin = PWM(Pin(5), freq=1_000, duty_u16=0)
right_LPin = PWM(Pin(16), freq=1_000, duty_u16=0)
right_RPin = PWM(Pin(4), freq=1_000, duty_u16=0)

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

class ESPJoystick:
    def __init__(self):
        self.conn_handle = None
        self.joystick_service = None
        self.joystick_char = None
        self.found_device = None
        self.scanning = False
        self.x = 128
        self.y = 128
        self.trigger = False
        self.btnA = False
        self.btnB = False
        self.btnX = False
        self.btnY = False
        self.lastTrigger = False
        self.lastBtnA = False
        self.lastBtnB = False
        self.lastBtnX = False
        self.lastBtnY = False
        self.addr = b''

    def setMacAddress(self, addr):
        self.addr = addr
        self.start_scan()

    def handle_joystick_input(self, data):
        if len(data) >= 2:
            # data = b'008000800080008000000000090000'
            #forward 82820000
            #right ffff0080
            #left 00007b7b
            #backward 8c8cffff
            self.btnA = (data[7] & 1 != 0)
            self.btnB = (data[7] & 2 != 0)
            self.btnX = (data[7] & 4 != 0)
            self.btnY = (data[7] & 8 != 0)
            self.trigger = (data[8] & 8 != 0)
            self.x = data[2] #0-255, 128 is stop
            self.y = data[3] #0-255, 128 is stop
            print(f"Joystick X: {self.x}, Y: {self.y}, a {self.btnA},b {self.btnB},x {self.btnX},y {self.btnY}, trg {self.trigger}  {hexlify(data)}")

    def btnAPressed(self):
        ret = False
        if self.btnA == True and self.lastBtnA == False:
            ret = True
        self.lastBtnA = self.btnA
        return ret
    def btnBPressed(self):
        ret = False
        if self.btnB == True and self.lastBtnB == False:
            ret = True
        self.lastBtnB = self.btnB
        return ret
    def btnXPressed(self):
        ret = False
        if self.btnX == True and self.lastBtnX == False:
            ret = True
        self.lastBtnX = self.btnX
        return ret
    def btnYPressed(self):
        ret = False
        if self.btnY == True and self.lastBtnY == False:
            ret = True
        self.lastBtnY = self.btnY
        return ret
    def triggerPressed(self):
        ret = False
        if self.trigger == True and self.lastTrigger == False:
            ret = True
        self.lastTrigger = self.trigger
        return ret  

    def start_scan(self):
        print("Scanning for joystick...")
        self.scanning = True
        self.ble.gap_scan(5000, 30000, 30000)  # Scan for 5 seconds


lastLeft = 0
lastRight = 0
def drive(x,y):
    global lastLeft, lastRight
    UPPER_DEADBAND = 3
    LOWER_DEADBAND = -3
    ALPHA = 0.3
    leftVal = clamp((y+x)-255, -127, 127)
    leftVal = leftVal*ALPHA + lastLeft*(1-ALPHA)
    lastLeft = leftVal
    rightVal = clamp(((y-127)-(x-127)), -127, 127)
    rightVal = rightVal*ALPHA + lastRight*(1-ALPHA)
    lastRight = rightVal
    if leftVal > UPPER_DEADBAND:
        left_RPin.duty_u16(to_u16(0))
        left_LPin.duty_u16(to_u16(leftVal))
    elif leftVal < LOWER_DEADBAND:
        left_LPin.duty_u16(to_u16(0))
        left_RPin.duty_u16(to_u16(leftVal*-1))
    else:
        left_LPin.duty_u16(to_u16(0))
        left_RPin.duty_u16(to_u16(0))
    if rightVal > UPPER_DEADBAND:
        right_RPin.duty_u16(to_u16(0))
        right_LPin.duty_u16(to_u16(rightVal))
    elif rightVal < LOWER_DEADBAND:
        right_LPin.duty_u16(to_u16(0))
        right_RPin.duty_u16(to_u16(rightVal*-1))
    else:
        right_LPin.duty_u16(to_u16(0))
        right_RPin.duty_u16(to_u16(0)) 
        
#servo stuff
#servo1 = pyb.Servo(23)
# Set up PWM Pin for servo control
servo_pin = Pin(14)
servo = PWM(servo_pin)

# Set Duty Cycle for Different Angles
max_duty = 7864
min_duty = 1802
half_duty = int(max_duty/2)

#Set PWM frequency
frequency = 50
servo.freq (frequency)

def setServoAngle(angle):
    output = int((angle / 180) * (max_duty - min_duty)) + min_duty
    servo.duty_u16(output)

#input values are 0-100, with 100 being max brightness
def set_internal_led(red,green,blue):
    global internalR, internalG, internalB
    internalR = red
    internalG = green
    internalB = blue
    #m_set_internal_led(internalR, internalG, internalB)
    
#ext_red_pwm = PWM(Pin(8), freq=1_000, duty_u16=U16)
#ext_green_pwm = PWM(Pin(18), freq=1_000, duty_u16=U16)
#ext_blue_pwm = PWM(Pin(27), freq=1_000, duty_u16=U16)

#input values are 0-100, with 100 being max brightness
def set_external_led(red,green,blue):
    r = U16 - min(max(math.floor((1-red / 100) * U16), 0), U16)
    ext_red_pwm.duty_u16(r)
    g = U16 - min(max(math.floor((1-green / 100) * U16), 0), U16)
    ext_green_pwm.duty_u16(g)
    b = U16 - min(max(math.floor((1-blue / 100) * U16), 0), U16)
    ext_blue_pwm.duty_u16(b)
#set_external_led(0,0,0)
joystick = ESPJoystick()
#joystick.start_scan()

# ESP-NOW setup
esp = espnow.ESPNow()
esp.active(True)

def handle_espnow_message(peer, msg):
    try:
        print('ESPNow from', hexlify(peer), msg)
        # add message handling logic here, e.g. parse commands
    except Exception as e:
        print('Error handling espnow msg', e)

def check_espnow(timer):
    print('trying esp')
    try:
        res = esp.recv()  # non-blocking
        
        if res:
            peer, msg = res
            handle_espnow_message(peer, msg)
    except Exception as e:
        # recv may raise if not ready; ignore
        pass

# periodic timer to poll espnow messages
espnow_timer = Timer(3)
espnow_timer.init(mode=Timer.PERIODIC, period=100, callback=check_espnow)

def getLightSensorPeriod():
    global lightPeriod
    return lightPeriod
