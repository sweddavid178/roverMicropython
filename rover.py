import bluetooth
import struct
import time
from micropython import const
from ubinascii import hexlify
from machine import Pin, PWM
import math

left_LPin = PWM(Pin(13), freq=1_000, duty_u16=0)
left_RPin = PWM(Pin(5), freq=1_000, duty_u16=0)
right_LPin = PWM(Pin(16), freq=1_000, duty_u16=0)
right_RPin = PWM(Pin(4), freq=1_000, duty_u16=0)

_ADV_TYPE_FLAGS = const(0x01)
_ADV_TYPE_NAME = const(0x09)
_ADV_TYPE_UUID16_COMPLETE = const(0x03)

def advertising_payload(name=None, services=None, appearance=0):
    payload = bytearray()

    if name:
        name_bytes = name.encode()
        payload += struct.pack("BB", len(name_bytes) + 1, _ADV_TYPE_NAME) + name_bytes

    if services:
        for uuid in services:
            b = struct.pack("<H", uuid)
            payload += struct.pack("BB", len(b) + 1, _ADV_TYPE_UUID16_COMPLETE) + b

    return payload

def decode_services(data):
    services = []
    i = 0
    while i < len(data):
        length = data[i]
        if length == 0:
            break
        type_ = data[i + 1]
        value = data[i + 2 : i + length + 1]
        if type_ == _ADV_TYPE_UUID16_COMPLETE:
            services.append(struct.unpack("<H", value)[0])
        i += length + 1
    return services

_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_GATTC_SERVICE_RESULT = const(9)
_IRQ_GATTC_CHARACTERISTIC_RESULT = const(11)
_IRQ_GATTC_WRITE_DONE = const(17)
_IRQ_GATTC_NOTIFY = const(18)

class BLEJoystick:
    def __init__(self):
        self.ble = bluetooth.BLE()
        self.ble.active(True)
        self.ble.irq(self.ble_irq)
        self.conn_handle = None
        self.joystick_service = None
        self.joystick_char = None
        self.found_device = None
        self.scanning = False
        self.x = 128
        self.y = 128
        self.trigger = 0
        self.btnA = 0
        self.btnB = 0
        self.btnX = 0
        self.btnY = 0
        self.addr = b''

    def setMacAddress(self, addr):
        self.addr = addr
        self.start_scan()

    def ble_irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            print("test: ", hexlify(addr), " data: ", str(adv_data))
            if self.addr == hexlify(addr):  # Check if advertisement contains 'Joystick'
                print("Found Joystick at", hexlify(addr))
                self.found_device = (addr_type, addr)
                self.ble.gap_scan(None)  # Stop scanning
                self.ble.gap_connect(addr_type, addr)

        elif event == _IRQ_SCAN_COMPLETE:
            print("Scan complete.")

        elif event == _IRQ_GATTC_SERVICE_RESULT:
            conn_handle, start_handle, end_handle, uuid = data
            if uuid == bluetooth.UUID("1812"):  # HID Service UUID
                self.joystick_service = (start_handle, end_handle)
                self.ble.gattc_discover_characteristics(conn_handle, start_handle, end_handle)

        elif event == _IRQ_GATTC_CHARACTERISTIC_RESULT:
            conn_handle, def_handle, value_handle, properties, uuid = data
            if uuid == bluetooth.UUID("2A4D"):  # HID Report Characteristic UUID
                self.joystick_char = value_handle
                print("Joystick Characteristic found.")
                self.conn_handle = conn_handle
                self.ble.gattc_write(conn_handle, value_handle, b"\x01", 1)  # Enable notifications

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            self.handle_joystick_input(notify_data)

    def handle_joystick_input(self, data):
        if len(data) >= 2:
            # data = b'008000800080008000000000090000'
            #forward 82820000
            #right ffff0080
            #left 00007b7b
            #backward 8c8cffff
            self.btn = data[7]
            self.trigger = data[8]
            self.x = data[2] #0-255, 128 is stop
            self.y = data[3] #0-255, 128 is stop
            print(f"Joystick X: {self.x}, Y: {self.y}, btn {self.btn}, trg {self.trigger} {hexlify(data)}")

    def btnAPressed(self):
        return 0
    def btnBPressed(self):
        return 0
    def btnXPressed(self):
        return 0
    def btnYPressed(self):
        return 0    

    def start_scan(self):
        print("Scanning for joystick...")
        self.scanning = True
        self.ble.gap_scan(5000, 30000, 30000)  # Scan for 5 seconds

#motor stuff
U16 = 65535
def to_u16(value) -> int:
    return min(max(math.floor(value / 127 * U16), 0), U16)
def clamp(n, minn, maxn):
    return max(min(maxn, n), minn)


UPPER_DEADBAND = 3
LOWER_DEADBAND = -3

def drive(x,y):
    leftVal = clamp((y+x)-255, -127, 127)
    rightVal = clamp(((y-127)-(x-127)), -127, 127)
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
servo_pin = Pin(23)
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
    
#pwm led stuff
#int_red_pin = Pin(25)
int_red_pwm = PWM(Pin(25), freq=1_000, duty_u16=U16)
int_green_pwm = PWM(Pin(33), freq=1_000, duty_u16=U16)
int_blue_pwm = PWM(Pin(32), freq=1_000, duty_u16=U16)

#input values are 0-100, with 100 being max brightness
def set_internal_led(red,green,blue):
    r = U16 - min(max(math.floor(red / 100 * U16), 0), U16)
    int_red_pwm.duty_u16(r)
    g = U16 - min(max(math.floor(green / 100 * U16), 0), U16)
    int_green_pwm.duty_u16(g)
    b = U16 - min(max(math.floor(blue / 100 * U16), 0), U16)
    int_blue_pwm.duty_u16(b)
    
ext_red_pwm = PWM(Pin(14), freq=1_000, duty_u16=U16)
ext_green_pwm = PWM(Pin(27), freq=1_000, duty_u16=U16)
ext_blue_pwm = PWM(Pin(26), freq=1_000, duty_u16=U16)

#input values are 0-100, with 100 being max brightness
def set_external_led(red,green,blue):
    r = U16 - min(max(math.floor(red / 100 * U16), 0), U16)
    ext_red_pwm.duty_u16(r)
    g = U16 - min(max(math.floor(green / 100 * U16), 0), U16)
    ext_green_pwm.duty_u16(g)
    b = U16 - min(max(math.floor(blue / 100 * U16), 0), U16)
    ext_blue_pwm.duty_u16(b)

joystick = BLEJoystick()
#joystick.start_scan()

def getLightSensorPeriod():
    return 0
