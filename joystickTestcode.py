import bluetooth
import struct
import time
from micropython import const
from ubinascii import hexlify

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

    def ble_irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            print("test: ", hexlify(addr), " data: ", str(adv_data))
            if b'd0547b4c4b89' == hexlify(addr):  # Check if advertisement contains 'Joystick'
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
            btn = data[13]
            trigger = data[11]
            x = data[1] #0-255, 128 is stop
            y = data[3] #0-255, 128 is stop
            print(f"Joystick X: {x}, Y: {y}, btn {btn}, trg {trigger} {hexlify(data)}")

    def start_scan(self):
        print("Scanning for joystick...")
        self.scanning = True
        self.ble.gap_scan(5000, 30000, 30000)  # Scan for 5 seconds

joystick = BLEJoystick()
joystick.start_scan()

while True:
    time.sleep(1)
