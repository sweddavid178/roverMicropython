#rename this file to main.py when running on the remote
import time
import machine
import network
import espnow

# UART setup (change pins and baudrate as needed)
uart = machine.UART(1, baudrate=9600, tx=17, rx=18)  # pins for ESP32s3

# Get MAC address
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
mac = wlan.config('mac')
mac_str = ':'.join('{:02x}'.format(b) for b in mac)

# analog id -> human name
ANALOG_MAP = {
    1:  "right_x",   # right joystick X
    2:  "right_y",   # right joystick Y
    3:  "left_x",    # left joystick X
    6:  "left_y",    # left joystick Y
    10: "voltage",   # battery voltage / analog sensor
}

# button id -> human name
BUTTON_MAP = {
    10: "right_trigger",
    7:  "left_trigger",
    5:  "left_button",
    8:  "right_button",
    9:  "btn0_left_down",
    11: "btn2",
    12: "btn3",
    13: "btn4",
    14: "btn5",
    15: "btn6_left_up",
    16: "btn7",
    46: "left_middle",
    40: "right_trigger",
    37: "right_up",
    36: "right_middle",
    35: "right_down",
    39: "right_thumb",
    47: "left_thumb",
}

def _read_adc_value(pin_no):
    """Try to read an analog value from the given GPIO pin number.
    Returns a tuple (raw, normalized) or (None, None) on error.
    """
    try:
        adc = machine.ADC(machine.Pin(pin_no))
    except Exception as e:
        # Could not construct ADC for that pin
        return None, None

    # MicroPython ports differ: try read_u16(), otherwise read()
    try:
        raw = adc.read_u16()
        # read_u16 returns 0..65535
        norm = raw / 65535.0
    except AttributeError:
        try:
            raw = adc.read()  # common on some ports (0..4095)
            norm = raw / 4095.0 if raw is not None else None
        except Exception:
            return None, None

    return raw, norm

def read_all_analog_values(analog_map):
    """Read all analog values from the given map of id -> name.
    Returns a string of comma-separated name=value pairs.
    """
    readings = ""
    for aid, name in analog_map.items():
        raw, norm = _read_adc_value(aid)
        if raw is not None:
            readings += f"{norm:.3f},"
        else:
            readings += f"0.000,"  # or "ERROR" if you prefer
    return readings

def read_all_button_states(button_map):
    """Read all button states from the given map of id -> name.
    Returns a string of comma-separated names for pressed buttons.
    """
    readings = ""
    for bid, name in button_map.items():
        try:
            pin = machine.Pin(bid, machine.Pin.IN, machine.Pin.PULL_UP)
            readings += f"{pin.value()},"
            
        except Exception:
            pass  # ignore errors for missing pins
    return readings

def send_mac_address(uart, mac_str):
    """Send the MAC address over UART."""
    uart.write('MAC: {}\n'.format(mac_str))
    print('Sent MAC address:', mac_str)

def save_mac_address_to_file(filename, mac_string):
    """Save the MAC address string to a file."""
    try:
        with open(filename, 'w') as f:
            f.write(mac_string)
    except Exception as e:
        print('Failed to save MAC address:', e)


def mac_string_to_bytes(mac_string):
    """Convert a colon-separated MAC string to a bytes object."""
    try:
        parts = mac_string.split(':')
        # Expect 6 parts of two hex characters each
        parts = [p.strip() for p in parts if p.strip() != '']
        print('Parsed MAC parts:', parts)
        vals = []
        for p in parts:
            if len(p) > 2 or len(p) < 1:
                # if a part is longer (e.g. '0a'), allow parsing but still require <=2
                # pad or trim would be unsafe; raise error
                raise ValueError('Invalid MAC part length')
            vals.append(int(p, 16))
        return bytes(vals)
    except Exception as e:
        print('Failed to parse MAC string:', e)
        raise


def check_for_mac_address(uart):
    """Check for incoming UART data and print it if it contains a MAC address."""
    if uart.any():
        data = uart.read()
        if data:
            print('Received:', data)
            
            if b'MAC:' in data and b'\n' in data:  # basic sanity check for MAC address line
                mac_line = data.decode().strip()
                # remove possible leading 'MAC:' prefix
                if mac_line.upper().startswith('MAC:'):
                    mac_line = mac_line.split(':', 1)[1].strip()

                print('Received MAC address:', mac_line)
                save_mac_address_to_file('received_mac.txt', mac_line)
                return True
    return False

def load_saved_mac_address():
    """Load the saved MAC address from a file."""
    try:
        with open('received_mac.txt', 'r') as f:
            mac_string = f.read().strip()
            mac_bytes = mac_string_to_bytes(mac_string)  # Validate and convert to bytes
            if (len(mac_bytes) != 6):
                return None  # Invalid MAC address length
            print('Loaded saved MAC address:', mac_bytes)
            return mac_bytes
    except Exception as e:
        print('Failed to load MAC address:', e)
        return None


def clear_all_peers(esp):
    """Remove all ESP-NOW peers from the given ESPNow object."""
    try:
        for peer in esp.peers():
            try:
                esp.remove_peer(peer)
            except Exception:
                pass
    except Exception:
        pass

if __name__ == "__main__":

    try:
        # Initialize ESP-NOW
        e = espnow.ESPNow()
        e.active(True)

        mac_bytes = load_saved_mac_address()  # Load the saved MAC address at startup
        if mac_bytes:
            print('Using saved MAC address:', mac_bytes)
            e.add_peer(mac_bytes)  # Add the saved MAC address as a peer

        while True:
            buttons = read_all_button_states(BUTTON_MAP)
            analogs = read_all_analog_values(ANALOG_MAP)
            out = buttons + analogs 
            print(out)
            #send_mac_address(uart, mac_str)
            if check_for_mac_address(uart):
                mac_bytes = load_saved_mac_address()  # Reload the MAC address after receiving it
                if mac_bytes:
                    print('Using new saved MAC address:', mac_bytes)
                    clear_all_peers(e)  # Clear existing peers before adding the new one
                    e.active(False)  # Deactivate ESP-NOW to reset peer list
                    e.active(True)   # Reactivate ESP-NOW to apply changes
                    e.add_peer(mac_bytes)  # Add the new MAC address as a peer
            if mac_bytes:
                try:
                    e.send(mac_bytes, out.encode())  # Send the combined button and analog data to the peer
                except Exception as err:
                    print('Failed to send data:', err)

            time.sleep(0.1)
    except KeyboardInterrupt:
        pass