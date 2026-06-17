import time
import machine

# remote_io_test.py
# GitHub Copilot

# 1 = right joystick X
# 2 = right joystick Y
# 3 = left joystick X
# 6 = left joystick Y


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

def print_analog_values(pin_list=(3, 4, 1, 2), interval=1.0, iterations=None):
    """Print analog readings for the given GPIO pins.
    - pin_list: iterable of GPIO pin numbers
    - interval: seconds between prints
    - iterations: number of times to print; None for infinite
    """
    count = 0
    try:
        while iterations is None or count < iterations:
            readings = []
            for p in pin_list:
                raw, norm = _read_adc_value(p)
                if raw is None:
                    readings.append(f"GPIO{p}: ERROR")
                else:
                    readings.append(f"GPIO{p}: raw={raw} norm={norm:.3f}")
            print(", ".join(readings))
            count += 1
            time.sleep(interval)
    except KeyboardInterrupt:
        print("Stopped by user.")

if __name__ == "__main__":
    print("Starting analog value reader. Press Ctrl+C to stop.")
    # Example: run indefinitely, printing every second
    print_analog_values()