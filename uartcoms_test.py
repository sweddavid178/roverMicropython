import machine
import network
import time

# UART setup (change pins and baudrate as needed)
uart = machine.UART(1, baudrate=9600, tx=17, rx=18)  # Example pins for ESP32

# Get MAC address
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
mac = wlan.config('mac')
mac_str = ':'.join('{:02x}'.format(b) for b in mac)

send_interval = 1  # seconds
last_send = time.ticks_ms()

while True:
    # Periodically send MAC address
    if time.ticks_diff(time.ticks_ms(), last_send) > send_interval * 1000:
        uart.write('MAC: {}\n'.format(mac_str))
        print('Sent MAC address:', mac_str)
        last_send = time.ticks_ms()

    # Print any received UART data
    if uart.any():
        data = uart.read()
        if data:
            print('Received:', data)

    time.sleep(0.05)