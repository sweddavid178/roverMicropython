import subprocess
import sys
import time

import serial.tools.list_ports

def find_com_port():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No COM ports found.")
        sys.exit(1)
    return ports[0].device
com_port = find_com_port()
print(f"Using COM port: {com_port}")

commands = [
    ["esptool", "erase-flash"],
    ["esptool", "--after=watchdog-reset", "--baud", "460800", "write-flash", "0", "ESP32_GENERIC_S3-20260406-v1.28.0.bin"],
]
for cmd in commands:
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"Command failed: {' '.join(cmd)}")
        sys.exit(result.returncode)
    time.sleep(2)  # Wait for 2 seconds between commands
    com_port = find_com_port()  # Re-find the COM port after each command

print("All commands completed successfully.")