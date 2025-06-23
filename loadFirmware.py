import subprocess
import sys
import time

import serial.tools.list_ports

ports = list(serial.tools.list_ports.comports())
if not ports:
    print("No COM ports found.")
    sys.exit(1)
com_port = ports[0].device
print(f"Using COM port: {com_port}")

commands = [
    ["esptool", "erase_flash"],
    ["esptool", "--baud", "460800", "write_flash", "0x1000", "ESP32_GENERIC-20250415-v1.25.0.bin"],
    ["python", "pyboard.py", "--device", com_port, "-f", "cp", "rover.py", ":rover.py"],
    ["python", "pyboard.py", "--device", com_port, "-f", "cp", "ir_control.py", ":ir_control.py"],
    ["python", "pyboard.py", "--device", com_port, "-f", "cp", "main.py", ":main.py"]
]

for cmd in commands:
    print(f"Running: {' '.join(cmd)}")
    result = subprocess.run(cmd)
    if result.returncode != 0:
        print(f"Command failed: {' '.join(cmd)}")
        sys.exit(result.returncode)
    time.sleep(2)  # Wait for 2 seconds between commands

print("All commands completed successfully.")