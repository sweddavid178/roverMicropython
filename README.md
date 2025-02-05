# roverMicropython

 Steps to flash
1. Install esptool.py "pip install esptool" https://docs.espressif.com/projects/esptool/en/latest/esp32/
2. Erase flash "esptool erase_flash"
3. load code "esptool --baud 460800 write_flash 0x1000 ESP32_GENERIC-20241129-v1.24.1.bin" https://micropython.org/download/ESP32_GENERIC/
