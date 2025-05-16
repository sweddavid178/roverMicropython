# roverMicropython

 Steps to flash micropython from Terminal
1. Install esptool.py "pip install esptool" https://docs.espressif.com/projects/esptool/en/latest/esp32/
2. Erase flash "esptool erase_flash"
3. load code "esptool --baud 460800 write_flash 0x1000 ESP32_GENERIC-20241129-v1.24.1.bin" https://micropython.org/download/ESP32_GENERIC/

 Steps to flash micropython from Thonny
1. Set interpretter (Run -> Configure Interpreter)
2. change to "Micropython Esp32"
3. Install Micropython (esptool)
4. Select comport
5. Select Esp32
6. Select variant "Espressif Esp32/Wroom
7. Select Latest version
8. wait to finish
9. close interpreter window and Terminal should update with the micropython REPL

 Setting up Thonny IDE
1. Set interpretter (Run -> Configure Interpreter)
2. change to "Micropython Esp32"

 Loading Libraries
1. File -> Open -> this computer "ir_control.py" "rover.py"
2. For both files: file -> save copy -? "ir_control.py" "rover.py"


 Running ir_test.py
1. File -> Open -> this computer "ir_test.py"
2. hit green run triangle
3. device should periodically transmit and look for recieve code
