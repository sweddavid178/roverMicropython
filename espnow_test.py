import network
import espnow

#temporary test code with python pyboard.py -d COM6 espnow_test.py

# Initialize WiFi in station mode
w0 = network.WLAN(network.STA_IF)
w0.active(True)

# Initialize ESP-NOW
e = espnow.ESPNow()
e.active(True)

# Print this device's MAC address
print("This device's MAC address:", ':'.join('{:02X}'.format(b) for b in w0.config('mac')))

# Add peer (replace with the MAC address of the peer device)
#rover's MAC address: E0:72:A1:B1:F5:40
#remote's MAC address: E0:72:A1:B1:F5:5C
this_device_mac = w0.config('mac')
peer_mac = b'\xE0\x72\xA1\xB1\xF5\x5C'  # Replace with the actual MAC address
if peer_mac == this_device_mac:
    peer_mac = b'\xE0\x72\xA1\xB1\xF5\x40'  # Use the other device's MAC if it's the same as this one
e.add_peer(peer_mac)

# Send a message
e.send(peer_mac, b'Hello ESP-NOW!')

# Receive messages (non-blocking)
while True:
    host, msg = e.irecv()
    if msg:
        print('Received from {}: {}'.format(host, msg))