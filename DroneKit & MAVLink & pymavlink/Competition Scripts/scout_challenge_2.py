"""
Scout Challenge 2
# Send location to package delivery drone (prototype)
# Add lissajous waypoints later, test communications python sockets now since two drones aren't available

The prototype will act as Delivery drone, and a separate flight computer
will send commands, and not the GCS like the Scout drone will.
"""

import socket
import time
from pymavlink import mavutil

#Sets up UDP socket for sending messages
HOST = '127.0.0.1'
PORT = 5001
client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def send_command(command):
    client_socket.sendto(command.encode(), (HOST, PORT))
    print(f"Sent command to Delivery drone: {command}")

print("Scout is starting...")
print("Scout sent its location")
time.sleep(120) # wait until package drone script is initialized

# Add lissajous search, takeoff, and visual processing code
send_command("goto 47.397748 8.545596 20") # Change coordinates at field, later this can be scout.location() 
