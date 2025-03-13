"""
March 13, 2025
The delivery drone (aka prototype) will receive one location coordinate through Python sockets.
Once received, drone arms and takkes off to location to delivery package.
"""

########DEPENDENCIES###############
import socket
import json 
import threading
from pymavlink import mavutil
from dronekit import connect, VehicleMode, LocationGlobalRelative
import time
import math
import argparse

########CONSTANTS##################
PACKAGE_ALT = 1 # in meters
DROP_ALT = .3 

########CONNECTION BETWEEN FLIGHT COMPUTER AND CONTROLLER#################
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
delivery_drone = connect(connection_string, wait_ready=True)

#######CONNECTION BETWEEN SCOUT AND DELIVERY DRONE#########
# sets up a UDP socket for communication indicating both the host and port
# delivery drone is client, scout is host.
HOST = '127.0.0.1'
PORT = 5001
server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
server_socket.bind((HOST, PORT))

########FUNCTIONS##################
def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not the_connection.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    delivery_drone.mode = VehicleMode("GUIDED")
    delivery_drone.armed = True

    while not the_connection.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    delivery_drone.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", delivery_drone.location.global_relative_frame.alt)      
        if delivery_drone.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def execute_mission(command):
    parts = command.split() # each command tokenized
    if parts[0] == "goto":
        arm_and_takeoff(PACKAGE_ALT) # low-flying
     
        lat, lon, alt = map(float, parts[1:])
        print(f"Delivery drone going to ({lat}, {lon})...")
        drop_location = LocationGlobalRelative(lat, lon, PACKAGE_ALT) 
        
        delivery_drone.simple_goto(drop_location) # go to location
        delivery_drone.simple_takeoff(DROP_ALT) # lower drone closer to the ground

        # drop the package

        delivery_drone.mode = VehicleMode("RTL") # go home after pacakge is dropped off
        print('Mission Success!')

        # let Scout know of mission success through socket connection

        delivery_drone.close()
        
# Message Listener
def listen_for_messages():
    # Delivery Drone will constantly be listening for messages from Scout
    while True:
        data, addr = server_socket.recvfrom(1024)
        command = data.decode()
        print(f"Received command: {command} from {addr}")
        execute_mission(command)

print('Package Delivery Mission')

listener_thread = threading.Thread(target=listen_for_messages, daemon=True)
listener_thread.start()

print('Waiting for Drop-off location...')
