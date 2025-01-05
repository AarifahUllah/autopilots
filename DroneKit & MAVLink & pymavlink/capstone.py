"""
script to test out DroneKit functions and control drone movements
based on DroneKit documentation and Whozchillin project

Example Scout drone program

Looks like we can specify where to connect both drones for communication
passing using function parameters e.g. msg_send(UDP_addr), msg_receive(UDP_addr)

Questions:
1. On competition day, once we see where the ArUco markers are, can we preconfigure 
scripts with their locations. (determine their GPS coordinates beforehand?)
2. In Gazebo simulated world, can you find GPS coordinates and points
"""

####### DEPENDENCIES ########
from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil


####### CONSTANTS ########
EARTH_RADIUS = 6371010.0 
EARTH_RADIUS_spherical = 6378137.0

# ArUco marker locations
marker_locations = {
    "marker_17": LocationGlobalRelative(),
    "marker_22": LocationGlobalRelative(),
    "marker_200": LocationGlobalRelative(),
}

# ArduCopter failsafe behaviors
failsafe_table = {
    0: "Disabled",
    1: "Enabled, always RTL", # Return to launch
    2: "Enabled, continue with Mission in Auto",
    3: "Enabled, always land",
}

"""
Take in user command line arguments that specifies where the MAVLink connection 
to be for SITL (127.0.0.1:14550) or a hawrdware connection (COM3)
"""
import argparse  
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None

# Start SITL if no connection specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the drone
# Set to True to make sure default attributes are populated before connect() returns
print('Connecting to drone on: %s' % connection_string)
drone = connect(connection_string, wait_ready= True) 

# Get drone home location
while not drone.home_location:
    cmds = drone.commands
    cmds.download()
    cmds.wait_ready()
    if not drone.home_location:
        print('\n Waiting for home location...')

# Received home location
print('\n Home location: %s' % drone.home_location) # looks like home location is always stored, need to verify


####### FUNCTIONS ########
"""
SEND LATITUDE/LONGITUDE COORDINATES:

The Scout drone will be able to send a location to the delivery drone with some meters off accuracy,
Delivery drone can take it from there. Original location can be delivery drone's home location
"""
def get_location_meters(original_location, dist_North, dist_East): # source DroneKit mission_basic.py
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    #Coordinate offsets in radians
    dLat = dNorth/EARTH_RADIUS_spherical
    dLon = dEast/(EARTH_RADIUS_spherical*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

"""
SEND DISTANCE METERS FROM THE GROUND:

The Scout drone will also be able to send x distance meters away from specified locations,
need to test which works out better, or send both in parallel. (Redundancy)
"""
def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.

    This method is an approximation, and will not be accurate over large distances and close to the 
    earth's poles. It comes from the ArduPilot test code: 
    https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

# communication msg_send(), msg_receive

# video-streaming / visual processing
