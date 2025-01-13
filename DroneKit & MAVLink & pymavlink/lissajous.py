"""

Lissajous search mission. The drone's starting point is random. The only information we need is the center of the field. 

Lissajous curves are defined by two parametric equations:
[x,y] = < Ax*sin(wx * t + phix), Ay*sin(wy * t + phiy) >
    Ax = Amplitude in X-direction
    Ay = Amplitude in Y-direction

    wx = Angular frequency in X-dir
    wy = Angular frequency in Y-dir

    phix = Phase shift in X-dir
    phiy = Phase shift in Y-dir


Lissajous automatically bounds the field dimensions with Ax and Ay. A geofence will still be created 
for extra caution and fail-safe behavior. Adjusting the six parameters leads to infinitely many patterns.
If ratio rw = wx/wy is rational, the curve is cyclical (repeating). Irrational -> the pattern doesn't ever 
repeat.

"""

#############DEPENDENCIES#############
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil
import time
import numpy as np
import math
import argparse

#############CONSTANTS##############
T_STEP = pi/8 # defines granularity of curve with more (x,y) coordinates
T_MAX = 64 # how many time points we want from 0 - T_MAX (though it doesn't have to be 0) & should be enough for the challenge duration

AMP_X = 25 # meters
AMP_Y = 25 # field dimensions are 90ft x 90ft ~ approx. 27.4 m. The curve will be slightly less than boundary to not trigger geofence
W_X = pi/2
Y_X = 0
SCOUT_ALT = 10 # meters ~ approx. 33 ft

#####SETTING UP THE LISSAJOUS CURVE#######
print("Lissajous curve search pattern")
#let's make a repeating figure 8 for starters...
longitude_arr = np.arange(AMP_X * AMP_Y).reshape((AMP_X, AMP_Y))
print(longitude_arr)

latitude_arr = np.arrange((AMP_X * AMP_Y)).reshape((AMP_X, AMP_Y)) # store longitude and latitude coordinates every 1 meter apart
print(latitude_arr)

for 0 to T_MAX:
    X_point = math.ceil( AMP_X * sin(W_X * T_STEP * T_MAX + PHI_X) )
    Y_point = math.ceil( AMP_Y * sin(W_Y * T_STEP * T_MAX + PHI_Y) )
    print("(x,y) = (%s, %s)" %(X_point, Y_point))
    print("longitude point: %s" %longitude_arr[X_point][Y_point])
    print("latitude point: %s" %latitude_arr[X_point][Y_point])

print("YAY Lissajous curve made ^_^")

"""
# Boundary points that define field at Xelevate location (for geofence and breaches)

A.------- B.
|         |
|         |
|         |
C.------- D.

boundary_locations = {
    "point_A": LocationGlobalRelative(39.234743, -77.546500, 10),
    "point_B": LocationGlobalRelative(39.234718, -77.546199, 10),
    "point_C": LocationGlobalRelative(39.234519, -77.54653, 10),
    "point_D": LocationGlobalRelative(39.234501, -77.546227, 10),
}
"""
#############CONNECTION#############
parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
scout = connect(connection_string, wait_ready=True)

def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = scout.commands
    cmds.download()
    cmds.wait_ready() # wait until download is complete.

def get_location_metres(original_location, dNorth, dEast):
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
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat)+(dlong*dlong))*1.113195e5 # 1.113195e5 is the number of metres per degree of lat/long

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = scout.commands

    print(" Clear any existing commands")
    cmds.clear() 
    
    print(" Define/add new commands.")
    # Add new commands. The meaning/order of the parameters is documented in the Command class. 
     
    #Add MAV_CMD_NAV_TAKEOFF command. This is ignored if the vehicle is already in the air.
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #Define the four MAV_CMD_NAV_WAYPOINT locations and add the commands
    point1 = get_location_metres(aLocation, aSize, -aSize)
    point2 = get_location_metres(aLocation, aSize, aSize)
    point3 = get_location_metres(aLocation, -aSize, aSize)
    point4 = get_location_metres(aLocation, -aSize, -aSize)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point1.lat, point1.lon, 11))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point2.lat, point2.lon, 12))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point3.lat, point3.lon, 13))
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))
    #add dummy waypoint "5" at point 4 (lets us know when have reached destination)
    cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, point4.lat, point4.lon, 14))    

    print(" Upload new commands to vehicle")
    cmds.upload()
    
def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = scout.commands.next
    if nextwaypoint==0:
        return None
    missionitem=scout.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(scout.location.global_frame, targetWaypointLocation)
    return distancetopoint

def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not scout.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True

    while not scout.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    scout.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", scout.location.global_relative_frame.alt)      
        if scout.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def main():

    #print('Create a new mission')
    #adds_square_mission(scout.location.global_frame, 5)
    #print("Drone's starting location: %s" % scout.location.global_frame)
    
    download_mission()

    arm_and_takeoff(10)
    print("Starting mission")

    scout.commands.next = 0
    scout.mode = VehicleMode("AUTO")

    """
    while True:
        nextwaypoint=scout.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    
        if nextwaypoint==3: #Skip to next waypoint
            print('Skipping to Waypoint 5 when reach waypoint 3')
            scout.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)
    """

    print("Flying 5m/s NORTH relative to the front of the drone for 25m")
    counter = 0
    while counter < 5 : # change counter based on speed
        send_local_ned_velocity(5, 0, 0) #5m/s 
        time.sleep(1) #pause 1 second
        counter = counter + 1

    print('Return to launch')
    scout.mode = VehicleMode("RTL")


    #Close vehicle object before exiting script
    print("Close vehicle object")
    scout.close()
 
if __name__ == "__main__":

    main()
