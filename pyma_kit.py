from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil, mavwp
import time

"""
def arm_and_takeoff(vehicle, altitude):

    #Arm the vehicle and take off to the specified altitude.

    print("Arming the vehicle...")

    vehicle.mav.command_long_send(vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)

    print("Waiting for the vehicle to arm")

    while not vehicle.motors_armed():

        time.sleep(1)

    print("Taking off...")

    vehicle.mav.command_long_send( vehicle.target_system, vehicle.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
 
def create_waypoint(lat, lon, alt):

    #Create a MAVLink waypoint message.

    return mavutil.mavlink.MAVLink_mission_item_message(

        0, 0, 0,

        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,

        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,

        0, 0, 0, 0, 0, 0,

        lat, lon, alt

    )
 
def upload_mission(vehicle, waypoints):

    #Upload the mission waypoints to the vehicle.

    print("Uploading mission...")

    vehicle.waypoint_clear_all_send()

    vehicle.waypoint_count_send(len(waypoints))

    for i, wp in enumerate(waypoints):

        vehicle.mav.send(wp)

        print(f"Waypoint {i+1} sent")
"""
def download_mission():
    """
    Download the current mission from the vehicle.
    """
    cmds = the_connection.commands
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

def adds_square_mission(aLocation, aSize):
    """
    Adds a takeoff command and four waypoint commands to the current mission. 
    The waypoints are positioned to form a square of side length 2*aSize around the specified LocationGlobal (aLocation).

    The function assumes vehicle.commands matches the vehicle mission state 
    (you must have called download at least once in the session and after clearing the mission)
    """	

    cmds = vehicle.commands

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

def arm_and_takeoff(aTargetAltitude):

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not the_connection.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

        
    print("Arming motors")
    # Copter should arm in GUIDED mode
    the_connection.mode = VehicleMode("GUIDED")
    the_connection.armed = True

    while not the_connection.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    the_connection.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", the_connection.location.global_relative_frame.alt)      
        if the_connection.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

def main():

    # Start a connection listening on a UDP port
    the_connection = mavutil.mavlink_connection('udpin:localhost:14551')

    # Wait for the first heartbeat
    # This sets the system and component ID of remote system for the link
    the_connection.wait_heartbeat()
    print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

    print('Create a new mission (for current location)')
    adds_square_mission(the_connection.location.global_frame, 15)

    arm_and_takeoff(10)
    print("Starting mission")

    the_connection.commands.next = 0
    the_connection.mode = VehicleMode("AUTO")

    while True:
        nextwaypoint=the_connection.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    
        if nextwaypoint==3: #Skip to next waypoint
            print('Skipping to Waypoint 5 when reach waypoint 3')
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)

    print('Return to launch')
    the_connection.mode = VehicleMode("RTL")


    #Close vehicle object before exiting script
    print("Close vehicle object")
    the_connection.close()

    """
    #Set the flight mode of the vehicle. Mode ID (1 for AUTO)
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_DO_SET_MODE, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    # Once connected, use 'the_connection' to get and send messages
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)

    #Take off 10 meters
    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 10)
    msg = the_connection.recv_match(type='COMMAND_ACK', blocking=True)
    print(msg)
 
    print("Mission in progress...")

    while True:

        # Monitor mission progress (e.g., check for mission completion)

        msg = the_connection.recv_match(type="MISSION_ITEM_REACHED", blocking=True)

        print(f"Reached waypoint {msg.seq}")

        if msg.seq == len(waypoints) - 1:

            print("Mission completed")

            break
 
    print("Landing...")

    the_connection.mav.command_long_send(the_connection.target_system, the_connection.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 0, 0, 0, 0, 0, 0)
    """
 
if __name__ == "__main__":

    main()
