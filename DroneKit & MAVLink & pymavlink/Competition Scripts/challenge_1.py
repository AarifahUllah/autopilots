#!/usr/bin/env python
"""
This script demonstrates a combined search-and-land mission. It:
  • Uploads a Lissajous search pattern mission.
  • Arms and takes off.
  • Sets the scout to AUTO mode to follow the search pattern.
  • Starts a separate thread that continuously processes camera frames to detect a target ArUco marker.
  • When the marker is detected, the vehicle switches to GUIDED mode and executes a marker-guided landing maneuver.
  
Required calibration files (cameraMatrix.txt and cameraDistortion.txt) should reside in a "calib" folder
located in the same directory as this script.
"""

import argparse
import math
import time
import threading
import queue
import os

import cv2
import numpy as np
from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
from pymavlink import mavutil

###############################################
# CONSTANTS AND PARAMETERS
###############################################

# --- Search Pattern (Lissajous curve) Parameters ---
pi = math.pi
T_STEP = pi / 8         # Granularity of curve points with more (x,y) coordinates
T_MAX = 64              # Number of time step from 0 - T_MAX
AMP_X = 10              # Amplitude in X-direction (meters)
AMP_Y = 10              # Amplitude in Y-direction (meters)
W_X = 1                 # Angular frequency in X
W_Y = 2                 # Angular frequency in Y
PHI_X = pi / 2          # Phase shift in X
PHI_Y = 0               # Phase shift in Y
SCOUT_ALT = 10          # Search altitude in meters
CENTER_LAT = -35.363262 # Field center latitude (for waypoint calculations)
CENTER_LONG = 149.165237# Field center longitude
EARTH_RADIUS = 6378.137   # km (used for degree-meter conversion)

# --- Marker Landing Parameters ---
TARGET_MARKER_ID = 0   # ID of the target ArUco marker
MARKER_SIZE = 10        # Physical marker size in meters
LAND_ALT_CM = 50.0      # Landing threshold altitude in meters (when marker is close)
ANGLE_DESCEND = 20 * (pi/180)  # Angular threshold (in radians) to allow descent
LAND_SPEED_CMS = 30.0   # Descent speed in cm/s
FREQ_SEND = 1.0         # Frequency (Hz) for sending landing commands

# --- Camera Calibration and Video Settings ---
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
CALIB_PATH = os.path.join(BASE_DIR, "calib")
CAMERA_MATRIX_FILE = os.path.join(CALIB_PATH, "cameraMatrix.txt")
CAMERA_DIST_FILE = os.path.join(CALIB_PATH, "cameraDistortion.txt")
CAMERA_SIZE = [640, 480]  # [width, height]

###############################################
# UTILITY FUNCTIONS
###############################################

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns new latitude and longitude (as a tuple) offset by dNorth and dEast (meters)
    from the original_location.
    """
    earth_radius = 6378137.0  # in meters
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return newlat, newlon

def marker_position_to_angle(x, y, z):
    """
    Computes angular offsets (radians) given x, y, z position (assumed in meters).
    """
    angle_x = math.atan2(x, z)
    angle_y = math.atan2(y, z)
    return angle_x, angle_y

def camera_to_uav(x_cam, y_cam):
    """
    Converts camera coordinate system to UAV's coordinate frame.
    """
    x_uav = -y_cam
    y_uav = x_cam
    return x_uav, y_uav

def uav_to_ne(x_uav, y_uav, yaw_rad):
    """
    Converts UAV-relative coordinates to north-east offsets using the UAV's current yaw.
    """
    c = math.cos(yaw_rad)
    s = math.sin(yaw_rad)
    north = x_uav * c - y_uav * s
    east = x_uav * s + y_uav * c
    return north, east

def check_angle_descend(angle_x, angle_y, angle_desc):
    """
    Returns True if the total angular error is below a threshold.
    """
    return math.sqrt(angle_x**2 + angle_y**2) <= angle_desc

###############################################
# MISSION FUNCTIONS
###############################################
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

def lissajous_search():
    """
    The array represents the Cartesian plane with the origin being in the middle of the array.

    +-------^-------+
    |       |       |
    |       |       |
    <-------+------->
    |       |       |
    |       |       |
    +---------------+

    The array will need to be twice the amplitudes in both directions + 1 to include the four quadrants.
    The amplitudes will only count half, and are therefore known as half-amplitudes.

    y-coordinates corrspond to rows. x-coordinates correspond to the columns.

    To access the cell that corresponds to the appropriate (x,y) points, follow this mapping:

    cell array[row] = -Y_point + AMP_Y
    cell array[column] = X_point + AMP_X

    So, an (x,y) pair maps to array[-Y_point + AMP_Y][X_point + AMP_X].
    and the origin (0,0), center of field, is array[AMP_Y][AMP_X]
    """
  
    print("Uploading Lissajous search pattern mission...")
    cmds = scout.commands
    cmds.download()
    cmds.wait_ready()
    cmds.clear()

    rows = 2 * AMP_Y + 1
    columns = 2 * AMP_X + 1
    # Conversion from meters to degrees (approximate)
    met_to_deg = (1 / ((pi/180) * EARTH_RADIUS)) / 1000

    # Create 2D arrays for latitude and longitude
    latitude_arr = np.zeros((rows, columns), dtype=np.float64)
    longitude_arr = np.zeros((rows, columns), dtype=np.float64)
    for row in range(rows):
        for col in range(columns):
            latitude_arr[row, col] = CENTER_LAT + (AMP_Y - row) * met_to_deg
            longitude_arr[row, col] = CENTER_LONG + ((AMP_X - col) * met_to_deg) / math.cos(CENTER_LAT * (pi/180))

    # Add takeoff command as the first mission item
    cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                     mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, SCOUT_ALT))
    
    # Add waypoints following the Lissajous curve
    for T in range(T_MAX):
        X_point = math.ceil(AMP_X * math.sin(W_X * T_STEP * T + PHI_X))
        Y_point = math.ceil(AMP_Y * math.sin(W_Y * T_STEP * T + PHI_Y))
        lat = latitude_arr[-Y_point + AMP_Y, X_point + AMP_X]
        lon = longitude_arr[-Y_point + AMP_Y, X_point + AMP_X]
        print("Waypoint %d: (x,y)=(%d, %d) -> lat=%.10f, lon=%.10f" % (T, X_point, Y_point, lat, lon))
        cmds.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                         mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0,
                         lat, lon, SCOUT_ALT))
    print("Mission upload complete.")
    cmds.upload()

def arm_and_takeoff(target_alt):
    print("Performing pre-arm checks...")
    while not scout.is_armable:
        print(" Waiting for Scout initialization...")
        time.sleep(1)
    print("Arming motors...")
    scout.mode = VehicleMode("GUIDED")
    scout.armed = True
    while not scout.armed:
        print(" Waiting for arming...")
        time.sleep(1)
    print("Taking off!")
    scout.simple_takeoff(target_alt)
    while True:
        alt = scout.location.global_relative_frame.alt
        print(" Altitude: ", alt)
        if alt >= target_alt * 0.95:
            print("Reached target altitude.")
            break
        time.sleep(1)

###############################################
# ARUCO TRACKER CLASS
###############################################

class ArucoTracker(threading.Thread):
    """
    This class continuously captures video frames and processes them for ArUco marker detection.
    If the target marker (with specified ID) is detected, it stores the pose estimation data.
    """
    def __init__(self, target_id, marker_size, camera_matrix, camera_distortion, camera_size=[640,480], show_video=False):
        threading.Thread.__init__(self)
        self.target_id = target_id
        self.marker_size = marker_size  # in centimeters
        self.camera_matrix = camera_matrix
        self.camera_distortion = camera_distortion
        self.camera_size = camera_size
        self.show_video = show_video

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_100)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Open video capture
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, camera_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_size[1])

        # Shared variable for detection results
        self.found_marker = None
        self.shutdown_flag = False

        # FPS tracking (optional)
        self._t_read = time.time()
        self._t_detect = self._t_read
        self.fps_read = 0.0
        self.fps_detect = 0.0

    def update_fps_read(self):
        t = time.time()
        self.fps_read = 1.0 / (t - self._t_read)
        self._t_read = t

    def update_fps_detect(self):
        t = time.time()
        self.fps_detect = 1.0 / (t - self._t_detect)
        self._t_detect = t

    def run(self):
        while not self.shutdown_flag:
            ret, frame = self.cap.read()
            if not ret:
                continue
            self.update_fps_read()
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
                for i, marker_id in enumerate(ids.flatten()):
                    cv2.aruco.drawDetectedMarkers(frame, [corners[i]])
                    # Compute marker center (in pixels)
                    c = corners[i][0]
                    center_x = np.mean(c[:, 0])
                    center_y = np.mean(c[:, 1])
                    if marker_id == self.target_id:
                        self.update_fps_detect()
                        # Estimate pose: returns rotation and translation vectors
                        ret_val = cv2.aruco.estimatePoseSingleMarkers([corners[i]], self.marker_size,
                                                                       self.camera_matrix, self.camera_distortion)
                        if ret_val is not None:
                            rvec, tvec = ret_val[0][0, 0, :], ret_val[1][0, 0, :]
                            # tvec is in centimeters in the camera frame
                            self.found_marker = {
                                "id": marker_id,
                                "center": (center_x, center_y),
                                "rvec": rvec,
                                "tvec": tvec,
                                "timestamp": time.time()
                            }
                        # Draw coordinate axis on the marker
                        cv2.aruco.drawAxis(frame, self.camera_matrix, self.camera_distortion, rvec, tvec, 10)
                        cv2.putText(frame, "Marker Found", (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                                    1, (0, 255, 0), 2)
            if self.show_video:
                cv2.imshow("Aruco Tracker", frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    self.shutdown_flag = True
                    break
        self.cap.release()
        cv2.destroyAllWindows()

###############################################
# MAIN MISSION LOGIC
###############################################

def main():
    parser = argparse.ArgumentParser(description="Combined Search and Marker-Based Landing Mission")
    parser.add_argument('--connect', help="Vehicle connection target string.", required=True)
    parser.add_argument('--show_video', action='store_true', help="Display the video feed for marker detection.")
    args = parser.parse_args()

    # Load camera calibration parameters
    if not os.path.exists(CAMERA_MATRIX_FILE) or not os.path.exists(CAMERA_DIST_FILE):
        print("Camera calibration files not found in '%s'. Exiting." % CALIB_PATH)
        return
    camera_matrix = np.loadtxt(CAMERA_MATRIX_FILE, delimiter=',')
    camera_distortion = np.loadtxt(CAMERA_DIST_FILE, delimiter=',')

    print("Connecting to Scout on: %s" % args.connect)
    scout = connect(args.connect, wait_ready=True)

    # Start ArUco tracker thread
    tracker = ArucoTracker(target_id=TARGET_MARKER_ID, marker_size=MARKER_SIZE,
                           camera_matrix=camera_matrix, camera_distortion=camera_distortion,
                           camera_size=CAMERA_SIZE, show_video=args.show_video)
    tracker.daemon = True
    tracker.start()
  
    lissajous_search() # Upload the search pattern mission
    arm_and_takeoff(SCOUT_ALT)

    print("Starting search mission in AUTO mode...")
    scout.commands.next = 0
  
    scout.mode = VehicleMode("AUTO")
    while scout.mode != "AUTO":
        time.sleep(0.2)

    mission_start_time = time.time()
    
        while True:
            # Check if marker has been detected
            if tracker.found_marker is not None:
                print("Aruco marker detected. Initiating landing sequence.")
            
                scout.mode = VehicleMode("GUIDED")
                while scout.mode != "GUIDED":
                    time.sleep(0.2)
                  
                # Enter marker-guided landing loop
                while True:
                    marker_data = tracker.found_marker
                    if marker_data is None:
                        break  # If marker lost, exit landing loop
                    tvec = marker_data["tvec"]  # in centimeters
                    # Convert from camera frame to UAV frame (still in centimeters)
                    x_cam, y_cam, z_cam = tvec[0], tvec[1], tvec[2]
                    x_uav, y_uav = camera_to_uav(x_cam, y_cam)
                    current_location = scout.location.global_relative_frame
                    yaw = scout.attitude.yaw
                  
                    # Convert offsets from cm to meters for guidance
                    north_offset, east_offset = uav_to_ne(x_uav / 100.0, y_uav / 100.0, yaw)
                    marker_lat, marker_lon = get_location_metres(current_location, north_offset, east_offset)
                    angle_x, angle_y = marker_position_to_angle(x_uav / 100.0, y_uav / 100.0, z_cam / 100.0)
                  
                    # If the angular error is within threshold, command a descent.
                    if check_angle_descend(angle_x, angle_y, ANGLE_DESCEND):
                        print("Low angular error. Descending.")
                        new_alt = current_location.alt - (LAND_SPEED_CMS * 0.01 / FREQ_SEND)
                    else:
                        new_alt = current_location.alt
                    target_location = LocationGlobalRelative(marker_lat, marker_lon, new_alt)
                    scout.simple_goto(target_location)
                    print("UAV: Lat=%.7f, Lon=%.7f, Alt=%.2f" % (current_location.lat, current_location.lon, current_location.alt))
                    print("Commanding: Lat=%.7f, Lon=%.7f, Alt=%.2f" % (marker_lat, marker_lon, new_alt))
                    time.sleep(1.0 / FREQ_SEND)
                    # If the marker is very close or altitude is low, initiate landing.
                    if (z_cam <= LAND_ALT) or (new_alt <= 0.5):
                        print("Landing criteria met. Switching to LAND mode.")
                        scout.mode = VehicleMode("LAND")
                        break
                break  # Exit the outer mission loop once landing is initiated.

            # continue following waypoints if no marker is detected

            if scout.commands.next is T_MAX: # T_MAX is the last waypoint in the list
                print("Search mission complete. No marker found. Returning to launch.")
                scout.mode = VehicleMode("RTL")
                time.sleep(5)
                break

            time.sleep(1)
    
    print("Mission complete. Cleaning up...")
    tracker.shutdown_flag = True
    tracker.join() # close threads
    scout.close() # close Scout object

if __name__ == "__main__":
    main()
