
#BEST CODE 


from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy.distance import distance as geopy_distance
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import threading
from picamera2 import Picamera2
import argparse
import logging

logging.basicConfig(
    filename='drone_mission_log.txt',
    filemode='w',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger()
# ------------------- CONFIG -------------------
takeoff_altitude = 3  # meters
camera_resolution = (1600, 1080)
marker_id = 3
marker_size = 0.253  # meters
descent_speed = 0.2
final_land_height = 2.5  # meters
fast_descent_speed = 0.35
slow_descent_speed = 0.25
slow_down_altitude = 2
far_center_threshold = 50
near_center_threshold = 10
far_Kp = 0.0015
near_Kp = 0.001
marker_found_flag = threading.Event()

# ------------------- CONNECT -------------------
def connectMyCopter():
    print("Start Connection")
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600
    print("Connecting Pi to Drone...")
    vehicle = connect(connection_string, baud=baud_rate)
    return vehicle

vehicle = connectMyCopter()

# ------------------- CAMERA SETUP -------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ------------------- ARUCO SETUP -------------------
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
parameters.maxErroneousBitsInBorderRate = 0.1
parameters.adaptiveThreshWinSizeMin = 3
parameters.adaptiveThreshWinSizeMax = 23
parameters.adaptiveThreshWinSizeStep = 10
parameters.minCornerDistanceRate = 0.02
parameters.polygonalApproxAccuracyRate = 0.03

def capture_photo(index=None):
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    suffix = f"_{index}" if index is not None else ""
    photo_path = f"aruco_photo_{timestamp}{suffix}.jpg"
    cv2.imwrite(photo_path, img)
    print(f"Photo captured and saved at: {photo_path}")

# ------------------- FLIGHT FUNCTIONS -------------------
def manual_arm():
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    while not vehicle.armed:
        print("Waiting for user to arm vehicle...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    print("Vehicle armed. Mode:", vehicle.mode.name)

def takeoff(aTargetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f}")
        if alt >= aTargetAltitude * 0.70:
            print("Reached target altitude")
            break
        time.sleep(0.1)

def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5

def goto_waypoint(waypoint, num):
    print(f"Going to waypoint {num}...")
    vehicle.simple_goto(waypoint,airspeed = 10)
    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current)
        if dist < 1 or marker_found_flag.is_set():
            break
        time.sleep(0.01)
    if marker_found_flag.is_set():
        print("Marker found. Interrupting waypoint navigation.")

def land():
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    vehicle.close()
    print("Drone has landed.")

# ------------------- MARKER WATCHER -------------------
def marker_watcher():
    print("Marker watcher started...")
    frame_width = camera_resolution[0]
    middle_left = int(0.1 * frame_width)      # 10% from the left
    middle_right = int(0.9 * frame_width)       # 90% from the left

    while not marker_found_flag.is_set():
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and marker_id in ids:
            index = np.where(ids == marker_id)[0][0]
            c = corners[index][0]
            cx = int(np.mean(c[:, 0]))  # x center of marker

            # Only trigger if it's in the middle column
            if middle_left <= cx <= middle_right:
                capture_photo(0)
                print("DropZone FOUND in center column! Triggering precision landing...")
                logger.info("DropZone Detected")
                marker_found_flag.set()
                break
            else:
                print("DropZone found, but NOT in center column.")
                possible_aruco_lat = vehicle.location.global_frame.lat
                possible_aruco_lon = vehicle.location.global_frame.lon
                logger.info(f"Possible DropZone Location: Lat {possible_aruco_lat}, Lon {possible_aruco_lon}") 
        time.sleep(0.01)


def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # USB telemetry module
    baud_rate = 57600  # Ensure the correct baud rate
    
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link


telem_link = setup_telem_connection()
 
# ------------------- PRECISION LANDING -------------------
def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

def precision_land_pixel_offset():
    print("Beginning precision landing...")
    time.sleep(0.2)
    aruco_lat = vehicle.location.global_frame.lat
    aruco_lon = vehicle.location.global_frame.lon
    capture_photo(1)
    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and marker_id in ids:
            index = np.where(ids == marker_id)[0][0]
            c = corners[index][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            frame_center = (camera_resolution[0] // 2, camera_resolution[1] // 2)
            dx = cx - frame_center[0] + 15
            dy = cy - frame_center[1] - 120  # Adjust for camera pos
            altitude = vehicle.rangefinder.distance or 10.0
            if altitude > slow_down_altitude:
                descent_vz = fast_descent_speed
                center_threshold = far_center_threshold
                Kp = far_Kp
            else:
                descent_vz = slow_descent_speed
                center_threshold = near_center_threshold
                Kp = near_Kp
            if altitude > final_land_height:
                if abs(dx) < center_threshold and abs(dy) < center_threshold:
                    print("Marker centered. Lnading...")
                    send_ned_velocity(0, 0, descent_vz)
                    aruco_lat = vehicle.location.global_frame.lat
                    aruco_lon = vehicle.location.global_frame.lon
                    print(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
                    logger.info(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
                    vehicle.mode = VehicleMode("LAND")
                    break
                    
                else:
                    print("Centering marker...")
                    vx = -dy * Kp
                    vy = dx * Kp
                    send_ned_velocity(vx, vy, 0.01)
            else:
                aruco_lat = vehicle.location.global_frame.lat
                aruco_lon = vehicle.location.global_frame.lon
                print(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
                logger.info(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
                print("Reached final height. Switching to LAND.")
                vehicle.mode = VehicleMode("LAND")
                break
        else:
            print("Marker Lost. Returning to last known location")
            vehicle.simple_goto(LocationGlobalRelative(aruco_lat, aruco_lon, takeoff_altitude + 1))
        time.sleep(0.1)

# ------------------- MAIN MISSION -------------------
print("Starting mission...")
manual_arm()
takeoff(takeoff_altitude)


watcher_thread = threading.Thread(target=marker_watcher, daemon=True)
watcher_thread.start()

waypoints = [
LocationGlobalRelative(27.9867158, -82.3015961, takeoff_altitude),
LocationGlobalRelative(27.9866808, -82.3018502, takeoff_altitude),
LocationGlobalRelative(27.9866749, -82.3015961, takeoff_altitude),
LocationGlobalRelative(27.9866435, -82.3018496, takeoff_altitude),
LocationGlobalRelative(27.9866370, -82.3015954, takeoff_altitude),
LocationGlobalRelative(27.9866056, -82.3018502, takeoff_altitude),
LocationGlobalRelative(27.9866027, -82.3015954, takeoff_altitude),
LocationGlobalRelative(27.9865671, -82.3018496, takeoff_altitude),
LocationGlobalRelative(27.9865671, -82.3015961, takeoff_altitude),
LocationGlobalRelative(27.9865275, -82.3018509, takeoff_altitude),
LocationGlobalRelative(27.9865291, -82.3015920, takeoff_altitude),
LocationGlobalRelative(27.9867193, -82.3018502, takeoff_altitude),
]

for i, wp in enumerate(waypoints):
    goto_waypoint(wp, i + 1)
    if marker_found_flag.is_set():
        break

if marker_found_flag.is_set():
    precision_land_pixel_offset()
else:
    print("No marker detected during mission. Proceeding to normal landing.")
    land()

picam2.stop()
vehicle.close()
print("Mission completed.")
exit()
# ------------------- END OF SCRIPT -------------------