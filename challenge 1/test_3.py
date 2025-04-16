                


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
takeoff_altitude = 5  # meters
camera_resolution = (1600, 1080)
marker_id = 3
marker_size = 0.253  # meters
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
        if alt >= aTargetAltitude * 0.90:
            print("Reached target altitude")
            break
        time.sleep(1)

def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5

def goto_waypoint(waypoint, num):
    print(f"Going to waypoint {num}...")
    vehicle.simple_goto(waypoint, airspeed = 9)
    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current)
        print(f"Distance to waypoint {num}: {dist:.2f}m")
        if dist < 1 or marker_found_flag.is_set():
            break
        time.sleep(1)
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
    while not marker_found_flag.is_set():
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and marker_id in ids:
            print("DropZone FOUND! Triggering precision landing...")
            marker_found_flag.set()
            aruco_lat = vehicle.location.global_frame.lat
            aruco_lon = vehicle.location.global_frame.lon
            print(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
            logger.info(f"DropZone Location: Lat {aruco_lat}, Lon {aruco_lon}")
            vehicle.mode = VehicleMode("LAND")
            break
        time.sleep(0.5)




# ------------------- MAIN MISSION -------------------
print("Starting mission...")
logger.info("Mission Start")
vehicle.mode = VehicleMode("GUIDED")
manual_arm()
takeoff(takeoff_altitude)


watcher_thread = threading.Thread(target=marker_watcher, daemon=True)
watcher_thread.start()

waypoints = [
LocationGlobalRelative(27.9867265, -82.3018582, takeoff_altitude),
LocationGlobalRelative(27.9865177, -82.3018379, takeoff_altitude),
LocationGlobalRelative(27.9867269, -82.3018226, takeoff_altitude),
LocationGlobalRelative(27.9865173, -82.3018023, takeoff_altitude),
LocationGlobalRelative(27.9867274, -82.3017870, takeoff_altitude),
LocationGlobalRelative(27.9865170, -82.3017667, takeoff_altitude),
LocationGlobalRelative(27.9867278, -82.3017515, takeoff_altitude),
LocationGlobalRelative(27.9865166, -82.3017311, takeoff_altitude),
LocationGlobalRelative(27.9867283, -82.3017159, takeoff_altitude),
LocationGlobalRelative(27.9865162, -82.3016955, takeoff_altitude),
LocationGlobalRelative(27.9867287, -82.3016803, takeoff_altitude),
LocationGlobalRelative(27.9865158, -82.3016599, takeoff_altitude),
LocationGlobalRelative(27.9867292, -82.3016447, takeoff_altitude),
LocationGlobalRelative(27.9865155, -82.3016243, takeoff_altitude),
LocationGlobalRelative(27.9867297, -82.3016091, takeoff_altitude),
LocationGlobalRelative(27.9865151, -82.3015888, takeoff_altitude),
]

for i, wp in enumerate(waypoints):
    goto_waypoint(wp, i + 1)
    if marker_found_flag.is_set():
        break


picam2.stop()
vehicle.close()
print("Mission completed.")
logger.info("Mission End")
exit()
# ------------------- END OF SCRIPT -------------------
                