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
import os

# ------------------- CONFIG -------------------
takeoff_altitude = 5  # meters
camera_resolution = (1280, 720)
marker_size = 0.253  # meters
center_threshold = 20  # pixels
Kp = 0.0015
photo_save_path = "/home/uav/drone/photos"

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
        if alt >= aTargetAltitude * 0.85:
            print("Reached target altitude")
            break
        time.sleep(1)

def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5

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

def search_and_center_on_marker():
    print("Searching for any ArUco marker...")
    timeout = time.time() + 10  # Try for up to 10 seconds per stop
    while time.time() < timeout:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            largest_marker_index = np.argmax([cv2.contourArea(corner) for corner in corners])
            c = corners[largest_marker_index][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            frame_center = (camera_resolution[0] // 2, camera_resolution[1] // 2)
            dx = cx - frame_center[0]
            dy = cy - frame_center[1]

            if abs(dx) < center_threshold and abs(dy) < center_threshold:
                print("Marker centered. Taking photo...")
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                if not os.path.exists(photo_save_path):
                    os.makedirs(photo_save_path)
                photo_path = os.path.join(photo_save_path, f"marker_{timestamp}.jpg")
                cv2.imwrite(photo_path, img)
                print(f"Photo saved: {photo_path}")
                return True
            else:
                vx = -dy * Kp
                vy = dx * Kp
                send_ned_velocity(vx, vy, 0)
        else:
            print("No marker detected.")
        time.sleep(0.2)
    print("No marker centered within timeout.")
    return False

def goto_waypoint(waypoint, num):
    print(f"Going to waypoint {num}...")
    vehicle.simple_goto(waypoint)
    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current)
        print(f"Distance to waypoint {num}: {dist:.2f}m")
        if dist < 0.75:
            print("Reached waypoint.")
            break
        time.sleep(1)

    print(f"Scanning for marker at waypoint {num}...")
    found = search_and_center_on_marker()
    if found:
        print(f"Marker handled at waypoint {num}.")
    else:
        print("No marker handled at this waypoint.")

def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        print("Waiting for landing...")
        time.sleep(1)
    print("Landed.")
    vehicle.close()

# ------------------- MAIN MISSION -------------------
print("Starting mission...")
manual_arm()
takeoff(takeoff_altitude)
vehicle.airspeed = 3

# Define your waypoint list
waypoints = [
    LocationGlobalRelative(27.9865908,-82.3017772, 6),
    LocationGlobalRelative(27.9865914,-82.3016015, 6),
    LocationGlobalRelative(27.9866785,-82.3015860, 6),
    LocationGlobalRelative(27.9866660,-82.3017711, 6)
]

for i, wp in enumerate(waypoints):
    goto_waypoint(wp, i + 1)

land()

picam2.stop()
print("Mission completed.")
exit()
# ------------------- END OF CODE -------------------