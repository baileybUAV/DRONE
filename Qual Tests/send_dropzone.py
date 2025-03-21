from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import future
import socket
import math
import argparse
import os
import numpy as np
import cv2
import cv2.aruco as aruco
from picamera2 import Picamera2

# Initialize Pi Camera
width = 1280
height = 720
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ARUCO SETTINGS
id_to_find = 1
marker_size = 0.253
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Load camera calibration
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

horizontal_fov = 70 * (math.pi / 180)
vertical_fov = 70 * (height / width) * (math.pi / 180)

# Connect to the drone
def connectMyCopter():
    print("Start Connection")
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600

    print("Connecting Pi to Drone...")
    vehicle = connect(connection_string, baud=baud_rate)
    print("Connected to drone")
    return vehicle

# Detect and check if marker is centered

def is_dropzone_centered(threshold_px=50):
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.undistort(img, cameraMatrix, cameraDistortion)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and id_to_find in ids:
        index = np.where(ids == id_to_find)[0][0]
        marker_corners = corners[index][0]
        center_x = np.mean(marker_corners[:, 0])
        center_y = np.mean(marker_corners[:, 1])

        # Check if marker is near the center of the image
        if abs(center_x - width/2) < threshold_px and abs(center_y - height/2) < threshold_px:
            return True

    return False

# Telemetry setup
def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"
    baud_rate = 57600
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link

# MAIN EXECUTION
vehicle = connectMyCopter()
telem_link = setup_telem_connection()

print("Waiting for DropZone detection and centering...")

dropzone_found = False
while not dropzone_found:
    if is_dropzone_centered():
        print("DropZone detected and centered! Starting data transmission.")
        dropzone_found = True
    else:
        print("Searching for centered DropZone...")
    time.sleep(1)

# Begin data sending loop for 1 minute
start_time = time.time()
while time.time() - start_time < 60:
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt
    rel_alt = vehicle.location.global_relative_frame.alt
    velocity_north = vehicle.velocity[0]
    velocity_east = vehicle.velocity[1]
    velocity_down = vehicle.velocity[2]

    if lat is None or lon is None or lat == 0 or lon == 0:
        print("Error: No valid GPS data available")
    else:
        covariance_matrix = np.full((36,), float('nan'), dtype=np.float32)

        msg = telem_link.mav.global_position_int_cov_encode(
            int(time.time() * 1e6),
            mavutil.mavlink.MAV_ESTIMATOR_TYPE_GPS,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000),
            int(rel_alt * 1000),
            float(velocity_north),
            float(velocity_east),
            float(velocity_down),
            covariance_matrix)

        telem_link.mav.send(msg)
        print(f"Sent GLOBAL_POSITION_INT_COV Data: Lat {lat}, Lon {lon}, Alt {alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")

    time.sleep(2)

# Land the drone
print("1 minute passed. Initiating landing sequence...")
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    print("Waiting for landing to complete...")
    time.sleep(1)

print("Drone has landed and disarmed. Mission complete.")
