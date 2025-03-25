from dronekit import connect, VehicleMode
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import argparse
from picamera2 import Picamera2

# ------------------- CONFIGURATION -------------------
takeoff_altitude = 6  # meters
camera_resolution = (1280, 720)
marker_id = 1
marker_size = 0.253  # meters (black square size)
HFOV = 110 * math.pi / 180  # Horizontal FOV in radians
VFOV = HFOV * (camera_resolution[1] / camera_resolution[0])  # Vertical FOV
landing_target_stream_rate = 0.1  # seconds (10 Hz)

# ------------------- CONNECT TO VEHICLE -------------------
# Connect to the Vehicle function
def connectMyCopter():
  print("Start Connection")
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()

  connection_string = "/dev/ttyAMA0"
  baud_rate = 57600

  print("Connecting Pi to Drone...")
  vehicle = connect(connection_string,baud=baud_rate) 
  print("GPS: %s" % vehicle.gps_0)
  print("Battery: %s" % vehicle.battery)
  print("Armable?: %s" % vehicle.is_armable)
  print("Height from Lidar: %s" % vehicle.rangefinder)
  print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
  print("Global Location: %s" % vehicle.location.global_frame)
  print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
  print("Local Location: %s" % vehicle.location.local_frame)
  print("Mode: %s" % vehicle.mode.name)     
  return vehicle

  
vehicle = connectMyCopter()
print("Pi Connected")

# ------------------- CAMERA SETUP -------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ------------------- CALIBRATION -------------------
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# ------------------- TAKEOFF -------------------
def manual_arm_and_takeoff(target_alt):
    print("Pre-arm checks...")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialize...")
        time.sleep(1)

    print("Waiting for manual arming...")
    while not vehicle.armed:
        print("Waiting for user to arm vehicle...")
        time.sleep(1)

    print("Vehicle armed. Taking off!")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.2f}")
        if current_alt >= target_alt * 0.85:
            print("Target altitude reached.")
            break
        time.sleep(1)

# ------------------- SEND LANDING TARGET -------------------
def send_land_message(x, y, distance):
    msg = vehicle.message_factory.landing_target_encode(
        0,  # time_boot_ms (not used)
        0,  # target number
        mavutil.mavlink.MAV_FRAME_BODY_FRD,  # Required for ArduPilot PLND
        x, y,  # angle_x, angle_y (radians)
        distance,  # distance to target (meters)
        0, 0  # size_x, size_y (optional, can be zero)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ------------------- PLND LOOP -------------------
def start_plnd_tracking():
    print("Starting ArUco tracking and PLND stream...")
    land_triggered = False

    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            index = np.where(ids == marker_id)[0][0]
            marker_corners = corners[index][0]

            cx = int(np.mean(marker_corners[:, 0]))
            cy = int(np.mean(marker_corners[:, 1]))

            dx = cx - (camera_resolution[0] / 2)
            dy = cy - (camera_resolution[1] / 2)

            x_ang = dx * (HFOV / camera_resolution[0])
            y_ang = dy * (VFOV / camera_resolution[1])

            altitude = vehicle.rangefinder.distance
            if altitude is None or altitude <= 0:
                altitude = 10.0  # Fallback

            send_land_message(x_ang, y_ang, altitude)

            print(f"PLND Message: x_ang={math.degrees(x_ang):.2f}°, y_ang={math.degrees(y_ang):.2f}°, distance={altitude:.2f}m")

            if not land_triggered:
                print("Target acquired. Switching to LAND mode for PLND...")
                vehicle.mode = VehicleMode("LAND")
                land_triggered = True
        else:
            print("Marker not detected. Continuing to scan...")

        time.sleep(landing_target_stream_rate)

# ------------------- RUN SEQUENCE -------------------
print("Starting PLND-based Test Flight...")
manual_arm_and_takeoff(takeoff_altitude)
time.sleep(2)
start_plnd_tracking()
print("Landing process complete.")
vehicle.close()
picam2.stop()