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
takeoff_altitude = 6
camera_resolution = (1280, 720)
marker_id = 0
marker_size = 0.253  # meters (black square only)
Kp = 0.0025
center_threshold = 20  # pixels
descent_speed = 0.2  # m/s downward
final_land_height = 1.0  # meters above target

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

# ------------------- CALIBRATION FILES -------------------
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# ------------------- MANUAL ARM + TAKEOFF -------------------
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

# ------------------- SEND VELOCITY COMMAND -------------------
def send_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ------------------- PRECISION LANDING LOGIC -------------------
def precision_land_pixel_offset():
    print("Beginning precision landing...")

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
            dx = cx - frame_center[0]
            dy = cy - frame_center[1]

            print(f"Offset dx={dx}, dy={dy}")

            if abs(dx) < center_threshold and abs(dy) < center_threshold:
                altitude = vehicle.rangefinder.distance
                if altitude is None or altitude <= 0:
                    altitude = 10.0  # fallback if LiDAR fails
                
                if altitude > final_land_height:
                    print("Centered. Descending...")
                    send_ned_velocity(0, 0, descent_speed)
                else:
                    print("Reached final height. Switching to LAND.")
                    vehicle.mode = VehicleMode("LAND")
                    break
            else:
                vx = -dy * Kp
                vy = dx * Kp
                print(f"Correcting position: vx={vx:.2f}, vy={vy:.2f}")
                send_ned_velocity(vx, vy, 0)

        else:
            print("Marker not detected. Hovering.")
            send_ned_velocity(0, 0, 0)

        time.sleep(0.1)  # 10 Hz

# ------------------- FLIGHT EXECUTION -------------------
print("Starting Test Flight...")
manual_arm_and_takeoff(takeoff_altitude)
time.sleep(2)
precision_land_pixel_offset()
print("Landing complete.")

vehicle.close()
picam2.stop()