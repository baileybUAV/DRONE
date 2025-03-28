import threading
from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy.distance import distance as geopy_distance
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import argparse
from picamera2 import Picamera2

# ------------------- CONFIGURATION -------------------
takeoff_altitude = 6
marker_id = 0
camera_resolution = (1280, 720)
marker_size = 0.253  # meters
final_land_height = 1
fast_descent_speed = 0.35
slow_descent_speed = 0.15
slow_down_altitude = 3.0
far_center_threshold = 50
near_center_threshold = 15
far_Kp = 0.004
near_Kp = 0.002

marker_detected = threading.Event()

# FUNCTIONS
# Connect to the Vehicle function
def connectMyCopter():
  print("Start Pi Connection")
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

# ------------------- ARUCO SETUP -------------------
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# ------------------- FLIGHT CONTROL -------------------
def manual_arm():
    print("Waiting for vehicle to initialize...")
    while not vehicle.is_armable:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    while vehicle.mode.name != "GUIDED":
        print("Waiting for GUIDED mode...")
        time.sleep(1)

    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Vehicle armed and in GUIDED mode.")

def takeoff(target_altitude):
    print("Taking off...")
    vehicle.simple_takeoff(target_altitude)
    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.2f}")
        if current_alt >= target_altitude * 0.85:
            print("Target altitude reached.")
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

# ------------------- PRECISION LANDING -------------------

def precision_landing_loop():
    print("Precision landing detection thread started.")
    while vehicle.armed and not marker_detected.is_set():
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            print("Target marker detected! Interrupting mission.")
            marker_detected.set()
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)  # ensure mode change
            precision_land_pixel_offset()
            break

        time.sleep(0.2)


def precision_land_pixel_offset():
    print("Beginning precision descent...")
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

            altitude = vehicle.rangefinder.distance
            if altitude is None or altitude <= 0:
                altitude = 10.0

            if altitude > slow_down_altitude:
                descent_vz = fast_descent_speed
                center_threshold = far_center_threshold
                Kp = far_Kp
            else:
                descent_vz = slow_descent_speed
                center_threshold = near_center_threshold
                Kp = near_Kp

            print(f"dx={dx}, dy={dy}, Alt={altitude:.2f}, Vz={descent_vz}, Kp={Kp}")

            if altitude > final_land_height:
                if abs(dx) < center_threshold and abs(dy) < center_threshold:
                    print("Centered. Descending...")
                    send_ned_velocity(0, 0, descent_vz)
                else:
                    vx = -dy * Kp
                    vy = dx * Kp
                    print(f"Correcting: vx={vx:.2f}, vy={vy:.2f}, vz={descent_vz}")
                    send_ned_velocity(vx, vy, descent_vz)
            else:
                print("Final height reached. Switching to LAND.")
                vehicle.mode = VehicleMode("LAND")
                break
        time.sleep(0.1)

# ------------------- WAYPOINT NAVIGATION -------------------
def goto_waypoint(waypoint, number):
    print(f"Going to waypoint {number}...")
    vehicle.simple_goto(waypoint)
    while True:
        if marker_detected.is_set():
            print(f"Waypoint {number} interrupted: Marker detected.")
            return False

        current_location = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current_location)
        print(f"[Waypoint {number}] Distance: {dist:.2f} m")

        if dist < 0.5:
            print(f"Reached waypoint {number}.")
            return True
        time.sleep(1)

# ------------------- MISSION -------------------
manual_arm()
takeoff(takeoff_altitude)

# Start ArUco detection thread
landing_thread = threading.Thread(target=precision_landing_loop, daemon=True)
landing_thread.start()

waypoints = [
    LocationGlobalRelative(27.9873411, -82.3012447, takeoff_altitude),
    LocationGlobalRelative(27.9871291, -82.3012541, takeoff_altitude),
    LocationGlobalRelative(27.9871243, -82.3016618, takeoff_altitude),
    LocationGlobalRelative(27.9873340, -82.3016605, takeoff_altitude),
    LocationGlobalRelative(27.9873411, -82.3012447, takeoff_altitude)
]

# Go to first waypoint before searching for marker
if goto_waypoint(waypoints[0], 1):
    # Start ArUco thread only after drone is moving and stable
    landing_thread = threading.Thread(target=precision_landing_loop, daemon=True)
    landing_thread.start()

    # Proceed with remaining waypoints
    for i, wp in enumerate(waypoints[1:], start=2):
        if not goto_waypoint(wp, i):
            break

print("Mission complete or interrupted. Landing.")
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    time.sleep(1)

picam2.stop()
vehicle.close()