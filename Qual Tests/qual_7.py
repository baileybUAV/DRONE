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
takeoff_altitude = 4
marker_id = 0
camera_resolution = (1280, 720)
marker_size = 0.253  # meters
final_land_height = 1.0  # meters above target
fast_descent_speed = 0.35
slow_descent_speed = 0.15
slow_down_altitude = 3.0
far_center_threshold = 50
near_center_threshold = 15
far_Kp = 0.004
near_Kp = 0.002

# ------------------- CONNECT TO VEHICLE -------------------
def connectMyCopter():
    print("Connecting to drone...")
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()
    vehicle = connect("/dev/ttyAMA0", baud=57600)
    print("Connected.")
    return vehicle

vehicle = connectMyCopter()
print("Vehicle status checked.")

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

# ------------------- FLIGHT CONTROL FUNCTIONS -------------------
def manual_arm():
    print("Waiting for vehicle to initialize...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Waiting for arming...")
    while not vehicle.armed:
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    print("Vehicle armed.")

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
        0b0000111111000111,  # velocity only
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# ------------------- PRECISION LANDING -------------------
def precision_land_pixel_offset():
    print("Precision landing initiated.")
    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None:
            print(f"[DEBUG] Detected IDs: {ids.flatten()}")
        else:
            print("[DEBUG] No markers detected.")

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
                altitude = 10.0  # fallback

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
                print("Final height reached. Landing.")
                vehicle.mode = VehicleMode("LAND")
                break
        else:
            send_ned_velocity(0, 0, 0)
        time.sleep(0.1)

# ------------------- WAYPOINT LOGIC -------------------
def goto_waypoint(waypoint, number):
    print(f"Heading to waypoint {number}...")
    vehicle.simple_goto(waypoint)
    while True:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            print(f"[WAYPOINT DEBUG] Marker(s) seen: {ids.flatten()}")
        if ids is not None and marker_id in ids:
            print("Marker detected mid-mission. Switching to precision landing.")
            precision_land_pixel_offset()
            return False

        current_location = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current_location)
        if dist < 0.5:
            print(f"Reached waypoint {number}.")
            return True

        time.sleep(0.5)

# ------------------- MISSION EXECUTION -------------------
manual_arm()
takeoff(takeoff_altitude)

waypoints = [
    LocationGlobalRelative(27.9873411, -82.3012447, takeoff_altitude),
    LocationGlobalRelative(27.9871291, -82.3012541, takeoff_altitude),
    LocationGlobalRelative(27.9871243, -82.3016618, takeoff_altitude),
    LocationGlobalRelative(27.9873340, -82.3016605, takeoff_altitude),
    LocationGlobalRelative(27.9873411, -82.3012447, takeoff_altitude)
]

for i, wp in enumerate(waypoints):
    success = goto_waypoint(wp, i + 1)
    if not success:
        break

print("Landing after waypoints or successful detection.")
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    time.sleep(1)

picam2.stop()
vehicle.close()
print("Mission complete.")
