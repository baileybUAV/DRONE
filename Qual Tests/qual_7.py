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

# ------------------- VEHICLE CONNECTION -------------------
def connectMyCopter():
    print("Start Pi Connection")
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600
    vehicle = connect(connection_string, baud=baud_rate)

    print("Connected to vehicle")
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
    print("Vehicle armed and ready.")

def takeoff(target_alt):
    print("Taking off...")
    vehicle.simple_takeoff(target_alt)
    while True:
        alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {alt:.2f}m")
        if alt >= target_alt * 0.85:
            break
        time.sleep(1)
    vehicle.airspeed = 2

def distance_to(target, current):
    dlat = target.lat - current.lat
    dlong = target.lon - current.lon
    return math.sqrt((dlat**2) + (dlong**2)) * 1.113195e5

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

# ------------------- ARUCO DETECTION -------------------
def detect_marker():
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.undistort(img, camera_matrix, camera_distortion)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None and marker_id in ids:
        return corners[ids.tolist().index([marker_id])]
    return None

def precision_land():
    print("Precision landing started...")
    while vehicle.armed:
        marker = detect_marker()
        if marker is not None:
            c = marker[0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))
            dx = cx - camera_resolution[0] // 2
            dy = cy - camera_resolution[1] // 2

            altitude = vehicle.rangefinder.distance or 10.0

            if altitude > slow_down_altitude:
                vz = fast_descent_speed
                thresh = far_center_threshold
                kp = far_Kp
            else:
                vz = slow_descent_speed
                thresh = near_center_threshold
                kp = near_Kp

            print(f"dx={dx}, dy={dy}, Alt={altitude:.2f}m")
            if altitude > final_land_height:
                if abs(dx) < thresh and abs(dy) < thresh:
                    print("Centered, descending...")
                    send_ned_velocity(0, 0, vz)
                else:
                    vx = -dy * kp
                    vy = dx * kp
                    print(f"Correcting: vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f}")
                    send_ned_velocity(vx, vy, vz)
            else:
                print("Landing...")
                vehicle.mode = VehicleMode("LAND")
                break
        else:
            print("Marker not detected. Hovering.")
            send_ned_velocity(0, 0, 0)
        time.sleep(0.1)

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
    print(f"Going to waypoint {i+1}...")
    vehicle.simple_goto(wp)
    last_cmd_time = time.time()

    while True:
        current_location = vehicle.location.global_relative_frame
        dist = distance_to(wp, current_location)
        print(f"[Waypoint {i+1}] Distance: {dist:.2f}m")

        if time.time() - last_cmd_time > 5:
            vehicle.simple_goto(wp)
            last_cmd_time = time.time()

        marker = detect_marker()
        if marker is not None:
            print("Marker found! Initiating precision landing.")
            vehicle.mode = VehicleMode("GUIDED")
            time.sleep(1)
            precision_land()
            break

        if dist < 0.5:
            print(f"Reached waypoint {i+1}")
            break
        time.sleep(1)

    if vehicle.mode.name == "LAND":
        break

print("Mission complete or interrupted. Landing.")
vehicle.mode = VehicleMode("LAND")
while vehicle.armed:
    time.sleep(1)

picam2.stop()
vehicle.close()
