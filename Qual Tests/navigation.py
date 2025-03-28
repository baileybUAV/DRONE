from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import threading
from picamera2 import Picamera2

# ------------------- CONFIGURATION -------------------
takeoff_altitude = 6
camera_resolution = (1280, 720)
marker_id = 0
marker_size = 0.253  # meters
fast_descent_speed = 0.35
slow_descent_speed = 0.15
slow_down_altitude = 3.0
final_land_height = 1.0
far_center_threshold = 50
near_center_threshold = 15
far_Kp = 0.004
near_Kp = 0.002

# Shared flag for precision landing trigger
precision_landing_event = threading.Event()

# ------------------- CONNECT TO VEHICLE -------------------
def connectMyCopter():
    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600
    print("Connecting to drone...")
    vehicle = connect(connection_string, baud=baud_rate)
    print("Connected!")
    return vehicle

vehicle = connectMyCopter()

# ------------------- CAMERA SETUP -------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ------------------- LOAD CALIBRATION FILES -------------------
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# ------------------- FUNCTIONS -------------------
def manual_arm_and_takeoff(target_alt):
    print("Waiting for vehicle to become armable...")
    while not vehicle.is_armable:
        time.sleep(1)

    print("Waiting for user to arm the vehicle...")
    while not vehicle.armed:
        time.sleep(1)

    print("Armed. Taking off...")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"Altitude: {current_alt:.2f}m")
        if current_alt >= target_alt * 0.85:
            print("Target altitude reached.")
            break
        time.sleep(1)

def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
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

def marker_detection_loop():
    while vehicle.armed and not precision_landing_event.is_set():
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            print("Marker detected!")
            precision_landing_event.set()

        time.sleep(0.1)

def precision_land():
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
                    print("Marker centered. Descending...")
                    send_ned_velocity(0, 0, descent_vz)
                else:
                    vx = -dy * Kp
                    vy = dx * Kp
                    print(f"Adjusting: vx={vx:.2f}, vy={vy:.2f}, vz={descent_vz}")
                    send_ned_velocity(vx, vy, descent_vz)
            else:
                print("Final landing height reached. LAND mode activated.")
                vehicle.mode = VehicleMode("LAND")
                break
        else:
            print("Marker lost during landing. Hovering.")
            send_ned_velocity(0, 0, 0)

        time.sleep(0.1)

def fly_through_waypoints(waypoints):
    for i, waypoint in enumerate(waypoints):
        print(f"Navigating to waypoint {i+1}...")
        vehicle.simple_goto(waypoint)

        while True:
            if precision_landing_event.is_set():
                return

            current_location = vehicle.location.global_relative_frame
            distance = distance_to(waypoint, current_location)

            if distance < 0.5:
                print(f"Reached waypoint {i+1}")
                break

            time.sleep(0.5)

# ------------------- MISSION -------------------
manual_arm_and_takeoff(takeoff_altitude)

waypoints = [
    LocationGlobalRelative(27.9873411, -82.3012447, 6),
    LocationGlobalRelative(27.9871291, -82.3012541, 6),
    LocationGlobalRelative(27.9871243, -82.3016618, 6),
    LocationGlobalRelative(27.9873340, -82.3016605, 6),
    LocationGlobalRelative(27.9873411, -82.3012447, 6)
]

# Start detection thread
detection_thread = threading.Thread(target=marker_detection_loop, daemon=True)
detection_thread.start()

# Navigate
fly_through_waypoints(waypoints)

# If triggered, precision land
if precision_landing_event.is_set():
    precision_land()

print("Mission complete. Cleaning up.")
vehicle.close()
picam2.stop()
