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


# ------------------- CONFIG -------------------
takeoff_altitude = 6  # meters
camera_resolution = (1280, 720)
marker_id = 0
marker_size = 0.253  # meters
descent_speed = 0.2
final_land_height = 1.0
fast_descent_speed = 0.30
slow_descent_speed = 0.12
slow_down_altitude = 3.0
far_center_threshold = 50
near_center_threshold = 20
Kp = 0.0005
Kd = 0.0005
marker_found_flag = threading.Event()

# ------------------- LOGGING SETUP -------------------
logging.basicConfig(
    filename='drone_mission_log.txt',
    filemode='w',
    level=logging.INFO,
    format='%(asctime)s [%(levelname)s]: %(message)s',
    datefmt='%Y-%m-%d %H:%M:%S'
)
logger = logging.getLogger()


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

def goto_waypoint(waypoint, num):
    print(f"Going to waypoint {num}...")
    vehicle.simple_goto(waypoint)
    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current)
        print(f"Distance to waypoint {num}: {dist:.2f}m")
        if dist < 0.5 or marker_found_flag.is_set():
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
            print("DropZone Found! Triggering precision landing...")
            logger.info("DropZone Found! Triggering precision landing...")
            marker_found_flag.set()
            break
        time.sleep(0.5)

def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"
    baud_rate = 57600
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
    print("Beginning precision landing with PD control...")

    dx_prev, dy_prev = 0, 0
    last_time = time.time()

    while vehicle.armed:
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

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

            dx_rate = (dx - dx_prev) / dt
            dy_rate = (dy - dy_prev) / dt
            dx_prev, dy_prev = dx, dy

            altitude = vehicle.rangefinder.distance
            if altitude is None or altitude <= 0:
                altitude = 10.0  # fallback

            # Adaptive center threshold
            if altitude > slow_down_altitude:
                current_threshold = far_center_threshold
            else:
                current_threshold = near_center_threshold

            print(f"Offset dx={dx}, dy={dy}, dx_rate={dx_rate:.1f}, dy_rate={dy_rate:.1f}, alt={altitude:.2f}, threshold={current_threshold}")

            if abs(dx) < current_threshold and abs(dy) < current_threshold:
                if altitude > final_land_height:
                    print("[DESCENT] Centered. Descending...")
                    send_ned_velocity(0, 0, descent_speed)
                else:
                    print("[LAND] Centered and low. Switching to LAND.")
                    vehicle.mode = VehicleMode("LAND")
                    break
            else:
                vx = -dy * Kp - dy_rate * Kd
                vy = dx * Kp + dx_rate * Kd
                print(f"[PD-CONTROL] vx={vx:.3f}, vy={vy:.3f}")
                send_ned_velocity(vx, vy, 0)

        else:
            print("[INFO] Marker not detected. Hovering.")
            send_ned_velocity(0, 0, 0)

        time.sleep(0.1)

# ------------------- MAIN MISSION -------------------
print("Starting mission...")
logger.info("Starting mission...")
manual_arm()


takeoff(takeoff_altitude)


watcher_thread = threading.Thread(target=marker_watcher, daemon=True)
watcher_thread.start()

waypoints = [
    LocationGlobalRelative(27.9865908,-82.3017772, 6),
    LocationGlobalRelative(27.9865914,-82.3016015, 6),
    LocationGlobalRelative(27.9866785,-82.3015860, 6),
    LocationGlobalRelative(27.9866660,-82.3017711, 6)
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


logger.info("Mission completed.")
logger.info("Logging Ended.")
print("Mission completed.")

picam2.stop()
vehicle.close() 
exit()
# ------------------- END OF SCRIPT -------------------
