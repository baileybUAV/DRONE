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
near_center_threshold = 15
far_Kp = 0.0025
near_Kp = 0.0015
marker_found_flag = threading.Event()

file = open("log_uav.txt", "w")

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

# Function to establish telemetry connection between Raspberry Pis

def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # USB telemetry module
    baud_rate = 57600  # Ensure the correct baud rate
    
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link

telem_link = setup_telem_connection()

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
            print("MARKER FOUND! Triggering precision landing...")
            marker_found_flag.set()
            break
        time.sleep(0.5)

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
    print("Beginning precision landing...")
    send_ned_velocity(-1, 0, 0)
    time.sleep(1)
    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
        if ids is not None and marker_id in ids:
            #LOG DISCOVERY
            file.write("Marker Found!\n")
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
                    send_ned_velocity(0, 0, descent_vz)
                else:
                    vx = -dy * Kp
                    vy = dx * Kp
                    send_ned_velocity(vx, vy, descent_vz)
            else:
                print("Reached final height. Getting GPS Lock....")
                time.sleep(5)
                print("GPS Lock Acquired...")
                print("Starting data transmission...")
                start_time = time.time()
                while time.time() - start_time < 10:
                    #LOG LOCATION

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
                        file.write("Location Found:" + str(lat) + "/n" + str(lon))
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
                        print(f"Sent Aruco Location Data: Lat {lat}, Lon {lon}, Alt {alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")
                        time.sleep(2)  # Send every 2 seconds
                        print("DropZone Location Sent!")
                        #LOG TRANSMISSION
                        file.write("Location Sent to UGV")
                        print("Switching to LAND mode...")
                        vehicle.mode = VehicleMode("LAND")
                break
        else:
            send_ned_velocity(0, 0, 0)
        time.sleep(0.1)

# ------------------- MAIN MISSION -------------------
print("Starting mission...")
#LOG START TIME
file.write("Start Time:" + str(time.strftime("%Y-%m-%d %H:%M:%S")))
print("Start Time:", time.strftime("%Y-%m-%d %H:%M:%S"))  # Local time
manual_arm()
takeoff(takeoff_altitude)
vehicle.airspeed = 3

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

picam2.stop()
vehicle.close()
#LOG END TIME
file.wirte("End Time:" + str(time.strftime("%Y-%m-%d %H:%M:%S")))
print("End Time:", + time.strftime("%Y-%m-%d %H:%M:%S"))  # Local time
print("Mission completed.")
file.close()
exit()