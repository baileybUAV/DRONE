from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
from picamera2 import Picamera2

# ------------------- CONFIGURATION -------------------
connection_string = "/dev/ttyAMA0"
baud_rate = 57600
takeoff_altitude = 6  # meters
marker_id = 1
marker_size = 0.253  # meters
angle_threshold = 15 * (math.pi / 180)  # radians
land_alt_threshold = 0.5  # meters
descent_speed = 0.2  # m/s
update_freq = 1.0  # Hz
camera_resolution = (1280, 720)

# ------------------- CAMERA SETUP -------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ------------------- CV/ARUCO -------------------
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')



# ------------------- MANUAL ARM + TAKEOFF -------------------
def manual_arm_and_takeoff(target_alt):
    print("    Pre-arm checks...")
    while not vehicle.is_armable:
        print("    Waiting for vehicle to initialize...")
        time.sleep(1)

    print("    Waiting for manual arming (via RC/GCS)...")
    while not vehicle.armed:
        print("    Vehicle not armed yet...")
        time.sleep(1)

    print("    Vehicle armed. Taking off!")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"    Altitude: {current_alt:.2f}")
        if current_alt >= target_alt * 0.80:
            print("    Reached target altitude.")
            break
        time.sleep(1)


# ------------------- UTILS -------------------
def get_location_metres(original_location, dNorth, dEast):
    earth_radius = 6378137.0
    dLat = dNorth / earth_radius
    dLon = dEast / (earth_radius * math.cos(math.pi * original_location.lat / 180))
    newlat = original_location.lat + (dLat * 180 / math.pi)
    newlon = original_location.lon + (dLon * 180 / math.pi)
    return LocationGlobalRelative(newlat, newlon, original_location.alt)

def detect_marker():
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.undistort(img, camera_matrix, camera_distortion)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None and marker_id in ids:
        index = np.where(ids == marker_id)[0][0]
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
        return True, tvecs[index][0]
    return False, None

def precision_landing():
    print("Hovering and waiting for marker...")
    while True:
        marker_found, tvec = detect_marker()

        if not marker_found:
            print("No marker. Holding hover...")
            vehicle.simple_goto(vehicle.location.global_relative_frame)
            time.sleep(1)
            continue

        x, y, z = tvec
        angle_x = math.atan2(x, z)
        angle_y = math.atan2(y, z)

        print(f"Marker offset (cm): x={x*100:.1f}, y={y*100:.1f}, z={z*100:.1f}")
        print(f"Angle: {math.degrees(angle_x):.1f}°, {math.degrees(angle_y):.1f}°")

        # Convert to North/East movement based on yaw
        x_uav = -y
        y_uav = x
        yaw = vehicle.attitude.yaw
        north = x_uav * math.cos(yaw) - y_uav * math.sin(yaw)
        east = x_uav * math.sin(yaw) + y_uav * math.cos(yaw)

        current = vehicle.location.global_relative_frame
        target = get_location_metres(current, north, east)

        if math.sqrt(angle_x**2 + angle_y**2) <= angle_threshold:
            new_alt = max(current.alt - descent_speed / update_freq, land_alt_threshold)
            print("Marker aligned. Descending.")
        else:
            new_alt = current.alt
            print("Correcting position, not descending.")

        vehicle.simple_goto(LocationGlobalRelative(target.lat, target.lon, new_alt))
        time.sleep(1 / update_freq)

        if current.alt <= land_alt_threshold + 0.1:
            print("Reached landing threshold. Landing now.")
            vehicle.mode = VehicleMode("LAND")
            break





# ------------------- RUN TEST -------------------


# ------------------- CONNECT TO VEHICLE -------------------
print("Connecting Pi to vehicle...")
vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
print("Pi Connected.")
print("Starting test flight...")
manual_arm_and_takeoff(takeoff_altitude)

precision_landing()

while vehicle.armed:
    time.sleep(1)

print("Landed successfully.")
vehicle.close()
picam2.stop()
