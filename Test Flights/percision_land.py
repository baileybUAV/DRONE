from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import cv2
import cv2.aruco as aruco
import numpy as np
import math
import time
import argparse
from collections import deque
from picamera2 import Picamera2

# ------------------- CONFIGURATION -------------------
connection_string = "/dev/ttyAMA0"
baud_rate = 57600
takeoff_altitude = 6  # meters
marker_id = 1
marker_size = 0.253  # centi? meters
angle_threshold = 15 * (math.pi / 180)  # radians
land_alt_threshold = 0.5  # meters
descent_speed = 0.2  # m/s
update_freq = 1.0  # Hz
camera_resolution = (1280, 720)

#Connect pi to drone first

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

    # Simple low-pass filter buffer
    buffer_len = 5
    x_buffer = deque(maxlen=buffer_len)
    y_buffer = deque(maxlen=buffer_len)
    z_buffer = deque(maxlen=buffer_len)

    stable_count = 0
    stable_required = 5

    while True:
        marker_found, tvec = detect_marker()

        if not marker_found:
            print("No marker. Holding hover...")
            vehicle.simple_goto(vehicle.location.global_relative_frame)
            stable_count = 0
            time.sleep(1)
            continue

        # tvec: [x, y, z] in camera frame
        x_raw, y_raw, z_raw = tvec

        # Fill buffer for smoothing
        x_buffer.append(x_raw)
        y_buffer.append(y_raw)
        z_buffer.append(z_raw)

        # Use average for smoothing
        x = np.mean(x_buffer)
        y = np.mean(y_buffer)
        z = np.mean(z_buffer)

        # For downward camera:
        # x = right → East
        # y = down → -North (flip sign)
        # z = altitude above marker

        north_offset = -y
        east_offset = x

        # Use vehicle yaw to rotate into global frame
        yaw = vehicle.attitude.yaw
        north = north_offset * math.cos(yaw) - east_offset * math.sin(yaw)
        east = north_offset * math.sin(yaw) + east_offset * math.cos(yaw)

        current = vehicle.location.global_relative_frame
        target = get_location_metres(current, north, east)

        # Stability check
        angle_x = math.atan2(x, z)
        angle_y = math.atan2(y, z)
        angle_error = math.sqrt(angle_x**2 + angle_y**2)

        print(f"Smoothed marker offset (cm): x={x*100:.1f}, y={y*100:.1f}, z={z*100:.1f}")
        print(f"Angle error: {math.degrees(angle_error):.1f}°")

        if angle_error <= angle_threshold:
            stable_count += 1
            if stable_count >= stable_required:
                new_alt = max(current.alt - descent_speed / update_freq, land_alt_threshold)
                print("Marker aligned and stable. Descending.")
            else:
                new_alt = current.alt
                print("Aligning before descent...")
        else:
            stable_count = 0
            new_alt = current.alt
            print("Correcting position. Not descending yet.")

        vehicle.simple_goto(LocationGlobalRelative(target.lat, target.lon, new_alt))
        time.sleep(1 / update_freq)

        if current.alt <= land_alt_threshold + 0.1:
            print("Reached landing threshold. Landing now.")
            vehicle.mode = VehicleMode("LAND")
            break






# ------------------- RUN TEST -------------------


# ------------------- CONNECT TO VEHICLE -------------------

print("Starting test flight...")
manual_arm_and_takeoff(takeoff_altitude)

precision_landing()

while vehicle.armed:
    time.sleep(1)

print("Landed successfully.")
vehicle.close()
picam2.stop()
