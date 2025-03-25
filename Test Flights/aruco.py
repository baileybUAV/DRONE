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

takeoff_altitude = 6  # meters
marker_id = 0
marker_size = 0.253  # centi? meters
angle_threshold = 3 * (math.pi / 180)  # radians
land_alt_threshold = 0.5  # meters
descent_speed = 0.2  # m/s
update_freq = 5  # Hz
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

    print("    Waiting for manual arming...")
    while not vehicle.armed:
        print("    Vehicle not armed yet...")
        time.sleep(1)

    print("    Vehicle armed. Taking off!")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.simple_takeoff(target_alt)

    while True:
        current_alt = vehicle.location.global_relative_frame.alt
        print(f"    Altitude: {current_alt:.2f}")
        if current_alt >= target_alt * 0.85:
            print("    Reached target altitude.")
            break
        time.sleep(1)


# ------------------- UTILS -------------------


# ------------------- LANDING HELPERS -------------------
def send_land_message(x, y):
    msg = vehicle.message_factory.landing_target_encode(
        0,
        0,
        mavutil.mavlink.MAV_FRAME_BODY_FRD,
        x, y,
        0, 0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

    ####### Function to send velocity commands in the NED frame
def send_ned_velocity(velocity_x, velocity_y, velocity_z):
    """
    Move vehicle in direction based on specified velocity vectors.
    velocity_x: velocity in the North direction (m/s)
    velocity_y: velocity in the East direction (m/s)
    velocity_z: velocity in the Down direction (m/s, positive is downward)
    duration: time to apply the movement (seconds)
    """
    print(f"Setting NED velocity: x={velocity_x}, y={velocity_y}, z={velocity_z}")

    # Send the velocity command once
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms (not used)
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_BODY_NED,  # frame relative to drone's orientation
        0b0000111111000111,  # type_mask (only velocity enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # x, y, z acceleration (not used)
        0, 0)  # yaw, yaw_rate (not used, no yaw change)
    vehicle.send_mavlink(msg)



# ------------------- PRECISION LANDING -------------------
def precision_land():
    print("Initiating precision landing sequence...")

    HFOV = 110 * math.pi / 180  # Horizontal FOV in radians
    VFOV = HFOV * (camera_resolution[1] / camera_resolution[0])  # Vertical FOV in radians

    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            index = np.where(ids == marker_id)[0][0]
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size, camera_matrix, camera_distortion)
            tvec = tvecs[index][0]

            x_sum = sum([p[0] for p in corners[index][0]])
            y_sum = sum([p[1] for p in corners[index][0]])
            x_avg = x_sum / 4
            y_avg = y_sum / 4

            dx = x_avg - (camera_resolution[0] / 2)
            dy = y_avg - (camera_resolution[1] / 2)

            x_ang = dx * (HFOV / camera_resolution[0])
            y_ang = dy * (VFOV / camera_resolution[1])

            # Get altitude for LANDING_TARGET message
            measured_distance = vehicle.rangefinder.distance
            if measured_distance is None or measured_distance <= 0:
                measured_distance = 10.0  # fallback if LiDAR fails

            send_land_message(x_ang, y_ang, measured_distance)

            print(f"DropZone detected at angle x={math.degrees(x_ang):.2f}, y={math.degrees(y_ang):.2f}")

            if abs(x_ang) < angle_threshold and abs(y_ang) < angle_threshold:
                if vehicle.mode.name != 'LAND':
                    print("Centered. Switching to LAND mode and letting PLND guide descent...")
                    vehicle.mode = VehicleMode('LAND')
            else:
                vx = x_ang * 2
                vy = y_ang * 2
                print(f"Sending correction velocity vx={vx:.2f}, vy={vy:.2f}")
                send_ned_velocity(-vx, -vy, 0)

        else:
            print("Marker not found. Hovering...")
            send_ned_velocity(0, 0, 0)

        time.sleep(0.1)  # 10 Hz update rate

# ------------------- RUN TEST -------------------


# ------------------- CONNECT TO VEHICLE -------------------

print("Starting test flight...")
print(angle_threshold)
manual_arm_and_takeoff(takeoff_altitude)
time.sleep(2)

precision_land()

print("Landed successfully.")
vehicle.close()
picam2.stop()
