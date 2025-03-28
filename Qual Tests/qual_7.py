from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
import threading
from picamera2 import Picamera2
from geopy.distance import geodesic as geopy_distance

# ------------------- CONFIGURATION -------------------
takeoff_height = 6  # meters
camera_resolution = (1280, 720)
marker_id = 0
marker_size = 0.253  # meters (black square only)
descent_speed = 0.2  # m/s downward
final_land_height = 1.0  # meters above target
fast_descent_speed = 0.35
slow_descent_speed = 0.15
slow_down_altitude = 3.0
far_center_threshold = 50  # pixels
near_center_threshold = 15  # pixels
far_Kp = 0.004
near_Kp = 0.002

# Shared marker detection event
marker_found_event = threading.Event()

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

#Wait for manual arming function
def manual_arm():
  print ("    Pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print ("    Waiting for vehicle to initialise...")
    time.sleep(1)

  while not vehicle.armed:
    print ("    Waiting for arming...")
    time.sleep(1)

  print("   Waiting for manual arming...")
  while not vehicle.armed:
    print("   Waiting for arming...")
    time.sleep(1)

  vehicle.mode = VehicleMode("GUIDED")

  print("   Vehicle armed.")
  print("   Mode: %s" % vehicle.mode.name) 

# Function to arm and then takeoff to a specified altitude
def takeoff(aTargetAltitude):
  print ("    Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print ("    Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.80:
      print ("    Reached target altitude")
      #print("Height from Lidar: " % vehicle.rangefinder1)
      break
    time.sleep(1)

def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"
    baud_rate = 57600
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link

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

def precision_land_pixel_offset():
    print("Beginning precision landing...")
    

    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            print("Drop Zone Detected!")
            marker_found_event.set()  # Set the event when the marker is detected

            index = np.where(ids == marker_id)[0][0]
            c = corners[index][0]
            cx = int(np.mean(c[:, 0]))
            cy = int(np.mean(c[:, 1]))

            frame_center = (camera_resolution[0] // 2, camera_resolution[1] // 2)
            dx = cx - frame_center[0]
            dy = cy - frame_center[1]

            altitude = vehicle.rangefinder.distance
            if altitude is None or altitude <= 0:
                altitude = 10.0  # fallback if LiDAR fails

            # Adaptive parameters based on altitude
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
                    print(f"Correcting position: vx={vx:.2f}, vy={vy:.2f}, vz={descent_vz}")
                    send_ned_velocity(vx, vy, descent_vz)
            else:
                print("Reached final height. Switching to LAND.")
                vehicle.mode = VehicleMode("LAND")
                break

        else:
            print("Marker not detected. Hovering.")
            break

        time.sleep(0.1)  # 10 Hz

def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5

def goto_waypoint(waypoint, waypoint_number):
    print(f"Going to waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint)
    while True:
        if marker_found_event.is_set():
            print("Marker detected! Interrupting mission for landing...")
            return False  # Abort mission after landing
        current_location = vehicle.location.global_relative_frame
        distance = distance_to(waypoint, current_location)
        if distance < 0.5:
            print(f"Reached waypoint {waypoint_number}")
            return True
        time.sleep(1)

def land():
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    vehicle.close()


# ---- Begin mission ----
vehicle = connectMyCopter()
print("Connected Pi to drone.")
telem_link = setup_telem_connection()
print("Telemetry link established!")
print("Starting Test Flight...")



manual_arm()
takeoff(takeoff_height)

marker_thread = threading.Thread(target=precision_land_pixel_offset, daemon=True)
marker_thread.start()

# Waypoints List (add coordinates here)
waypoints = [
    LocationGlobalRelative(27.9868129,-82.3016216, 6),
    LocationGlobalRelative(27.9868271,-82.3012877, 6),
    LocationGlobalRelative(27.9867146,-82.3012809, 6),
    LocationGlobalRelative(27.9867122,-82.3016216, 6),
    LocationGlobalRelative(27.9867773,-82.3014553, 6)
]


for i, waypoint in enumerate(waypoints):
    result = goto_waypoint(waypoint, i + 1)
    if not result:
        break  # Marker found and landing occurred

if not marker_found_event.is_set():
    print("No marker found during mission. Proceeding to land.")
    land()


print("Mission Complete.")
picam2.stop()
exit()
