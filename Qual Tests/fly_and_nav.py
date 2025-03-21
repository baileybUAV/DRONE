from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2
from geopy.distance import geodesic as geopy_distance

# Initialize Pi Camera
width = 1280
height = 720
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
picam2.configure(config)
picam2.start()

# ARUCO SETTINGS
id_to_find = 1
marker_size = .253  # meters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

# Load Camera Calibration Data
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

# DRONEKIT SETTINGS
takeoff_height = 6  # meters
horizontal_fov = 70 * (math.pi / 180)
vertical_fov = 70 * (height / width) * (math.pi / 180)

# FUNCTIONS
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
  print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
  print("Global Location: %s" % vehicle.location.global_frame)
  print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
  print("Local Location: %s" % vehicle.location.local_frame)
  print("Mode: %s" % vehicle.mode.name)     
  return vehicle

#Wait for manual arming function
def manaul_arm():
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

d# Function to arm and then takeoff to a specified altitude
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

def send_local_ned_velocity(vx, vy, vz):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

def detect_aruco_marker():
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.undistort(img, cameraMatrix, cameraDistortion)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    if ids is not None:
        ids = ids.flatten()
        for marker_id in ids:
            print(f"Found Marker ID: {marker_id}")
            if marker_id == 1:
                return True, marker_id, corners
    return False, None, None

def precision_landing():
    print("Initiating precision landing.")
    last_x_ang, last_y_ang = 0, 0
    descent_rate = 0.2  # m/s down in NED frame (positive value = descent)

    while vehicle.armed:
        current_alt = vehicle.location.global_relative_frame.alt
        

        if current_alt <= 1.5:
            # Get UAV GPS and velocity data
            drop_lat = vehicle.location.global_frame.lat
            drop_lon = vehicle.location.global_frame.lon
            drop_alt = vehicle.location.global_frame.alt
            rel_alt = vehicle.location.global_relative_frame.alt
            velocity_north = vehicle.velocity[0]  # North velocity (m/s)
            velocity_east = vehicle.velocity[1]   # East velocity (m/s)
            velocity_down = vehicle.velocity[2]   # Down velocity (m/s)

            if drop_lat is None or drop_lon is None or drop_lat == 0 or drop_lon == 0:
                print("Error: No valid GPS data available")
            else:
                # Create a covariance matrix with NaN (unknown values)
                covariance_matrix = np.full((36,), float('nan'), dtype=np.float32)

                print("GPS STATUS: %s" % vehicle.gps_0.fix_type)
                # Create MAVLink message with GPS and velocity data
                msg = telem_link.mav.global_position_int_cov_encode(
                    int(time.time() * 1e6),  # Timestamp (microseconds)
                    mavutil.mavlink.MAV_ESTIMATOR_TYPE_GPS,  # MAV_ESTIMATOR_TYPE_GPS (3 = GPS-based estimation)
                    int(drop_lat * 1e7),  # Latitude in degrees * 1E7
                    int(drop_lon * 1e7),  # Longitude in degrees * 1E7
                    int(drop_alt * 1000),  # Altitude above MSL in mm
                    int(rel_alt * 1000),  # Relative altitude in mm
                    float(velocity_north),  # Velocity North (m/s)
                    float(velocity_east),   # Velocity East (m/s)
                    float(velocity_down),   # Velocity Down (m/s)
                    covariance_matrix  # 6x6 Covariance matrix
                )


                    # Send the message to the other Pi
                telem_link.mav.send(msg)
                print(f"Sent GLOBAL_POSITION_INT_COV Data: Lat {drop_lat}, Lon {drop_lon}, Alt {drop_alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")
                print("Reached 1 meter AGL. Switching to LAND mode.")
                vehicle.mode = VehicleMode("LAND")
                break

        found, marker_id, corners = detect_aruco_marker()
        if found:
            print(f"Marker {marker_id} detected! Adjusting position...")
            x_avg = np.mean(corners[0][0][:, 0])
            y_avg = np.mean(corners[0][0][:, 1])
            x_ang = (x_avg - width * 0.5) * (horizontal_fov / width)
            y_ang = (y_avg - height * 0.5) * (vertical_fov / height)

            if abs(x_ang) < 0.01 and abs(y_ang) < 0.01:
                print("Marker centered. Descending...")
                send_local_ned_velocity(0, 0, descent_rate)
            else:
                send_local_ned_velocity(-x_ang * 0.3, -y_ang * 0.3, 0)

            last_x_ang, last_y_ang = x_ang, y_ang
        else:
            print("Marker lost, descending while maintaining last direction.")
            send_local_ned_velocity(0, 0, descent_rate)

        msg = vehicle.message_factory.landing_target_encode(
            0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
            last_x_ang, last_y_ang, 0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()

        time.sleep(0.5)


def goto_waypoint(waypoint, waypoint_number):
    print(f"Going to waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint)
    while True:
        current_location = vehicle.location.global_relative_frame
        distance = geopy_distance((current_location.lat, current_location.lon),
                                  (waypoint.lat, waypoint.lon)).meters
        found, marker_id, marker_type = detect_aruco_marker()
        if found:
            print(f"DropZone marker ({marker_id}) detected! Switching to precision landing.")
            precision_landing()
            break
        if distance < 0.5:
            print(f"Reached waypoint {waypoint_number}")
            print("Landing")
            vehicle.mode = VehicleMode("LAND")
            while vehicle.armed:
                time.sleep(1)
            vehicle.close()
            print("Drone has landed safely")
            exit()
            
        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(1)

# ---- Begin mission ----
vehicle = connectMyCopter()
print("Connected to drone.")
telem_link = setup_telem_connection()

#vehicle.parameters['PLND_ENABLED'] = 1 #enable precision landing
vehicle.parameters['PLND_TYPE'] = 1 #1 for companion computer
vehicle.parameters['PLND_EST_TYPE'] = 0 #0 for raw sensor, 1 for kalman filter pos estimation
vehicle.parameters['LAND_SPEED'] = 15 #speed in cm/s

manual_arm()
takeoff(takeoff_height)
print("Setting airspeed to 5 mp/h")
vehicle.airspeed = 1

# Waypoints List (add coordinates here)
waypoints = [
    LocationGlobalRelative(27.9867939, -82.3009524, 6)
]

for i, waypoint in enumerate(waypoints):
    goto_waypoint(waypoint, i + 1)

precision_landing()
print("Mission Complete.")
exit()
