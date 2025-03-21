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

#############################

# Initialize Pi Camera
width = 1280
height = 720
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": (width, height), "format": "RGB888"})
picam2.configure(config)
picam2.start()

############ ARUCO/CV2 SETTINGS ############
id_to_find = 1
marker_size = .253  # meters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_ARUCO_ORIGINAL)
parameters = aruco.DetectorParameters()
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX  # Enable corner refinement

# Load Camera Calibration Data
calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
cameraMatrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
cameraDistortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')

################ DRONEKIT SETTINGS ################
takeoff_height = 6  # meters

# Field of View for Pi Camera
horizontal_fov = 70 * (math.pi / 180)
vertical_fov = 70 * (height / width) * (math.pi / 180)  # Estimate vertical FOV

################ FUNCTIONS ################

def connectMyCopter():
    print("Connecting Pi to drone...")
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect if args.connect else "/dev/ttyAMA0"
    baud_rate = 57600
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    return vehicle

def manual_arm():
    print("    Pre-arm checks")
    while not vehicle.is_armable:
        print("    Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.armed:
        print("    Waiting for arming...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")

    print("   Vehicle armed.")
    print("   Mode: %s" % vehicle.mode.name) 

def takeoff(aTargetAltitude):
    print("    Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    while True:
        print("    Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.80:
            print("    Reached target altitude")
            break
        time.sleep(1)

# Function to establish telemetry connection between Raspberry Pis
def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # USB telemetry module
    baud_rate = 57600  # Ensure the correct baud rate
    
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link

def send_local_ned_velocity(vx, vy, vz):
    """ Sends velocity commands to adjust drone position. """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,
        0b0000111111000111,
        0, 0, 0,
        vx, vy, vz,
        0, 0, 0,
        0, 0)
    
    vehicle.send_mavlink(msg)
    vehicle.flush()

def detect_aruco_marker():
    """ Detects the ArUco marker and returns True if found. """
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    img = cv2.undistort(img, cameraMatrix, cameraDistortion)
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)
    return ids is not None and id_to_find in ids

def precision_landing():
    """ Handles precision landing using ArUco marker with velocity correction. """
    print("Initiating precision landing.")
    
    # Store last known marker position
    last_x_ang, last_y_ang = 0, 0

    while vehicle.armed:
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, cameraMatrix, cameraDistortion)
        gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(image=gray_img, dictionary=aruco_dict, parameters=parameters)

        if ids is not None and id_to_find in ids:
            print(f"Marker {id_to_find} detected! Adjusting position...")
            index = np.where(ids == id_to_find)[0][0]
            ret = aruco.estimatePoseSingleMarkers(corners, marker_size, cameraMatrix, cameraDistortion)
            rvec, tvec = ret[0][index, 0, :], ret[1][index, 0, :]

            x_avg = np.mean(corners[0][0][:, 0])
            y_avg = np.mean(corners[0][0][:, 1])

            x_ang = (x_avg - width * 0.5) * (horizontal_fov / width)
            y_ang = (y_avg - height * 0.5) * (vertical_fov / height)

            send_local_ned_velocity(-x_ang * 0.2, -y_ang * 0.2, 0)
            
            last_x_ang, last_y_ang = x_ang, y_ang
        else:
            print("Marker lost, using last known position.")

        msg = vehicle.message_factory.landing_target_encode(
            0, 0, mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED, last_x_ang, last_y_ang, 0, 0, 0)
        vehicle.send_mavlink(msg)
        vehicle.flush()

        print(f"Adjusting landing: x={last_x_ang:.4f}, y={last_y_ang:.4f}")

        time.sleep(0.5) 

        # Get UAV GPS and velocity data
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        alt = vehicle.location.global_frame.alt
        rel_alt = vehicle.location.global_relative_frame.alt
        velocity_north = vehicle.velocity[0]  # North velocity (m/s)
        velocity_east = vehicle.velocity[1]   # East velocity (m/s)
        velocity_down = vehicle.velocity[2]   # Down velocity (m/s)

        if lat is None or lon is None or lat == 0 or lon == 0:
            print("Error: No valid GPS data available")
        else:
            # Create a covariance matrix with NaN (unknown values)
            covariance_matrix = np.full((36,), float('nan'), dtype=np.float32)

            print("GPS STATUS: %s" % vehicle.gps_0.fix_type)
            # Create MAVLink message with GPS and velocity data
            msg = telem_link.mav.global_position_int_cov_encode(
                int(time.time() * 1e6),  # Timestamp (microseconds)
                mavutil.mavlink.MAV_ESTIMATOR_TYPE_GPS,  # MAV_ESTIMATOR_TYPE_GPS (3 = GPS-based estimation)
                int(lat * 1e7),  # Latitude in degrees * 1E7
                int(lon * 1e7),  # Longitude in degrees * 1E7
                int(alt * 1000),  # Altitude above MSL in mm
                int(rel_alt * 1000),  # Relative altitude in mm
                float(velocity_north),  # Velocity North (m/s)
                float(velocity_east),   # Velocity East (m/s)
                float(velocity_down),   # Velocity Down (m/s)
                covariance_matrix  # 6x6 Covariance matrix
            )


            # Send the message to the other Pi
            telem_link.mav.send(msg)
            print(f"Sent GLOBAL_POSITION_INT_COV Data: Lat {lat}, Lon {lon}, Alt {alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")
    
    print("Switching to LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.mode != 'LAND':
        print("Waiting for LAND mode...")
        time.sleep(1)

    print("Drone has landed.")

def goto_waypoint(waypoint, waypoint_number):
    print(f"Going to waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = geopy_distance((current_location.lat, current_location.lon), (waypoint.lat, waypoint.lon)).meters

        # Constantly check for ArUco marker
        if detect_aruco_marker():
            print("Marker detected! Switching to precision landing.")
            precision_landing()
            break

        if distance < 0.5:
            print(f"Reached waypoint {waypoint_number}")
            break

        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(1)

# ---- Begin mission ----
vehicle = connectMyCopter()
print("Connected to drone.")
telem_link = setup_telem_connection()

vehicle.parameters['PLND_ENABLED'] = 1
vehicle.parameters['PLND_TYPE'] = 1
vehicle.parameters['PLND_EST_TYPE'] = 0
vehicle.parameters['LAND_SPEED'] = 15

manual_arm()
takeoff(takeoff_height)

print("Setting airspeed to 5 mp/h")
vehicle.airspeed = 2.2  # m/s

# Waypoints List
waypoints = [
    LocationGlobalRelative(, , 6),
    LocationGlobalRelative(, , 6),
]

for i, waypoint in enumerate(waypoints):
    goto_waypoint(waypoint, i + 1)

precision_landing()
print("Mission Complete.")
exit()
