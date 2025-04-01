from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
from geopy.distance import distance as geopy_distance
import time, math, argparse
import cv2
import cv2.aruco as aruco
import numpy as np
from picamera2 import Picamera2

# ------------------- CONFIG -------------------
takeoff_altitude = 4
camera_resolution = (1280, 720)
marker_id = 0
marker_size = 0.253
FOV_X = math.radians(110)
FOV_Y = math.radians(85)
Kp_ang = 0.5
Kd_ang = 1
angle_threshold = 0.03
final_land_height = 1.5
descent_speed = 0.2

# ------------------- CONNECT -------------------
def connectMyCopter():
    parser = argparse.ArgumentParser()
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = "/dev/ttyAMA0"
    vehicle = connect(connection_string, baud=57600)
    return vehicle

vehicle = connectMyCopter()
print("Connected")

# ------------------- ARM + TAKEOFF -------------------
def manaul_arm():
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    print("Vehicle armed.")

def takeoff(aTargetAltitude):
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.8:
            print("Reached target altitude")
            break
        time.sleep(1)

# ------------------- CAMERA SETUP -------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

calib_path = "/home/uav/drone/OpenCV/calibrationFiles/"
camera_matrix = np.loadtxt(calib_path + 'cameraMatrix.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path + 'cameraDistortion.txt', delimiter=',')
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
parameters = aruco.DetectorParameters()

# ------------------- NED VELOCITY -------------------
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

# ------------------- LANDING -------------------
def precision_land_angle_pd():
    print("Switching to angle-based precision landing...")
    x_ang_prev, y_ang_prev = 0, 0
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
            cx0 = camera_resolution[0] // 2
            cy0 = camera_resolution[1] // 2

            x_ang = (cx - cx0) * (FOV_X / camera_resolution[0])
            y_ang = (cy - cy0) * (FOV_Y / camera_resolution[1])
            x_ang_rate = (x_ang - x_ang_prev) / dt
            y_ang_rate = (y_ang - y_ang_prev) / dt
            x_ang_prev, y_ang_prev = x_ang, y_ang

            print(f"Angle x={math.degrees(x_ang):.2f}°, y={math.degrees(y_ang):.2f}°")

            altitude = vehicle.rangefinder.distance
            if altitude is None or altitude <= 0:
                altitude = 10.0

            if abs(x_ang) < angle_threshold and abs(y_ang) < angle_threshold:
                if altitude > final_land_height:
                    print("[DESCENT] Centered. Descending...")
                    send_ned_velocity(0, 0, descent_speed)
                else:
                    print("[LAND] Centered and low. Switching to LAND.")
                    vehicle.mode = VehicleMode("LAND")
                    break
            else:
                vx = -y_ang * Kp_ang - y_ang_rate * Kd_ang
                vy =  x_ang * Kp_ang + x_ang_rate * Kd_ang
                print(f"[PD-ANGLE] vx={vx:.3f}, vy={vy:.3f}")
                send_ned_velocity(vx, vy, 0)

        else:
            print("[INFO] Marker not detected. Hovering.")
            send_ned_velocity(0, 0, 0)

        time.sleep(0.1)

# ------------------- WAYPOINT TRAVEL -------------------
def distance_to(target, current):
    dlat = target.lat - current.lat
    dlong = target.lon - current.lon
    return math.sqrt((dlat**2 + dlong**2)) * 1.113195e5

def goto_waypoint_with_detection(waypoint, waypoint_number):
    print(f"Navigating to waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint)

    while True:
        current = vehicle.location.global_relative_frame
        dist = distance_to(waypoint, current)
        print(f"Distance to waypoint {waypoint_number}: {dist:.2f}m")

        # ArUco detection during flight
        img = picam2.capture_array()
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        img = cv2.undistort(img, camera_matrix, camera_distortion)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None and marker_id in ids:
            print(f"[MARKER] Detected marker {marker_id}. Interrupting mission.")
            precision_land_angle_pd()
            return "interrupted"

        if dist < 0.5:
            print(f"Reached waypoint {waypoint_number}")
            return "reached"

        time.sleep(0.5)

# ------------------- FLIGHT MISSION -------------------
print("Starting mission...")
manaul_arm()
takeoff(takeoff_altitude)
vehicle.airspeed = 3

waypoints = [
    LocationGlobalRelative(27.9865908, -82.3017772, 6),
    LocationGlobalRelative(27.9865914, -82.3016015, 6),
    LocationGlobalRelative(27.9866785, -82.3015860, 6),
    LocationGlobalRelative(27.9866660, -82.3017711, 6)
]

for i, wp in enumerate(waypoints):
    result = goto_waypoint_with_detection(wp, i + 1)
    if result == "interrupted":
        break

print("Mission complete.")
vehicle.close()
picam2.stop()