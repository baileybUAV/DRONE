from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
from geopy.distance import distance as geopy_distance
import time
import future
import socket
import math
import argparse
import os

# Connect to the Vehicle function
def connectMyCopter():
    print("Start Connection")
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600

    print("Connecting Pi to Drone...")
    vehicle = connect(connection_string, baud=baud_rate, wait_ready=True)
    print("GPS: %s" % vehicle.gps_0)
    print("Battery: %s" % vehicle.battery)
    print("Armable?: %s" % vehicle.is_armable)
    print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
    print("Global Location: %s" % vehicle.location.global_frame)
    print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
    print("Local Location: %s" % vehicle.location.local_frame)
    print("Mode: %s" % vehicle.mode.name)
    return vehicle

vehicle = connectMyCopter()
print("Connected")

# Wait for manual arming function
def manaul_arm():
    print("    Pre-arm checks")
    while not vehicle.is_armable:
        print("    Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.armed:
        print("    Waiting for arming...")
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
    print("    Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)
    while True:
        print("    Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.80:
            print("    Reached target altitude")
            break
        time.sleep(1)

# Function to calculate distance between two GPS coordinates
def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5

# Send local NED velocity
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

# Set local NED position
def set_position_target_local_ned(north, east, down):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0, 0, 0,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b0000111111111000,
        north, east, down,
        0, 0, 0,
        0, 0, 0,
        0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Function to move forward a specified distance in meters
def move_forward_by_distance(distance_meters, speed_mps=5):
    duration = distance_meters / speed_mps
    print(f"Moving forward {distance_meters}m for {duration:.1f} seconds...")
    start_time = time.time()
    while time.time() - start_time < duration:
        send_local_ned_velocity(speed_mps, 0, 0)
        time.sleep(0.1)
    send_local_ned_velocity(0, 0, 0)

# Function to land the drone safely
def land():
    print("Landing")
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:
        time.sleep(1)
    vehicle.close()
    print("Drone has landed safely")

# ---- Begin mission ----
print("MAIN: Code Started")
print("Current Time:", time.strftime("%Y-%m-%d %H:%M:%S"))

manaul_arm()
print("MAIN: Manual Arm Success")

takeoff(6)
print("MAIN: TakeOff Completed")

print("MAIN: Moving forward 50 yards")
move_forward_by_distance(45.72)  # 50 yards in meters

land()
print("MAIN: If the drone is not upside down, congrats!")
print("Current Time:", time.strftime("%Y-%m-%d %H:%M:%S"))
exit()
