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
  vehicle = connect(connection_string,baud=baud_rate) 
  print("GPS: %s" % vehicle.gps_0)
  print("Battery: %s" % vehicle.battery)
  print("Armable?: %s" % vehicle.is_armable)
  print("Height from Lidar: %s" % vehicle.rangefinder)
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

# Send position target in local NED frame (used for 50-yard forward movement)
def send_position_target_local_ned_example():
    print("Sending SET_POSITION_TARGET_LOCAL_NED message for forward movement")
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,  # time_boot_ms
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame
        0b0000111111111000,  # type_mask: only positions enabled
        30, 0, -6,  # North, East, Down: 50 yards forward, maintaining altitude
        0, 0, 0,  # velocity
        0, 0, 0,  # acceleration
        0, 0      # yaw, yaw_rate
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

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

manaul_arm()
print("MAIN: Manual Arm Success")

takeoff(6)
print("MAIN: TakeOff Completed")

print("MAIN: Sending position target to move 50 yards forward")
send_position_target_local_ned_example()

# Allow time to reach position
time.sleep(10)

land()
print("MAIN: If the drone is not upside down, congrats!")

exit()
