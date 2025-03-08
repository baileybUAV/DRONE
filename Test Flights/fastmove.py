
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

  print("Connecting...")
  vehicle = connect(connection_string,baud=baud_rate) 
  print("GPS: %s" % vehicle.gps_0)
  print("Battery: %s" % vehicle.battery)
  print("Armable?: %s" % vehicle.is_armable)
  #print("Height from Lidar: " % vehicle.rangefinder1)
  print("Mode: %s" % vehicle.mode.name)
  #print("GPS Location: " % vehicle.location.global_frame)    

  return vehicle

  
vehicle = connectMyCopter()
print("Connected")


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

# Function to calculate distance between two GPS coordinates
def distance_to(target_location, current_location):
    dlat = target_location.lat - current_location.lat
    dlong = target_location.lon - current_location.lon
    return math.sqrt((dlat ** 2) + (dlong ** 2)) * 1.113195e5  # Convert lat/lon degrees to meters


# Function to move to a waypoint and check when it is reached
def goto_waypoint(waypoint, waypoint_number):
    print(f"Going towards waypoint {waypoint_number}...")
    vehicle.simple_goto(waypoint)

    while True:
        current_location = vehicle.location.global_relative_frame
        distance = distance_to(waypoint, current_location)

        if distance < 0.5:  # Stop when within 1 meter of the target
            print(f"Reached waypoint {waypoint_number}")
            break

        print(f"Distance to waypoint {waypoint_number}: {distance:.2f}m")
        time.sleep(1)  # Check every second


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

takeoff(4)  # Takeoff to 4 meters
print("MAIN: TakeOff Completed")

print("Setting airspeed to 3 m/s")
vehicle.airspeed = 3

# Waypoints List
waypoints = [
    LocationGlobalRelative(27.9873411, -82.3012447, 4),
    LocationGlobalRelative(27.9871291, -82.3012541, 4),
    LocationGlobalRelative(27.9871243, -82.3016618, 4),
    LocationGlobalRelative(27.9873340, -82.3016605, 4),
    LocationGlobalRelative(27.9873411, -82.3012447, 4)  # Returning to first waypoint
]

# Move through waypoints without waiting
for i, waypoint in enumerate(waypoints):
    goto_waypoint(waypoint, i + 1)

land()

print("MAIN: If the drone is not upside down, congrats!")
exit()