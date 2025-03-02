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
  print("Height from Lidar: " % vehicle.rangefinder)
  print("Mode: %s" % vehicle.mode.name)
  print("GPS Location: " % vehicle.location.global_frame)    

  return vehicle

  
vehicle = connectMyCopter()
print("Connected")


#Wait for manual arming function
def manaul_arm():
  print ("Pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print ("Waiting for vehicle to initialise...")
    time.sleep(1)

  while not vehicle.armed:
    print ("Waiting for arming...")
    time.sleep(1)

  print("Waiting for manual arming...")
  while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

  vehicle.mode = VehicleMode("GUIDED")

  print("Vehicle armed.")
  print("Mode: %s" % vehicle.mode.name) 


# Function to arm and then takeoff to a specified altitude
def takeoff(aTargetAltitude):

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print ("Altitude: ", vehicle.location.global_relative_frame.alt)
    #Break and return from function just below target altitude
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
      print ("Reached target altitude")
      print("Height from Lidar: " % vehicle.rangefinder)
      break
    time.sleep(1)


# Function to move forward by a specified distance in meters
def move_forward(distance_meters):
    print(f"Moving forward by {distance_meters} meters")

    # Get current GPS location
    current_location = vehicle.location.global_frame
    lat, lon = current_location.lat, current_location.lon

    # Compute new location using geopy
    new_location = geopy_distance(meters=distance_meters).destination((lat, lon), bearing=0)

    new_lat, new_lon = new_location.latitude, new_location.longitude
    print(f"New Target Location: Lat {new_lat}, Lon {new_lon}")

    # Command the drone to move to the new GPS location
    vehicle.airspeed = 3  # Set speed
    vehicle.simple_goto(LocationGlobalRelative(new_lat, new_lon, current_location.alt))

    # Wait for a short duration to allow movement
    time.sleep(5)
    print("Reached new location")

def loiter(duration):
  print("Switching to Loiter")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("Loiter")
  print("Mode: %s" % vehicle.mode.name)
  print("Height from Lidar: " % vehicle.rangefinder)
  while vehicle.armed:
    time.sleep(duration)


def RTL():
  time.sleep(1)
  print("Returning to Launch")
  vehicle.mode = VehicleMode("RTL")

def Land():
##This function ensures that the vehicle has landed (before vechile.close is called)

  print("Landing")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("LAND")
  print("Mode: %s" % vehicle.mode.name) 
  while vehicle.armed:
    time.sleep(1)
  vehicle.close()


#----begin programming form here


##----------------------------------------------------------------------------------------------------------------->
print("MAIN:  Code Started")

manaul_arm()
print("MAIN:  Manual Arm Success")

takeoff(5) # In meters
print("MAIN:  TakeOff Completed")

loiter(2) # Duration of loiter mode
print("MAIN:  LOITER Completed")

move_forward(5) # Move forward in XX meters
print("MAIN:  GO-TO Completed")

loiter(2) # Duration of loiter mode
RTL()

print("MAIN: IF DRONE IS NOT UPSIDE DOWN, CONGRATS!")