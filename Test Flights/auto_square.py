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
  print("Rangefinder distance: %s" % vehicle.rangefinder.distance)
  print("Global Location: %s" % vehicle.location.global_frame)
  print("Global Location (relative altitude): %s" % vehicle.location.global_relative_frame)
  print("Local Location: %s" % vehicle.location.local_frame)
  print("Mode: %s" % vehicle.mode.name)     
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






def RTL():
  time.sleep(1)
  print("   Returning to Launch")
  vehicle.mode = VehicleMode("RTL")
  print("   Mode: %s" % vehicle.mode.name) 
  while vehicle.armed:
    time.sleep(1)
  vehicle.close()

def Land():
##This function ensures that the vehicle has landed (before vechile.close is called)

  print("   Landing")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("LAND")
  print("   Mode: %s" % vehicle.mode.name) 
  while vehicle.armed:
    time.sleep(1)
  vehicle.close()


#----begin programming form here


##----------------------------------------------------------------------------------------------------------------->
print("MAIN:  Code Started")


manaul_arm()
print("MAIN:  Manual Arm Success")

takeoff(3) # In meters
print("MAIN:  TakeOff Completed")


print("Set default/target airspeed to 3")
vehicle.airspeed = 3

print("Going towards first point   ...")
#CORRECT COORDINATES
point1 = LocationGlobalRelative(27.9873411, -82.3012447, 4)
vehicle.simple_goto(point1)
time.sleep(20)
print("Reached Point")

print("Going towards second point  ...")
#CORRECT COORDINATES
point2 = LocationGlobalRelative(27.9871291, -82.3012541, 4)
vehicle.simple_goto(point2)
time.sleep(20)
print("Reached Point")

print("Going towards third point  ...")
#CORRECT COORDINATES
point3 = LocationGlobalRelative(27.9871243, -82.3016618, 4)
vehicle.simple_goto(point3)
time.sleep(20)
print("Reached Point")

print("Going towards fourth point ...")
#CORRECT COORDINATES
point4 = LocationGlobalRelative(27.9873340, -82.3016605, 4)
vehicle.simple_goto(point4)
time.sleep(20)
print("Reached Point")

print("Going towards first point  ...")
#CORRECT COORDINATES
point1 = LocationGlobalRelative(27.9873411, -82.3012447, 4)
vehicle.simple_goto(point1)
time.sleep(20)
print("Reached Point")

print("Landing")
Land()

print("MAIN: IF DRONE IS NOT UPSIDE DOWN, CONGRATS!")
exit()