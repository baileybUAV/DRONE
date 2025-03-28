
#This code is a simple hover test
#Changed to use lidar as alt, still must test

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import future
import socket
import math
import argparse
import os
import threading


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
    print ("Lidar: ", vehicle.rangefinder.distance)
    #Break and return from function just below target altitude
    if vehicle.rangefinder.distance>=aTargetAltitude*0.90:
      print ("Reached target altitude")
      
      break
    time.sleep(1)




def Land():
##This function ensures that the vehicle has landed (before vechile.close is called)

  print("Landing")
  
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

takeoff(3) # In meters
print("MAIN:  TakeOff Completed")
vehicle.mode = VehicleMode("RTL")
print("MAIN:  RTL Mode")
print("MAIN: IF DRONE IS NOT UPSIDE DOWN, CONGRATS!")
vehicle.close()
exit()