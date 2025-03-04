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

  print("Connecting...")
  vehicle = connect(connection_string,baud=baud_rate) 
  print("GPS: %s" % vehicle.gps_0)
  print("Battery: %s" % vehicle.battery)
  print("Armable?: %s" % vehicle.is_armable)
  print("Height from Lidar: " % vehicle.rangefinder1)
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
    #Break and return from function just below target altitude
    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
      print ("Reached target altitude")
      
      break
    time.sleep(1)

def loiter(duration):
  print("Switching to Loiter")
  ##thread_distance.join()
  time.sleep(1)
  vehicle.mode = VehicleMode("LOITER")
  print("Mode: %s" % vehicle.mode.name)
  while vehicle.armed:
    time.sleep(duration)

# Function to check for user input to switch mode
def check_for_switch():
    global running
    while running:
        user_input = input().strip().lower()  # Listens for keyboard input
        if user_input == 's':
            print("\n[EMERGENCY] Switching to STABILIZE mode! Manual Control Enabled.")
            vehicle.mode = VehicleMode("ALT HOLD")
            running = False  # Stop the mission loop

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
# Start a separate thread to listen for emergency switch input
running = True
threading.Thread(target=check_for_switch, daemon=True).start()


manaul_arm()
print("MAIN:  Manual Arm Success")
takeoff(1) # In meters
print("MAIN:  TakeOff Completed")
print("\nPress 's' at any time to switch to STABILIZE mode and take manual control.")
loiter(10) # Duration of loiter mode
print("MAIN:  LOITER Completed")
Land()
print("MAIN: IF DRONE IS NOT UPSIDE DOWN, CONGRATS!")