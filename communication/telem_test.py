

#Sends UGV the UAV's current GPS Location every 2s over hotspot and socket


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import future
import socket
import math
import argparse
import os
import socket

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

# Function to establish telemetry connection between Raspberry Pis

def setup_telem_connection():
    telem_port = "/dev/ttyUSB0"  # USB telemetry module
    baud_rate = 57600  # Ensure the correct baud rate
    
    print("Connecting to telemetry module for Pi-to-Pi communication...")
    telem_link = mavutil.mavlink_connection(telem_port, baud=baud_rate)
    print("Telemetry link established!")
    return telem_link


telem_link = setup_telem_connection()

while True:
    # Retrieve UAV GPS location
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt
    
    # Create a MAVLink message with GPS coordinates
    msg = telem_link.mav.gps_raw_int_encode(
        int(time.time() * 1e6),  # Timestamp (microseconds)
        3,  # Fix type (3D fix)
        int(lat * 1e7),  # Latitude in 1E7 degrees
        int(lon * 1e7),  # Longitude in 1E7 degrees
        int(alt * 1000),  # Altitude in mm
        int(vehicle.gps_0.eph),  # HDOP (horizontal accuracy in cm)
        int(vehicle.gps_0.epv),  # VDOP (vertical accuracy in cm)
        int(vehicle.groundspeed * 100),  # Velocity in cm/s
        int(vehicle.heading * 100),  # Course over ground (degrees * 100)
        vehicle.gps_0.satellites_visible  # Number of satellites visible
    )
    
    # Send the message to the other Pi
    telem_link.mav.send(msg)
    print(f"Sent GPS Data: Lat {lat}, Lon {lon}, Alt {alt}")
    
    time.sleep(2)
