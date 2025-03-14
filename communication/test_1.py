
#Sends UGV the UAV's current GPS Location every 2s over hotspot and socket


from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
from geopy.distance import distance as geopy_distance
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

  print("Connecting...")
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

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UGV_IP = "10.42.0.120"
PORT = 5005

def send_gps():
    while True:
         
        
        lat = vehicle.location.global_frame.lat
        lon = vehicle.location.global_frame.lon
        alt = vehicle.location.global_relative_frame.alt
        
        if lat != 0:
          # Create message
          data = f"{lat},{lon},{alt}"
          sock.sendto(data.encode(), (UGV_IP, PORT))
          print(f"Sent GPS to UGV: {data}")

          time.sleep(2)  # Send every 2 seconds
        else:
           print("Invlaid GPS Location")
           time.sleep(2)

send_gps()