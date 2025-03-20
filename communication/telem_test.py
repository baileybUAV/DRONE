

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
import numpy as np

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

def send_mavlink_message(text):
    """ Sends a MAVLink STATUSTEXT message to Mission Planner. """
    msg = vehicle.message_factory.statustext_encode(
        mavutil.mavlink.MAV_SEVERITY_INFO,  # Severity Level (INFO, WARNING, ERROR, etc.)
        text.encode()  # Message (must be encoded as bytes)
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()


##MAIN CODE##

telem_link = setup_telem_connection()
print("Press ENTER to start sending data")
input()
print("Sending data now")

while True:
    # Get UAV GPS and velocity data
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt
    rel_alt = vehicle.location.global_relative_frame.alt
    velocity_north = vehicle.velocity[0]  # North velocity (m/s)
    velocity_east = vehicle.velocity[1]   # East velocity (m/s)
    velocity_down = vehicle.velocity[2]   # Down velocity (m/s)

    if lat is None or lon is None:
        print("Error: No valid GPS data available")
    else:
        # Create a covariance matrix with NaN (unknown values)
        covariance_matrix = np.full((36,), float('nan'), dtype=np.float32)

        print("GPS STATUS: %s" % vehicle.gps_0.fix_type)
        # Create MAVLink message with GPS and velocity data
        msg = telem_link.mav.global_position_int_cov_encode(
            int(time.time() * 1e6),  # Timestamp (microseconds)
            3,  # MAV_ESTIMATOR_TYPE_GPS (3 = GPS-based estimation)
            int(lat * 1e7),  # Latitude in degrees * 1E7
            int(lon * 1e7),  # Longitude in degrees * 1E7
            int(alt * 1000),  # Altitude above MSL in mm
            int(rel_alt * 1000),  # Relative altitude in mm
            float(velocity_north),  # Velocity North (m/s)
            float(velocity_east),   # Velocity East (m/s)
            float(velocity_down),   # Velocity Down (m/s)
            covariance_matrix  # 6x6 Covariance matrix
        )


        # Send the message to the other Pi
        telem_link.mav.send(msg)
        print(f"Sent GLOBAL_POSITION_INT_COV Data: Lat {lat}, Lon {lon}, Alt {alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")
        send_mavlink_message("Location & velocity sent via GLOBAL_POSITION_INT_COV")
        print("Location & Velocity Sent\n")    

    time.sleep(2)