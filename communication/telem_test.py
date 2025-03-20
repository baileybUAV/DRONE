

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
    # Retrieve UAV GPS location
    lat = vehicle.location.global_frame.lat
    lon = vehicle.location.global_frame.lon
    alt = vehicle.location.global_frame.alt
    velocity_north = vehicle.velocity[0]  # North velocity (m/s)
    velocity_east = vehicle.velocity[1]   # East velocity (m/s)
    velocity_down = vehicle.velocity[2]   # Down velocity (m/s)

    if lat is None or lon is None:
        print("Error: No valid GPS data available")
    else:
        # Dummy covariance matrix (adjust based on real sensor accuracy)
        position_covariance = [0.01] * 21  # 21 elements needed for the matrix
        
        # Create MAVLink message with GPS and velocity data
        msg = telem_link.mav.global_position_int_cov_encode(
            int(time.time() * 1e6),  # Time since boot (microseconds)
            int(lat * 1e7),  # Latitude in degrees * 1E7
            int(lon * 1e7),  # Longitude in degrees * 1E7
            int(alt * 1000),  # Altitude in mm (above sea level)
            int(vehicle.location.global_relative_frame.alt * 1000),  # Altitude relative to home (mm)
            velocity_north * 100,  # North velocity (cm/s)
            velocity_east * 100,   # East velocity (cm/s)
            velocity_down * 100,   # Down velocity (cm/s)
            int(vehicle.heading * 100),  # Heading (centidegrees)
            position_covariance,  # Position covariance (21 elements)
            3  # Estimator type (3 = GPS, adjust as needed)
        )

        # Send the message to the other Pi
        telem_link.mav.send(msg)
        print(f"Sent GLOBAL_POSITION_INT_COV Data: Lat {lat}, Lon {lon}, Alt {alt}, VelN {velocity_north}, VelE {velocity_east}, VelD {velocity_down}")
        send_mavlink_message("Location & velocity sent via GLOBAL_POSITION_INT_COV")
        print("Location & Velocity Sent\n")    

    time.sleep(2)