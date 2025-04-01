# Simple hover test with LIDAR-based altitude check and image capture

from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
from pymavlink import mavutil
import time
import future
import socket
import math
import argparse
import os
import threading
from picamera2 import Picamera2
import cv2
import numpy as np

# ------------------ CONFIG ------------------
photo_save_path = "/home/uav/drone/photos"
camera_resolution = (1280, 720)
target_altitude = 6  # in meters

# ------------------ CAMERA SETUP ------------------
picam2 = Picamera2()
config = picam2.create_preview_configuration(main={"size": camera_resolution, "format": "RGB888"})
picam2.configure(config)
picam2.start()

def capture_photo():
    img = picam2.capture_array()
    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    timestamp = time.strftime("%Y%m%d_%H%M%S")
    photo_path = f"hover_photo_{timestamp}.jpg"
    cv2.imwrite(photo_path, img)
    print(f"Photo captured and saved at: {photo_path}")


# ------------------ DRONE CONNECT ------------------
def connectMyCopter():
    print("Start Connection")
    parser = argparse.ArgumentParser(description='commands')
    parser.add_argument('--connect')
    args = parser.parse_args()

    connection_string = "/dev/ttyAMA0"
    baud_rate = 57600

    print("Connecting Pi to Drone...")
    vehicle = connect(connection_string, baud=baud_rate)
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

# ------------------ ARMING ------------------
def manual_arm():
    print("Pre-arm checks")
    while not vehicle.is_armable:
        print("Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    print("Waiting for manual arming...")
    while not vehicle.armed:
        print("Waiting for arming...")
        time.sleep(1)

    vehicle.mode = VehicleMode("GUIDED")
    print("Vehicle armed.")
    print("Mode: %s" % vehicle.mode.name)

# ------------------ TAKEOFF ------------------
def takeoff(aTargetAltitude):
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)

    photo_taken = False

    while True:
        alt = vehicle.location.global_relative_frame.alt
        lidar = vehicle.rangefinder.distance
        print(f"Altitude (GPS): {alt:.2f}, LIDAR: {lidar:.2f}")

        if lidar >= aTargetAltitude * 0.90:
            print("Reached target altitude")

            if not photo_taken:
                print("Taking photo at target altitude...")
                capture_photo()
                photo_taken = True

            break

        time.sleep(1)

# ------------------ LAND ------------------
def land():
    print("Landing...")
    vehicle.mode = VehicleMode("LAND")
    print("Mode: %s" % vehicle.mode.name)
    while vehicle.armed:
        time.sleep(1)
    vehicle.close()

# ------------------ MAIN ------------------
print("MAIN: Code Started")


manual_arm()
print("MAIN: Manual Arm Success")

takeoff(target_altitude)
print("MAIN: Takeoff Completed")

vehicle.mode = VehicleMode("LOITER")
print("MAIN: Loiter Mode for 15 seconds")
time.sleep(15)

land()
print("MAIN: IF DRONE IS NOT UPSIDE DOWN, CONGRATS!")

picam2.stop()
exit()
