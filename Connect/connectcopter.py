from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import math
import argparse
import os

def connectMyCopter():
  parser = argparse.ArgumentParser(description='commands')
  parser.add_argument('--connect')
  args = parser.parse_args()

  connection_string = args.connect
  baud_rate = 57600

  print("Connecting...")
  vehicle = connect(connection_string,baud=baud_rate) #,wait_ready=True)
  print  ("Vehicle Status:")
  print (" GPS: %s" % vehicle.gps_0)
  print (" Battery: %s" % vehicle.battery)
  print (" ARMable?: %s" % vehicle.is_armable)
  print (" Mode: %s" % vehicle.mode.name)    

  return vehicle