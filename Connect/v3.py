from dronekit import connect, VehicleMode, LocationGlobalRelative, APIException
import time
import socket
import sys
import math
import argparse

def connectMyCopter():
    parser = argparse.ArgumentParser(description='commands)')
    parser.add_argument('--connect')
    args = parser.parse_args()
    connection_string = args.connect
    baud_rate = 57600

    vehicle = connect(connection_string,baud=baud_rate,wait_ready=true)
    return vehicle

vehicle = connectMyCopter()

print('connected')
