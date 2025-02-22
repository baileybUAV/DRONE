# Print the message "initiating connection" on the screen.
print ("initiating connection")

# The port to which the FCC connects to the rpi is our connection address. 
connection_string = "serial0"

# import the DroneKit-Python library.
from dronekit import connect, VehicleMode

# Connect the vehicle
print("The vehicle is connecting at this address: %s" % (connection_string,))
# Connect to the vehicle with the specified connection information and wait for the connection to complete.
vehicle = connect(connection_string, wait_ready=True)
print("Connected")

# Get some vehicle properties (status information) and print them on the screen.
print  ("Vehicle Status:")
print (" GPS: %s" % vehicle.gps_0)
print (" Battery: %s" % vehicle.battery)
print (" ARMable?: %s" % vehicle.is_armable)
print (" Mode: %s" % vehicle.mode.name)    
# Disconnect from the simulation before exiting the programme.
vehicle.close()


# Print the message "Completed" on the screen.
print("Completed")