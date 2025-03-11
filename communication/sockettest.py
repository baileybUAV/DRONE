import socket

client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
client.connect(('10.42.0.1', 9999))
print("Sending Message")
client.sendto('Hello from Drone/UAV'.encode(), ('10.42.0.22', 9999))
print("Message Sent")
data, addr = client.recvfrom(1024)

print(data.decode())

exit()
