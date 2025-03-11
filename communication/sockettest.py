import socket

client = socket.socket(socket.AF_NET, socket.SOCK_STREAM)
client.connect(('10.42.0.1', 9999))

client.send('Hello from Drone/UAV'.encode())
print(client.recv(1024).decode())