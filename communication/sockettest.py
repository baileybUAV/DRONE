import socket

client = socket.socket(socket.AF_NET, socket.SOCK_STREAM)
client.connect(('172.28.11.51', 9999))

client.send('Hello from Drone/UAV'.encode())
print(client.recv(1024).decode())