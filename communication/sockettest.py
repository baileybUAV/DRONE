import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UGV_IP = "10.42.0.1"
PORT = 5005

sock.sendto(b"Hello from UAV", (UGV_IP, PORT))
print("Message sent")
