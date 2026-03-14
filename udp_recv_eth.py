import socket
import json

UDP_IP = "10.42.0.2"
UDP_PORT = 5105

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening on {UDP_IP}:{UDP_PORT}")

while True:
    data, addr = sock.recvfrom(4096)

    try:
        msg = json.loads(data.decode())
    except:
        msg = data.decode()

    print(f"From {addr}: {msg}")
