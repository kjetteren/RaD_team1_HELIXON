import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF, MerweScaledSigmaPoints
import socket
import struct
import time

dim_x = 4
dim_z = 4
points = MerweScaledSigmaPoints(n=dim_x, alpha=0.1, beta=2.0, kappa=0.0)
ukf = UKF(dim_x=dim_x, dim_z=dim_z, fx=lambda x, dt: x, hx=lambda x: x, dt=1/25, points=points)
ukf.x = np.array([1, 0, 0, 0])
ukf.P *= 0.1

# UDP Configuration
UDP_IP = ""  # Bind to any IP
UDP_PORT = 12345
BUFFER_SIZE = 28  # 7 floats x 4 bytes

# Send synchronization start time
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
pc_start_time = float(time.time())  # Ensure it's a float
send_sock.sendto(struct.pack('f', pc_start_time), ("192.168.4.1", UDP_PORT))

# Receive data
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

filtered_count = 0
start_time = time.time()

# Main loop
while True:
    data, addr = sock.recvfrom(BUFFER_SIZE)
    float_data = struct.unpack('f' * 7, data)
    w, x, y, z, pressure, temperature, arduino_time = float_data

    q = np.array([w, x, y, z], dtype=np.float32)
    q = q / np.linalg.norm(q)

    ukf.predict()
    ukf.update(q)

    filtered_count += 1

    current_time = time.time()
    elapsed_time = current_time - start_time

    if elapsed_time >= 1.0:  # If 1 second has passed
        print(f"Data points filtered last second: {filtered_count}")
        start_time = current_time
        filtered_count = 0
