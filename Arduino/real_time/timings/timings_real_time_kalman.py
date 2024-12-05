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

UDP_IP = ""  # Bind to any IP
UDP_PORT = 12345
BUFFER_SIZE = 28  # 7 floats x 4 bytes

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
start_time = time.time()
send_sock.sendto(struct.pack('d', start_time), ("192.168.4.1", UDP_PORT))

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

x_data = []
ukf_data = {i: [] for i in range(4)}

while True:
    data, addr = sock.recvfrom(BUFFER_SIZE)
    float_data = struct.unpack('f' * 7, data)
    w, x, y, z, pressure, temperature, tm = float_data

    q = np.array([w, x, y, z], dtype=np.float32)
    q = q / np.linalg.norm(q)

    ukf.predict()
    ukf.update(q)

    x_data.append(len(x_data))
    for i in range(4):
        ukf_data[i].append(ukf.x[i])

    print(f"Time difference: {time.time() - tm:.3f} seconds")
