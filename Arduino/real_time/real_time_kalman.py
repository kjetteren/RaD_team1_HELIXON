import numpy as np
from filterpy.kalman import UnscentedKalmanFilter as UKF
from filterpy.kalman import MerweScaledSigmaPoints
import socket
import struct

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

dim_x = 4
dim_z = 4
points = MerweScaledSigmaPoints(n=dim_x, alpha=0.1, beta=2.0, kappa=0.0)
ukf = UKF(dim_x=dim_x, dim_z=dim_z, fx=lambda x, dt: x, hx=lambda x: x, dt=1/25, points=points)
ukf.x = np.array([1, 0, 0, 0])  # Initial state
ukf.P *= 0.1  # Initial covariance

plt.ion()
fig, ax = plt.subplots()
lines = [
    ax.plot([], [], label='Quaternion_W')[0],
    ax.plot([], [], label='Quaternion_X')[0],
    ax.plot([], [], label='Quaternion_Y')[0],
    ax.plot([], [], label='Quaternion_Z')[0],
]
ax.legend()
ax.set_ylim(-1, 1)

# UDP Configuration
UDP_IP = "" # any IP bind
UDP_PORT = 12345
BUFFER_SIZE = 72  # 18 floats x 4 bytes

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
start_message = "0"
send_sock.sendto(start_message.encode(), ("192.168.4.1", UDP_PORT))

# Receiving Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

x_data = []
ukf_data = {i: [] for i in range(4)}

first = True

while True:
    data, addr = sock.recvfrom(BUFFER_SIZE)
    float_data = struct.unpack('f' * 18, data)
    a_x, a_y, a_z, m_x, m_y, m_z, gy_x, gy_y, gy_z, gr_x, gr_y, gr_z, q_w, q_x, q_y, q_z, pressure, temperature = float_data
    q = np.array([q_w, q_x, q_y, q_z], np.float32)
    norm = np.linalg.norm(q)
    if norm != 0:
        q = q / np.linalg.norm(q)
    else:
        q = q / 1e-10
    if first:
        ukf.x = q
        first = False
    else:
        ukf.predict()
        ukf.update(q)

    x_data.append(len(x_data))
    for i in range(4):
        ukf_data[i].append(ukf.x[i])
    for i, line in enumerate(lines):
        line.set_data(x_data, ukf_data[i])

    ax.set_xlim(max(0, len(x_data) - 50), len(x_data))  # Keep the last 50 data points visible

    plt.pause(0.01)