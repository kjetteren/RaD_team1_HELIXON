from openal import *
import numpy as np
import socket
import struct
import threading
import time

from Arduino.real_time.qtoe import quaternion_to_euler

# Initialize OpenAL
device = alcOpenDevice(None)
context = alcCreateContext(device, None)
alcMakeContextCurrent(context)

# Load the audio source
source = oalOpen('./336598__inspectorj__footsteps-concrete-a.wav')  # Replace with your audio file path

# Set the initial position of the source
source.set_position([1, 0, 0])  # 1 unit away on the x-axis

# Get the listener instance
listener = oalGetListener()

# Set the initial position of the listener
listener.set_position([0, 0, 0])

# UDP Configuration
UDP_IP = "0.0.0.0"  # Bind to any IP
UDP_PORT = 12345
BUFFER_SIZE = 72  # 18 floats x 4 bytes

# Sending Socket
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
start_message = "0"
send_sock.sendto(start_message.encode(), ("192.168.4.1", UDP_PORT))

# Receiving Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

def receive_data():
    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        float_data = struct.unpack('f' * 18, data)
        q_w, q_x, q_y, q_z = float_data[12:16]
        quaternion = [q_w, q_x, q_y, q_z]

        # Convert quaternion to Euler angles
        roll, pitch, yaw = quaternion_to_euler(quaternion)

        # Update listener orientation
        listener.set_orientation([np.cos(yaw), np.sin(yaw), 0, 0, 0, 1])

        # Play the sound if not already playing
        if not source.get_state() == AL_PLAYING:
            source.play()

# Start receiving data in a separate thread
thread = threading.Thread(target=receive_data)
thread.start()

# Keep the main thread alive
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Real-time spatial sound stopped.")
finally:
    source.stop()
    alcDestroyContext(context)
    alcCloseDevice(device)
    sock.close()
    send_sock.close()
    oalQuit()