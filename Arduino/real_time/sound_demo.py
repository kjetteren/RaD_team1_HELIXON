from openal import *
import numpy as np
import socket
import struct
import threading

def quaternion_to_euler(q):
    """
    Convert a quaternion into Euler angles (roll, pitch, yaw) in radians
    q: a 4 element array representing the quaternion (w, x, y, z)
    """
    w, x, y, z = q
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = np.arctan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = np.arcsin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = np.arctan2(t3, t4)

    return roll_x, pitch_y, yaw_z

# Initialize OpenAL
device = alcOpenDevice(None)
context = alcCreateContext(device, None)
alcMakeContextCurrent(context)

# Load the audio source
source = oalOpen('./DontYouKnowHowBusyAndImportantIAm_mono.wav')  # Replace with your audio file path

# Set the initial position of the source
source.set_position([1, 0, 0])  # 1 units away on the x-axis

# Get the listener instance
listener = oalGetListener()

# Set the initial position of the listener
listener.set_position([0, 0, 0])

# UDP Configuration
UDP_IP = "0.0.0.0"  # Bind to any IP
UDP_PORT = 12345
BUFFER_SIZE = 72  # 18 floats x 4 bytes

# Sending Socket (not used in this version)
send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
start_message = "0"
send_sock.sendto(start_message.encode(), ("192.168.4.1", UDP_PORT))

# Receiving Socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))
sock.setblocking(False)  # Make socket non-blocking

def receive_data():
    while True:
        try:
            data, addr = sock.recvfrom(BUFFER_SIZE)
        except BlockingIOError:
            # No data received, continue waiting
            pass
        else:
            float_data = struct.unpack('f' * 18, data)
            q_w, q_x, q_y, q_z = float_data[12:16]
            quaternion = [q_w, q_x, q_y, q_z]

            # Convert quaternion to Euler angles and update listener orientation
            roll, pitch, yaw = quaternion_to_euler(quaternion)
            forward = [np.cos(yaw) * np.cos(pitch), np.sin(yaw) * np.cos(pitch), np.sin(pitch)]
            up = [0, 0, 1]  # Assuming up vector is always (0, 0, 1)
            listener.set_orientation(forward + up)

            # Optionally play the sound if not already playing
            if not source.get_state() == AL_PLAYING:
                source.play()

# Start receiving data in a separate thread
thread = threading.Thread(target=receive_data)
thread.start()

# Main loop (no sleep)
try:
    while True:
        # Update logic can be added here if needed
        pass
except KeyboardInterrupt:
    print("Real-time spatial sound stopped.")
finally:
    source.stop()
    alcDestroyContext(context)
    alcCloseDevice(device)
    sock.close()
    send_sock.close()  # Not used in this version