import socket
import struct

# UDP Configuration
UDP_IP = "192.168.4.2" # this device IP
UDP_PORT = 12345
BUFFER_SIZE = 56  # 14 floats x 4 bytes

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

while True:
    data, addr = sock.recvfrom(BUFFER_SIZE)
    print(f"Received data from {addr}")

    float_data = struct.unpack('f' * 14, data)

    print("Accelerometer (x, y, z):", float_data[0:3])
    print("Magnetometer (x, y, z):", float_data[3:6])
    print("Gyroscope (x, y, z):", float_data[6:9])
    print("Gravity (x, y, z):", float_data[9:12])
    print("Pressure:", float_data[12])
    print("Temperature:", float_data[13])