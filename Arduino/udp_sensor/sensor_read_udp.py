import csv
import socket
import struct

# UDP Configuration
UDP_IP = "" # any IP bind
UDP_PORT = 12345
BUFFER_SIZE = 72  # 18 floats x 4 bytes

with open("sensor_data.csv", mode="w", newline="") as file:
    csv_writer = csv.writer(file)

    # Write the header row (this will be written every time the file is created)
    csv_writer.writerow([
        "Accel_X", "Accel_Y", "Accel_Z",
        "Magneto_X", "Magneto_Y", "Magneto_Z",
        "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Gravity_X", "Gravity_Y", "Gravity_Z",
        "Quaternion_W", "Quaternion_X", "Quaternion_Y", "Quaternion_Z",
        "Pressure", "Temperature"
    ])

    # Sending Socket
    send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    start_message = "0"
    send_sock.sendto(start_message.encode(), ("192.168.4.1", UDP_PORT))

    # Receiving Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))

    while True:
        data, addr = sock.recvfrom(BUFFER_SIZE)
        float_data = struct.unpack('f' * 18, data)
        csv_writer.writerow(float_data)