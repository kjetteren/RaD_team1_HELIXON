import socket
import time

# set up UDP connection parameters
arduino_ip = "192.168.4.1"   # Arduino's IP address
port = 12345                 # port Arduino is listening on
packet_count = 100           # number of packets to send

# create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("Sending packets...")
for i in range(packet_count):
    message = f"Packet {i+1}"
    sock.sendto(message.encode(), (arduino_ip, port))
    print(f"Sent {message}")
    time.sleep(0.1) # delay between packets

sock.close()
print("All packets sent.")