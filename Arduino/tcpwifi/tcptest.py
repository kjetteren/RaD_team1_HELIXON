import socket
import time

SERVER_IP = "192.168.4.1"  # Replace with your Arduino's IP address
SERVER_PORT = 12345
PACKET_COUNT = 255
DELAY = 0.05  # Delay between packets (in seconds)

# Connect to the server
client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
client.connect((SERVER_IP, SERVER_PORT))

print(f"Sending {PACKET_COUNT} packets to {SERVER_IP}:{SERVER_PORT}")
for i in range(PACKET_COUNT):
    client.sendall(bytes([i]))
    time.sleep(DELAY)

# Wait for packet loss status (0 or 1)
client.settimeout(5)  # Set a timeout to wait for the response
try:
    response = client.recv(1)  # Receive the status (1 byte)
    if response == b'0':
        print("No packet loss detected.")
    elif response == b'1':
        print("Packet loss detected.")
    else:
        print("Unexpected response.")
except socket.timeout:
    print("Timeout: No response from server.")
finally:
    client.close()
    print("Test complete.")