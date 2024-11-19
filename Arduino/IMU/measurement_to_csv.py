import serial
import csv
import time
from datetime import datetime

ser = serial.Serial('COM7', 115200, timeout=1)  # Adjust 'COM7' as needed
time.sleep(2)  

try:
    with open('sensor_data.csv', 'w', newline='') as csvfile:
        csv_writer = csv.writer(csvfile)
        
        # Updated headers to match the Arduino output
        headers = [
            'a_x', 'a_y', 'a_z',           # Accelerometer
            'm_x', 'm_y', 'm_z',           # Magnetometer
            'omega_x', 'omega_y', 'omega_z', # Gyroscope
            'g_x', 'g_y', 'g_z',           # Gravity vector
            'Pressure', 'Temperature',     # BMP sensor data
            'Timestamp'                    # Timestamp for each row
        ]
        csv_writer.writerow(headers)

        while True:  # Continuous loop until interrupted
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8').strip()
                print(f"Raw data: {line}")  # For debugging, shows the raw serial data
                try:
                    # Split line into components
                    data = line.split(',')
                    # Ensure we have exactly 14 data points
                    if len(data) == 14:
                        # Convert data to floats
                        (
                            a_x, a_y, a_z,        # Accelerometer data
                            m_x, m_y, m_z,        # Magnetometer data
                            omega_x, omega_y, omega_z, # Gyroscope data
                            g_x, g_y, g_z,        # Gravity vector
                            pressure, temperature # BMP sensor data
                        ) = map(float, data)
                        
                        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
                        
                        # Write data with timestamp to CSV
                        csv_writer.writerow([
                            a_x, a_y, a_z,
                            m_x, m_y, m_z,
                            omega_x, omega_y, omega_z,
                            g_x, g_y, g_z,
                            pressure, temperature,
                            timestamp
                        ])
                        count += 1
                        print(f"Row {count} written: {[a_x, a_y, a_z, m_x, m_y, m_z, omega_x, omega_y, omega_z, g_x, g_y, g_z, pressure, temperature, timestamp]}")  # Debug confirmation
                except ValueError:
                    print("Error: Non-numeric data received")
except KeyboardInterrupt:
    print("Data collection interrupted.")
finally:
    ser.close()  
    print("Data collection complete. Saved to 'sensor_data.csv'.")
