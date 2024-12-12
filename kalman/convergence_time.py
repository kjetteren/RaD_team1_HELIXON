import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import math

def normalize_quaternion(q):
    norm = np.linalg.norm(q)
    if norm > 1e-10:
       q = q / norm
    return q

def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


measured_roll = []
measured_pitch = []
measured_yaw = []

df = pd.read_csv('spiral_data/sensor_data_up_b.csv')
v=np.array([0.0,0,0])
for i, row in df.iterrows():
    z = [row['Gyro_X']*(np.pi/180),
         row['Gyro_Y']*(np.pi/180),
         row['Gyro_Z']*(np.pi/180),
         row['Quaternion_W'], 
         row['Quaternion_X'], 
         row['Quaternion_Y'], 
         row['Quaternion_Z']]
    
    z[3:7] = normalize_quaternion(z[3:7])
    roll, pitch, yaw = euler_from_quaternion(z[3], z[4], z[5], z[6])
    measured_roll.append(roll)
    measured_pitch.append(pitch)
    measured_yaw.append(yaw)