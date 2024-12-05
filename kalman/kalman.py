import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter, ExtendedKalmanFilter
import quaternion
from mpl_toolkits.mplot3d import Axes3D

def phi(x: np.array, Ts: float, tau1: float , tau2: float, tau3: float) -> np.array:
    return np.array([[np.exp(-Ts/tau1), 0, 0, 0, 0, 0, 0],
                    [0, np.exp(-Ts/tau2), 0, 0, 0, 0, 0],
                    [0, 0, np.exp(-Ts/tau3), 0, 0, 0, 0],
                    [-(x[4]*Ts)/2, -(x[5]*Ts)/2, -(x[6]*Ts)/2, 1, -(x[0]*Ts)/2, -(x[1]*Ts)/2, -(x[2]*Ts)/2],
                    [(x[3]*Ts)/2, -(x[6]*Ts)/2, (x[5]*Ts)/2, (x[0]*Ts)/2, 1, (x[2]*Ts)/2, -(x[1]*Ts)/2],
                    [(x[6]*Ts)/2, (x[3]*Ts)/2, -(x[4]*Ts)/2, (x[1]*Ts)/2, -(x[2]*Ts)/2, 1, (x[0]*Ts)/2],
                    [-(x[5]*Ts)/2, (x[4]*Ts)/2, (x[3]*Ts)/2, (x[2]*Ts)/2, (x[1]*Ts)/2, -(x[0]*Ts)/2, 1]])

def quaternion_to_matrix(q: np.array) -> np.array:
    w, x, y, z = q
    return np.array([
        [1 - 2 * y ** 2 - 2 * z ** 2, 2 * x * y - 2 * z * w, 2 * x * z + 2 * y * w],
        [2 * x * y + 2 * z * w, 1 - 2 * x ** 2 - 2 * z ** 2, 2 * y * z - 2 * x * w],
        [2 * x * z - 2 * y * w, 2 * y * z + 2 * x * w, 1 - 2 * x ** 2 - 2 * y ** 2]
    ])

def normalize_state_quaternion(kf: KalmanFilter):
    # get quaternion part (indices 3-6)
    q = kf.x[3:7]
    norm = np.linalg.norm(q)
    if norm > 1e-10:
        kf.x[3:7] = q / norm
        
def update_velocity(v: np.array, accel: np.array, R: np.array, Ts: float) -> np.array:
    # remove gravity from accelerometer readings
    gravity = np.array([0, 0, 9.81])  # gravity in world frame
    # transform to world frame and remove gravity
    accel_world = R @ accel - gravity
    # integrate acceleration to velocity
    v_new = v + accel_world * Ts
    return v_new    

def plot_predicted_positions(positions):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot measured position
    ax.plot(positions[:, 0], positions[:, 1], positions[:, 2], label='Trajectory', color='blue')

    # Plot predicted positions
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    plt.legend()
    plt.show()

def plot_positions_over_time(positions):
    plt.figure(figsize=(12, 8))
    plt.plot(positions[:, 0], label='X Position over Time')
    plt.plot(positions[:, 1], label='Y Position over Time')
    plt.plot(positions[:, 2], label='Z Position over Time')
    plt.title('Position Over Time Derived from UKF Quaternions')
    plt.xlabel('Time Step')
    plt.ylabel('Position')
    plt.legend()
    plt.show()

def plot_predicted_quaternions(predicted_quaternions):
    plt.figure(figsize=(10, 6))
    plt.plot(predicted_quaternions[:, 0], label='Quaternion_W')
    plt.plot(predicted_quaternions[:, 1], label='Quaternion_X')
    plt.plot(predicted_quaternions[:, 2], label='Quaternion_Y')
    plt.plot(predicted_quaternions[:, 3], label='Quaternion_Z')
    plt.xlabel('Time Step')
    plt.ylabel('Quaternion Component')
    plt.title('Predicted Quaternion Components Over Time')
    plt.legend()
    plt.grid()
    plt.show()
