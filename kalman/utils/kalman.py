import numpy as np
import math
import pandas as pd
import matplotlib.pyplot as plt
from filterpy.kalman import KalmanFilter, ExtendedKalmanFilter
from mpl_toolkits.mplot3d import Axes3D

def phi(x: np.array, Ts: float, tau1: float , tau2: float, tau3: float) -> np.array:
    return np.array([[np.exp(-Ts/tau1), 0, 0, 0, 0, 0, 0],
                    [0, np.exp(-Ts/tau2), 0, 0, 0, 0, 0],
                    [0, 0, np.exp(-Ts/tau3), 0, 0, 0, 0],
                    [-(x[4]*Ts)/2, -(x[5]*Ts)/2, -(x[6]*Ts)/2, 1, -(x[0]*Ts)/2, -(x[1]*Ts)/2, -(x[2]*Ts)/2],
                    [(x[3]*Ts)/2, -(x[6]*Ts)/2, (x[5]*Ts)/2, (x[0]*Ts)/2, 1, (x[2]*Ts)/2, -(x[1]*Ts)/2],
                    [(x[6]*Ts)/2, (x[3]*Ts)/2, -(x[4]*Ts)/2, (x[1]*Ts)/2, -(x[2]*Ts)/2, 1, (x[0]*Ts)/2],
                    [-(x[5]*Ts)/2, (x[4]*Ts)/2, (x[3]*Ts)/2, (x[2]*Ts)/2, (x[1]*Ts)/2, -(x[0]*Ts)/2, 1]])
        
def update_velocity(v: np.array, accel: np.array, R: np.array, Ts: float) -> np.array:
    # remove gravity from accelerometer readings
    gravity = np.array([0, 0, 9.81])  # gravity in world frame
    # transform to world frame and remove gravity
    accel_world = R @ accel - gravity
    # integrate acceleration to velocity
    v_new = v + accel_world * Ts
    return v_new

def calculate_altitude(pressure, P0, T0, g, R_const):
    T0_kelvin = T0 + 273.15  # Convert temperature to Kelvin
    return (T0_kelvin / g) * np.log(P0 / pressure)  # Calculate altitude

def fx(x, dt, accel, g):
    pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, qx, qy, qz, qw, ang_vel_x, ang_vel_y, ang_vel_z = x
    q = np.array([qw, qx, qy, qz])
    ang_vel = np.array([ang_vel_x, ang_vel_y, ang_vel_z])
    R = quaternion_to_rotation_matrix(q)
    acc_body = np.array([accel[0], accel[1], accel[2] - g])
    acc_world = R @ acc_body

    new_pos = np.array([pos_x, pos_y, pos_z]) + acc_world * dt ** 2 * 0.5 + np.array([vel_x, vel_y, vel_z]) * dt
    new_vel = np.array([vel_x, vel_y, vel_z]) + acc_world * dt
    new_qx, new_qy, new_qz, new_qw = integrate_quaternion(q, ang_vel, dt)
    return np.concatenate((new_pos, new_vel, [new_qx, new_qy, new_qz, new_qw], [ang_vel_x, ang_vel_y, ang_vel_z]))

def calculate_altitude(pressure, P0, T0, g, R_const):
    T0_kelvin = T0 + 273.15  # Convert temperature to Kelvin
    return (T0_kelvin / g) * np.log(P0 / pressure)  # Calculate altitude

def hx(x):
    return x[:3]  # We only measure the position

def verify_data_structure(gyr, acc):
    print("Data Structure Information:")
    print(f"Gyroscope data shape: {gyr.shape}")
    print(f"Accelerometer data shape: {acc.shape}")
    print(f"Gyroscope data sample: {gyr[0]}")
    print(f"Accelerometer data sample: {acc[0]}")

    # Verify data types
    print(f"\nData Types:")
    print(f"Gyroscope data type: {gyr.dtype}")
    print(f"Accelerometer data type: {acc.dtype}")

    # Verify data ranges
    print(f"\nData Ranges:")
    print(f"Gyroscope range: [{np.min(gyr)}, {np.max(gyr)}]")
    print(f"Accelerometer range: [{np.min(acc)}, {np.max(acc)}]")

def quaternion_to_euler_angles(q):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    Quaternion format assumed as [w, x, y, z].
    """
    w, x, y, z = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.pi / 2 * np.sign(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

class QuaternionEKF:
    def __init__(self):
        # state vector: [p, q, r, a, b, c, d]
        # p,q,r are angular rates
        # a,b,c,d are quaternion components
        self.state = np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])

        # state covariance
        self.P = np.eye(7) * 10

        # process noise
        self.Q = np.eye(7)
        self.Q[:3, :3] *= 0.15  # angular rates noise
        self.Q[3:, 3:] *= 0.4  # quaternion noise

        # measurement noise
        self.R = np.eye(7) * 0.01

        # system parameters
        self.dt = 0.04  # sample time
        self.tau = 0.03  # Time constant for angular rates

    def quaternion_multiply(self, q1, q2):
        """Multiply two quaternions"""
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

    def quaternion_to_rotation_matrix(self, q):
        """
        Convert quaternion to rotation matrix
        q = [q0, q1, q2, q3] = [w, x, y, z]
        """
        # Extract quaternion components
        q0, q1, q2, q3 = q

        # Compute rotation matrix elements
        R = np.array([
            [1 - 2 * q2 ** 2 - 2 * q3 ** 2, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
            [2 * (q1 * q2 + q0 * q3), 1 - 2 * q1 ** 2 - 2 * q3 ** 2, 2 * (q2 * q3 - q0 * q1)],
            [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), 1 - 2 * q1 ** 2 - 2 * q2 ** 2]
        ])

        return R

    def quaternion_multiply(self, q1, q2):
        """
        Multiply two quaternions
        q = [q0, q1, q2, q3] = [w, x, y, z]
        """
        w1, x1, y1, z1 = q1
        w2, x2, y2, z2 = q2

        return np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,  # w
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,  # x
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,  # y
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2  # z
        ])

    def quaternion_conjugate(self, q):
        """
        Compute quaternion conjugate
        q = [q0, q1, q2, q3] = [w, x, y, z]
        """
        return np.array([q[0], -q[1], -q[2], -q[3]])

    def rotate_vector(self, v, q):
        """
        Rotate vector v by quaternion q
        """
        # Convert vector to pure quaternion
        v_quat = np.array([0, v[0], v[1], v[2]])

        # Compute rotation: q * v * q^*
        q_conj = self.quaternion_conjugate(q)
        rotated = self.quaternion_multiply(
            self.quaternion_multiply(q, v_quat),
            q_conj
        )

        # Return vector part
        return rotated[1:]

    def compute_measurement_jacobian(self, q, acc_earth, mag_earth):
        """
        Compute Jacobian matrix for measurement model
        """
        # Initialize Jacobian matrix (6x4 for accelerometer and magnetometer)
        J = np.zeros((6, 4))

        # Compute Jacobian for accelerometer measurements
        q0, q1, q2, q3 = q

        # Accelerometer Jacobian
        J[0:3, 0] = np.array([
            2 * (q0 * acc_earth[0] + q3 * acc_earth[1] - q2 * acc_earth[2]),
            2 * (q3 * acc_earth[0] + q0 * acc_earth[1] + q1 * acc_earth[2]),
            2 * (-q2 * acc_earth[0] + q1 * acc_earth[1] + q0 * acc_earth[2])
        ])

        J[0:3, 1] = np.array([
            2 * (q1 * acc_earth[0] + q2 * acc_earth[1] + q3 * acc_earth[2]),
            2 * (q2 * acc_earth[0] - q1 * acc_earth[1] - q0 * acc_earth[2]),
            2 * (q3 * acc_earth[0] - q0 * acc_earth[1] - q1 * acc_earth[2])
        ])

        J[0:3, 2] = np.array([
            2 * (-q2 * acc_earth[0] + q1 * acc_earth[1] + q0 * acc_earth[2]),
            2 * (q1 * acc_earth[0] + q2 * acc_earth[1] + q3 * acc_earth[2]),
            2 * (-q0 * acc_earth[0] - q3 * acc_earth[1] + q2 * acc_earth[2])
        ])

        J[0:3, 3] = np.array([
            2 * (-q3 * acc_earth[0] + q0 * acc_earth[1] + q1 * acc_earth[2]),
            2 * (q0 * acc_earth[0] - q3 * acc_earth[1] + q2 * acc_earth[2]),
            2 * (q1 * acc_earth[0] + q2 * acc_earth[1] + q3 * acc_earth[2])
        ])

        # Magnetometer Jacobian (similar structure but with mag_earth)
        J[3:6, 0] = np.array([
            2 * (q0 * mag_earth[0] + q3 * mag_earth[1] - q2 * mag_earth[2]),
            2 * (q3 * mag_earth[0] + q0 * mag_earth[1] + q1 * mag_earth[2]),
            2 * (-q2 * mag_earth[0] + q1 * mag_earth[1] + q0 * mag_earth[2])
        ])

        J[3:6, 1] = np.array([
            2 * (q1 * mag_earth[0] + q2 * mag_earth[1] + q3 * mag_earth[2]),
            2 * (q2 * mag_earth[0] - q1 * mag_earth[1] - q0 * mag_earth[2]),
            2 * (q3 * mag_earth[0] - q0 * mag_earth[1] - q1 * mag_earth[2])
        ])

        J[3:6, 2] = np.array([
            2 * (-q2 * mag_earth[0] + q1 * mag_earth[1] + q0 * mag_earth[2]),
            2 * (q1 * mag_earth[0] + q2 * mag_earth[1] + q3 * mag_earth[2]),
            2 * (-q0 * mag_earth[0] - q3 * mag_earth[1] + q2 * mag_earth[2])
        ])

        J[3:6, 3] = np.array([
            2 * (-q3 * mag_earth[0] + q0 * mag_earth[1] + q1 * mag_earth[2]),
            2 * (q0 * mag_earth[0] - q3 * mag_earth[1] + q2 * mag_earth[2]),
            2 * (q1 * mag_earth[0] + q2 * mag_earth[1] + q3 * mag_earth[2])
        ])

        return J

    def gauss_newton_optimization(self, acc_body, mag_body, acc_earth, mag_earth, initial_quaternion):
        """
        Gauss-Newton optimization for finding optimal quaternion
        """
        q = initial_quaternion.copy()
        max_iterations = 3  # As suggested in the paper

        for _ in range(max_iterations):
            # Create rotation matrix from current quaternion
            R = self.quaternion_to_rotation_matrix(q)

            # Predict measurements in body frame
            acc_pred = R @ acc_earth
            mag_pred = R @ mag_earth

            # Compute error
            error_acc = acc_body - acc_pred
            error_mag = mag_body - mag_pred
            error = np.concatenate([error_acc, error_mag])

            # Compute Jacobian
            J = self.compute_measurement_jacobian(q, acc_earth, mag_earth)

            # Gauss-Newton update
            try:
                delta = np.linalg.inv(J.T @ J) @ J.T @ error
                q = q + delta
                q = q / np.linalg.norm(q)  # Normalize
            except np.linalg.LinAlgError:
                print("Warning: Singular matrix in Gauss-Newton update")
                break

        return q

    def predict(self):
        """Prediction step"""
        # Extract current states
        p, q, r = self.state[:3]
        quat = self.state[3:]

        # Angular rate model
        p_dot = -p / self.tau
        q_dot = -q / self.tau
        r_dot = -r / self.tau

        # Quaternion rate
        omega = np.array([0, p, q, r])
        quat_dot = 0.5 * self.quaternion_multiply(quat, omega)

        # Update state
        self.state[:3] += np.array([p_dot, q_dot, r_dot]) * self.dt
        self.state[3:] += quat_dot * self.dt

        # Normalize quaternion
        self.state[3:] /= np.linalg.norm(self.state[3:])

        # Compute Jacobian
        F = self.get_state_transition_matrix()

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

    def get_state_transition_matrix(self):
        """Compute state transition matrix"""
        F = np.eye(7)
        dt = self.dt
        p, q, r = self.state[:3]
        a, b, c, d = self.state[3:]

        # Angular rate terms
        F[:3, :3] = np.eye(3) * (1 - dt / self.tau)

        # Quaternion terms
        F[3:, 3:] = np.eye(4) + dt * 0.5 * np.array([
            [0, -p, -q, -r],
            [p, 0, r, -q],
            [q, -r, 0, p],
            [r, q, -p, 0]
        ])

        return F

    def update(self, measurement):
        """Update step using MARG measurements"""
        # Compute Kalman gain
        H = np.eye(7)  # Measurement matrix is identity
        S = H @ self.P @ H.T + self.R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        innovation = measurement - self.state
        self.state += K @ innovation

        # Normalize quaternion part
        self.state[3:] /= np.linalg.norm(self.state[3:])

        # Update covariance
        self.P = (np.eye(7) - K @ H) @ self.P

    def process_measurement(self, gyro, accel, mag, quaternion):
        """Process one set of MARG sensor measurements"""
        # First predict
        self.predict()

        # Combine with gyro for measurement update
        measurement = np.zeros(7)
        measurement[:3] = gyro
        measurement[3:] = quaternion

        # Update
        self.update(measurement)

        return self.state.copy()

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])

def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(q)
    if norm > 1e-10:
       q = q / norm
    return q

# https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x: float, y: float, z: float, w: float):
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

    return roll_x, pitch_y, yaw_z  # in radians

def quaternion_to_euler_angles(q: np.ndarray):
    """
    Convert quaternion to Euler angles (roll, pitch, yaw) in radians.
    Quaternion format assumed as [w, x, y, z].
    """
    w, x, y, z = q
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp) if abs(sinp) <= 1 else np.pi / 2 * np.sign(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw

def quaternion_multiply(q, r):
    w0, x0, y0, z0 = q
    w1, x1, y1, z1 = r
    return np.array([
        -x0 * x1 - y0 * y1 - z0 * z1 + w0 * w1,
        x0 * w1 + y0 * z1 - z0 * y1 + w0 * x1,
        -x0 * z1 + y0 * w1 + z0 * x1 + w0 * y1,
        x0 * y1 - y0 * x1 + z0 * w1 + w0 * z1], dtype=float)

def integrate_quaternion(q, g, dt):
    omega = np.array([0, g[0], g[1], g[2]])
    q = np.array([q[0], q[1], q[2], q[3]])

    q_dot = 0.5 * quaternion_multiply(q, omega)
    q_new = q + q_dot * dt
    norm = np.linalg.norm(q_new)
    return q_new / norm  # Normalize quaternion

def quaternion_to_rotation_matrix(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y ** 2 + z ** 2), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x ** 2 + z ** 2), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x ** 2 + y ** 2)]
    ])
