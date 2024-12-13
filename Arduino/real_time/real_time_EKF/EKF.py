import numpy as np
import math
from scipy.spatial.transform import Rotation

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

    return roll_x, pitch_y, yaw_z  # in radians

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
        self.R = np.eye(7) * 0.001

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

    def compute_orientation_quaternion(self, accel, mag):
        """Compute orientation quaternion from accelerometer and magnetometer"""
        acc = accel / np.linalg.norm(accel)
        mag = mag / np.linalg.norm(mag)

        acc_earth = np.array([0, 0, -9.81])
        mag_earth = np.array([0.22, 0, -0.44])
        # mag_earth = mag_earth / np.linalg.norm(mag_earth)

        self.gauss_newton_optimization(acc, mag, acc_earth, mag_earth, self.state[3:7])
        return R.as_quat()[[3, 0, 1, 2]]  # reorder to [w,x,y,z]

    def initialize_state_vector(self, gyro, quat):
        """Initialize the state vector with the first received measurement"""
        self.state[:3] = gyro
        self.state[3:] = quat/np.linalg.norm(quat)

        return self.state.copy()
