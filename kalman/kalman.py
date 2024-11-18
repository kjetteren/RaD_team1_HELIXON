import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilter:
    def __init__(self,n,m):
        self._F = np.eye(n) # state transition matrix
        self._H = np.zeros((n,m)) # measurement matrix

        self._x = np.zeros((n,1)) # state vector
        self._P = np.zeros(n) # covariance matrix

        self._R = np.zeros(m) # measurement noise matrix
        self._Q = np.zeros(n) # process noise matrix

        self._u = np.zeros((m,1))
        self._z = np.zeros((m,1))

        self._K = np.zeros((n,m)) # Kalman gain matrix

    def predict(self, u, Q):
        self._x = np.dot(self._x, self._F) + np.dot(self._B, u)
        FP = np.dot(self._F, self._P)
        self._P = np.dot(FP, self._F.T) + Q

    def update(self,z,R):
        HP = np.dot(self._H, self._P)
        PHT = np.dot(self._P, self._H.T)
        self._K = np.dot(PHT, np.linalg.inv((np.dot(HP, self._H.T) + R)) )
        self._x += np.dot(self._K, (self._y - np.dot(self._H, self._x)))
        self._P = np.dot( (np.diag(len(self._P)) - np.dot(self._K, self._H)), self._P)

#TODO: POSITION TRACKING
def position_kf(n,m):
    S, v, ab = 0, 0, 0
    T, dt = 0, 0
    kf = KalmanFilter(n,m)
    kf.z = np.array([
        [S],
        [v],
        [ab]
    ])
    kf.F = np.array([
        [1, dt, -0.5*T**2],
        [0, 1, -T],
        [0, 0, T]
    ])
    kf.B = np.array([
        [0.5*T**2],
        [T],
        [0]
    ])
    Q_std, R_std = 0.01, 0.1
    kf.Q = np.array([
        [0.25*T**4 * Q_std**2, 0.5*T**3 * Q_std**2, 0],
        [0.5*T**3 * Q_std**2, T**2 * Q_std**2, 0],
        [0, 0, 0]
    ])
    kf.R = np.array([R_std])

    return kf

def update_position_x(kf, a, ab):
    T= 0.1
    kf.x[0] += kf.x[1]*T + 0.5*(a-kf.x[2])*T**2
    kf.x[1] += (a-kf.x[2])*T
    kf.x[2] = ab


# TODO: ORIENTATION TRACKING
def orientation_kf(n,m):
    alpha, qb = 0, 0
    dt, T = 0.1, 0.1
    kf = KalmanFilter(n,m)
    kf.x = np.array([
        [alpha],
        [qb]
    ])
    kf.F = np.array([
        [1, dt, -0.5*T**2],
        [0, 1, -T],
        [0, 0, 1]
    ])
    kf.B = np.array([
        [0.5*T**2],
        [T],
        [0]
    ])
    kf.H = np.array([1, 0, 0])
    Q_std, R_std = 0.01, 0.1
    kf.Q = np.array([
        [0.25*T**4 * Q_std**2, 0.5*T**3 * Q_std**2, 0],
        [0.5*T**3 * Q_std**2, T**2 * Q_std**2, 0],
        [0, 0, 0]
    ])
    kf.R = np.array([R_std])

def update_orientation_x(kf, w, qb):
    T = 0.1
    kf.x[0] += (w-kf.x[1])*T
    kf.x[1] = qb