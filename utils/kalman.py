import numpy as np
from filterpy.kalman import KalmanFilter

class KalmanFilter:
    def __init__(self,n,m):
        self._F = np.eye(n) # state transition matrix
        self._H = np.zeros((n,m)) # measurement matrix
        self._B = np.zeros((n,1)) # input matrix

        self._x = np.zeros((n,1)) # state vector
        self._P = np.ones(n) # covariance matrix

        self._R = np.zeros(m) # measurement noise matrix
        self._Q = np.zeros(n) # process noise matrix

        self._u = np.zeros((m,1)) # measurement vector
            # for position tracking it is acceleration
        self._y = np.zeros((m,1)) # estimation vector
            # for position tracking it is position

        self._K = np.zeros((n,m)) # Kalman gain matrix

    def predict(self, u, Q):
        # u and Q in the code below are not parts of KalmanFilter object
        # so that we could update them in necessary
        self._x = np.dot(self._F, self._x) + np.dot(self._B, u)
        self._P = congruence_transformation(self._F, self._P) + Q

    def update(self,y,R):
        print(self._H.shape,"times",self._P.shape,"times",self._H.T.shape)
        self._K = np.dot( np.dot(self._H.T, self._P),
                          ( congruence_transformation(self._H, self._P) + R))
        self._x += np.dot(self._K, (y - np.dot(self._H, self._x)))
        self._P = np.dot( (np.eye(len(self._P)) - np.dot(self._K, self._H)), self._P)

#TODO: POSITION TRACKING
def position_kf(n,m):
    S, v, ab = 0, 0, 0
    T, dt = 0, 0
    kf = KalmanFilter(n,m)
    kf.x = np.array([
        [float(S)],
        [float(v)],
        [float(ab)]
    ])
    kf.F = np.array([
        [1, dt, -0.5*T**2],
        [0, 1, -T],
        [0, 0, T]
    ])
    kf.H = np.array([ [1, 0, 0] ])
    kf.B = np.array([
        [0.5*T**2],
        [T],
        [0]
    ])
    s_std, v_std, ab_std = 0.01, 0.1, 0.02
    kf.P = np.array([
        [s_std**2, s_std*v_std, s_std*ab_std],
        [v_std*s_std, v_std**2, v_std*ab_std],
        [ab_std*s_std, ab_std*v_std, ab_std**2]
    ])
    Q_std, R_std = 0.01, 0.1
    kf.Q = np.array([
        [0.25*T**4 * Q_std**2, 0.5*T**3 * Q_std**2, 0],
        [0.5*T**3 * Q_std**2, T**2 * Q_std**2, 0],
        [0, 0, 0]
    ])
    kf.R = np.array([R_std])

    return kf


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
    kf.H = np.array([1, 0])
    alpha_std, qb_std = 0.01, 0.1
    kf.P = np.array([
        [alpha_std**2, alpha_std*qb_std],
        [qb_std**2, qb_std*alpha_std],
    ])
    Q_std, R_std = 0.01, 0.1
    kf.Q = np.array([
        [0.25*T**4 * Q_std**2, 0.5*T**3 * Q_std**2, 0],
        [0.5*T**3 * Q_std**2, T**2 * Q_std**2, 0],
        [0, 0, 0]
    ])
    kf.R = np.array([R_std])


def congruence_transformation(A, B):
    """
    A times B times A.T
    """
    #print(A.shape, " dot ", B.shape, " dot ", A.T.shape)
    if A.shape[1] != B.shape[0] != B.shape[1]:
        raise ValueError("Matrices in congruence transformation have wrong dimensions.")

    intermediate = np.dot(A, B)
    return np.dot(intermediate, A.T)