import numpy as np

def predict(x,F,P,Q):
    x = np.dot(F,x)
    P = np.dot( np.dot(F,P), F.T) + Q

def update(P,H,R,x,z):
    K = np.dot( np.dot(P,H.T), inv(( np.dot( np.dot(H,P), H.T))+R))
    x += np.dot(K, ( z - np.dot(H,x)))
    P = np.dot(np.dot(K, H), P)

