import numpy as np

def predict(x,F,P,Q):
    x = np.dot(F,x)
    P = np.dot( np.dot(F,P), F.T) + Q

def update(P,H,R,x,z):
    K = np.dot( np.dot(P,H.T), inv(( np.dot( np.dot(H,P), H.T))+R))
    x += np.dot(K, ( z - np.dot(H,x)))
    P = np.dot(np.dot(K, H), P)

def set_up_spiral(R=4.125, N=3.3, H=13, T=1300):
	true_position = np.array([[-R * np.cos(i * (2 * np.pi * N) / T),  R * np.sin(i * (2 * np.pi * N) / T), (i / T) * H] for i in range(T)])
	
	return true_position
	

