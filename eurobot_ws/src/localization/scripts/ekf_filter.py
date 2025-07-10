import numpy as np
import math


## ******************************
## EKF FILTER CLASS
## ******************************

class FilterEKF:

    def __init__(self, x, P, Q, normalize_angle):
        self.x = x  # Initial state estimate.
        self.P = P  # Initial covariance estimate.
        self.Q = Q
        self.normalize_angle = normalize_angle

    def get_current_state(self):
        return np.copy(self.x)
    
    def prediction(self, inc_t, u): # Non linear prediction
        x_ = self.get_fx(inc_t, u)
        A = self.get_A(inc_t, u)
        W = self.get_W(inc_t, u)
        self.x_ = np.copy(x_)
        self.P_ = A.dot(self.P).dot(A.transpose()) + W.dot(self.Q).dot(W.transpose())
    
    def false_prediction(self):
        self.x_ = np.copy(self.x)
        self.P_ = np.copy(self.P)
            
    def update_prediction(self):
        self.x = np.copy(self.x_)
        self.P = np.copy(self.P_)
        
    def update(self, z, H, R): # Linear Update 
        # Compute innovation
        innovation = z - H.dot(self.x_)
            
        innovation_cov = H.dot(self.P_).dot(H.transpose()) + R
        
        # Update step
        K = self.P_.dot(H.transpose()).dot(np.linalg.inv(innovation_cov))
        self.x = self.x_ + K.dot(innovation)
        self.P = (np.eye(self.P_.shape[0]) - K.dot(H)).dot(self.P_)
        
    def get_fx(self, inc_t, u):
        px = self.x[0, 0]
        py = self.x[1, 0]
        h = self.x[2, 0]
        v = u[0]
        w = u[1]
        px_ = px + v * inc_t * math.cos(h)
        py_ = py + v * inc_t * math.sin(h) 
        h_ = wrap_angle(h + w * inc_t)
        return np.array([[px_], [py_], [h_]])

    def get_A(self, inc_t, u):
        h = self.x[2, 0]
        v = u[0]
        A = np.array([[1, 0, -inc_t * v * math.sin(h)],
                      [0, 1, inc_t * v * math.cos(h)],
                      [0, 0, 1]])
        return A

    def get_W(self, inc_t, u):
        h = self.x[2, 0]
        v = u[0]
        w = u[1]
        t2 = (inc_t**2)/2.0
        W = np.array([[t2 * math.cos(h), 0],
                      [t2 * math.sin(h), 0],
                      [0, t2]])
        return W 



## ******************************
## UTIL FUNCTIONS
## ******************************

def wrap_angle(angle):
    """
    Porta un angle (en radians) dins del rang [-pi, pi].

    Args:
        angle (float): Angle en radians.

    Returns:
        float: Angle equivalent dins del rang [-pi, pi].
    """
    return (angle + math.pi) % (2 * math.pi) - math.pi