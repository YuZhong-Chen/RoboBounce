import math
import cv2
import numpy as np


# ---------------------------------------------------------------------------- #
#                                 1 Euro Filter                                #
# ---------------------------------------------------------------------------- #
class OneEuroFilter:
    def __init__(self, t0, x0, min_cutoff=1.0, beta=0.0, d_cutoff=1.0):
        self.min_cutoff = min_cutoff
        self.beta = beta
        self.d_cutoff = d_cutoff
        self.x_prev = float(x0)
        self.dx_prev = 0.0
        self.t_prev = float(t0)

    def smoothing_factor(self, t_e, cutoff):
        r = 2 * math.pi * cutoff * t_e
        return r / (r + 1)

    def exponential_smoothing(self, a, x, x_prev):
        return a * x + (1 - a) * x_prev

    def __call__(self, t, x):
        t_e = t - self.t_prev

        # Avoid duplicate timestamps or time going backward
        if t_e <= 0.0:
            return self.x_prev

        # Estimate velocity
        dx = (x - self.x_prev) / t_e
        dx_hat = self.exponential_smoothing(self.smoothing_factor(t_e, self.d_cutoff), dx, self.dx_prev)

        # Dynamic cutoff frequency based on velocity
        cutoff = self.min_cutoff + self.beta * abs(dx_hat)

        # Smooth position
        a = self.smoothing_factor(t_e, cutoff)
        x_hat = self.exponential_smoothing(a, x, self.x_prev)

        # Update state
        self.x_prev = x_hat
        self.dx_prev = dx_hat
        self.t_prev = t
        return x_hat


# ---------------------------------------------------------------------------- #
#                                Kalman Filter                                 #
# ---------------------------------------------------------------------------- #
class KalmanFilterWrapper:
    def __init__(self, dt=0.033):
        # State: [x, y, z, vx, vy, vz]
        # Measurement: [x, y, z]
        self.kf = cv2.KalmanFilter(6, 3)

        # Transition matrix (Constant velocity model)
        self.kf.transitionMatrix = np.array(
            [
                [1, 0, 0, dt, 0, 0],
                [0, 1, 0, 0, dt, 0],
                [0, 0, 1, 0, 0, dt],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, 0],
                [0, 0, 0, 0, 0, 1],
            ],
            dtype=np.float32,
        )

        # Measurement matrix
        self.kf.measurementMatrix = np.array(
            [
                [1, 0, 0, 0, 0, 0],
                [0, 1, 0, 0, 0, 0],
                [0, 0, 1, 0, 0, 0],
            ],
            dtype=np.float32,
        )

        # Noise covariance
        # Process noise (Q)
        self.kf.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-2

        # Measurement noise (R)
        self.kf.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-1
        self.kf.measurementNoiseCov[2, 2] = 0.5  # Higher penalty for Z-axis (Depth)

        self.kf.errorCovPost = np.eye(6, dtype=np.float32)
        self.is_initialized = False

    def update(self, meas_xyz, dt):
        # Update dt in transition matrix
        self.kf.transitionMatrix[0, 3] = dt
        self.kf.transitionMatrix[1, 4] = dt
        self.kf.transitionMatrix[2, 5] = dt

        prediction = self.kf.predict()

        if meas_xyz is not None:
            if not self.is_initialized:
                # Initialization
                self.kf.statePost = np.array([[meas_xyz[0]], [meas_xyz[1]], [meas_xyz[2]], [0], [0], [0]], dtype=np.float32)
                self.is_initialized = True
                return meas_xyz
            else:
                # Correction
                z = np.array(meas_xyz, dtype=np.float32).reshape(3, 1)
                self.kf.correct(z)
                return self.kf.statePost[:3].flatten()
        else:
            # Prediction only (if initialized)
            return prediction[:3].flatten() if self.is_initialized else None

    def reset(self):
        self.is_initialized = False
