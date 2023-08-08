
import numpy as np
from scipy.stats import chi2
from numpy.linalg import inv


class KalmanFilter:
    def __init__(self, F, B, H, x, P, Q, R):
        self.F = F
        self.B = B
        self.H = H
        self.x = x
        self.P = P
        self.Q = Q
        self.R = R

    # Update the state transition matrix based on the time step dt
    def update_state_transition_matrix(self, dt):
        n = self.F.shape[0]
        for i in range(n // 2):
            self.F[i, i + n // 2] = dt

    # Predict step: update the state and error covariance
    def predict(self, u):
        self.x = self.F @ self.x + self.B @ u
        self.P = self.F @ self.P @ self.F.T + self.Q

    # Update step: update the state and error covariance based on the observation
    def update(self, z, outlier_threshold=0.01):
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R

        # Calculate the Mahalanobis distance
        D = np.sqrt((y.T @ inv(S) @ y)[0, 0])

        # Calculate the threshold based on the chi-squared distribution
        threshold = chi2.ppf(1 - outlier_threshold, df=y.shape[0])

        # If the Mahalanobis distance is greater than the threshold,
        # consider the observation as an outlier and skip the update step
        if D > threshold:
            return

        K = self.P @ self.H.T @ inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(self.P.shape[0]) - K @ self.H) @ self.P

    # Get the current state estimate
    def get_current_state(self):
        return self.x
