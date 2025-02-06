import numpy as np

class KalmanFilter(object):
    
    # x0 - initial guess of the state vector (position)
    # P0 - initial guess of the covariance matrix of the state estimation error
    # A,B,C - system matrices describing the system model
    # Q - covariance matrix of the process noise 
    # R - covariance matrix of the measurement noise
    
    def __init__(self, x0, P0, A, B, C, Q, R):
        # Initialize vectors and matrices for a 1D state system (position)
        self.x0 = x0  # Initial position guess (1x1 matrix)
        self.P0 = P0  # Initial covariance matrix (1x1)
        self.A = A    # State transition matrix	 (1x1)
        self.B = B    # Control input matrix (1x1, likely zero)
        self.C = C    # Measurement matrix (1x1)
        self.Q = Q    # Process noise covariance (1x1)
        self.R = R    # Measurement noise covariance (1x1)
        
        # This variable tracks the current time step k of the estimator
        self.currentTimeStep = 0
        
        # Lists to store estimates and covariance matrices
        self.estimates_aposteriori = [x0]  # Start with initial guess x0
        self.estimates_apriori = []
        self.estimationErrorCovarianceMatricesAposteriori = [P0]  # Start with initial guess P0
        self.estimationErrorCovarianceMatricesApriori = []
        self.gainMatrices = []
        self.errors = []
        
    # This function propagates x_{k-1}^{+} through the model to compute x_{k}^{-}
    def propagateDynamics(self):
        # xk_minus = A * xk_minus (no control input here, B*u = 0)
        xk_minus = self.A * self.estimates_aposteriori[self.currentTimeStep]
        
        # Pk_minus = A * Pk_plus * A' + Q
        Pk_minus = self.A * self.estimationErrorCovarianceMatricesAposteriori[self.currentTimeStep] * self.A.T + self.Q
        
        self.estimates_apriori.append(xk_minus)
        self.estimationErrorCovarianceMatricesApriori.append(Pk_minus)
        
        # Increment time step
        self.currentTimeStep += 1
    
    # This function updates the a posteriori estimate based on the measurement
    def computeAposterioriEstimate(self, currentMeasurement):
        # Compute Kalman Gain
        Kk = (self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1] * self.C.T) * np.linalg.inv(
            self.R + self.C * self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1] * self.C.T)
        
        # Prediction error (residual)
        error_k = currentMeasurement - self.C * self.estimates_apriori[self.currentTimeStep-1]
        
        # A posteriori estimate
        xk_plus = self.estimates_apriori[self.currentTimeStep-1] + Kk * error_k
        
        # A posteriori covariance matrix update
        IminusKkC = np.eye(self.x0.shape[0]) - Kk * self.C
        Pk_plus = IminusKkC * self.estimationErrorCovarianceMatricesApriori[self.currentTimeStep-1] * IminusKkC.T + Kk * self.R * Kk.T
        
        # Store results
        self.gainMatrices.append(Kk)
        self.errors.append(error_k)
        self.estimates_aposteriori.append(xk_plus)
        self.estimationErrorCovarianceMatricesAposteriori.append(Pk_plus)


