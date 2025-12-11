import numpy as np

class ResidualCorrector:
    """
    [Step 3] Residual Correction (RLS Filter)
    Estimates and removes attack bias from the residual.
    """
    def __init__(self, feature_dim, lambda_rls):
        self.lam = lambda_rls
        self.theta = np.zeros(feature_dim)
        self.P = np.eye(feature_dim) * 100

    def correct(self, raw_residual, c_t, weights):
        # 1. Apply reliability weights
        z = c_t * weights
        
        # 2. Predict attack bias
        attack_bias_pred = np.dot(z, self.theta)
        
        # 3. Correct residual
        corrected_residual = raw_residual - attack_bias_pred

        # 4. Update RLS Filter
        P_z = np.dot(self.P, z)
        denominator = self.lam + np.dot(z, P_z)
        gain = P_z / denominator
        
        e_prior = raw_residual - attack_bias_pred 
        
        self.theta = self.theta + gain * e_prior
        self.P = (self.P - np.outer(gain, P_z)) / self.lam
        
        return corrected_residual