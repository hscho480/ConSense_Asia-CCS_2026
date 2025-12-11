import numpy as np
from scipy.spatial.distance import mahalanobis

class WeightDeterminator:
    """
    [Step 2] Context Weight Determination
    Calculates Mahalanobis distance and performs Source Attribution.
    """
    def __init__(self, feature_dim, groups, alpha, beta, threshold):
        self.dim = feature_dim
        self.groups = groups
        self.alpha = alpha
        self.beta = beta
        self.threshold = threshold

        # Initialize statistical model (assuming zero-mean residuals)
        self.mu = np.zeros(feature_dim)
        self.sigma = np.eye(feature_dim) * 1e-2
        self.inv_sigma = np.eye(feature_dim) * 100

    def compute_weights(self, c_t):
        """
        Returns: weights, anomaly_score, status
        """
        # 1. Check Full Context Consistency
        d2_full = self._calc_mahalanobis(c_t, self.mu, self.inv_sigma)
        weights = np.ones(self.dim)

        # Case: Benign
        if d2_full < self.threshold:
            self._update_model(c_t)
            return weights, d2_full, "Benign"

        # 2. Source Attribution (Iterative Exclusion)
        max_reduction = 0.0
        culprit_group = None

        for group_name, indices in self.groups.items():
            mask = np.ones(self.dim, dtype=bool)
            mask[indices] = False
            
            sub_c = c_t[mask]
            sub_mu = self.mu[mask]
            sub_sigma = self.sigma[np.ix_(mask, mask)]
            
            try:
                sub_inv = np.linalg.inv(sub_sigma)
                sub_d2 = self._calc_mahalanobis(sub_c, sub_mu, sub_inv)
            except np.linalg.LinAlgError:
                sub_d2 = d2_full

            reduction = d2_full - sub_d2
            if reduction > max_reduction:
                max_reduction = reduction
                culprit_group = group_name

        # 3. Weight Adjustment
        if culprit_group and max_reduction > (d2_full * 0.5):
            indices = self.groups[culprit_group]
            penalty = 1.0 / (1.0 + self.beta * max_reduction)
            weights[indices] = penalty
            return weights, d2_full, f"Attack Detected: {culprit_group}"

        return weights, d2_full, "Unknown/System-wide Anomaly"

    def _calc_mahalanobis(self, x, mu, inv_cov):
        delta = x - mu
        return np.dot(np.dot(delta.T, inv_cov), delta)

    def _update_model(self, x):
        # EMA Update
        self.mu = (1 - self.alpha) * self.mu + self.alpha * x
        
        diff = (x - self.mu).reshape(-1, 1)
        self.sigma = (1 - self.alpha) * self.sigma + self.alpha * np.dot(diff, diff.T)
        
        # Regularization
        self.sigma += np.eye(self.dim) * 1e-6
        self.inv_sigma = np.linalg.inv(self.sigma)