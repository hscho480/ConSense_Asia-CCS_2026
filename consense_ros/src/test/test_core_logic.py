import unittest
import numpy as np
from modules.weight_determinator import WeightDeterminator

class TestConSenseLogic(unittest.TestCase):
    def test_mahalanobis_benign(self):
        dim = 3
        groups = {'group1': [0, 2]}
        wd = WeightDeterminator(dim, groups, alpha=0.1, beta=1.0, threshold=10.0)
        
        c_t = np.zeros(dim)
        weights, score, status = wd.compute_weights(c_t)
        
        self.assertTrue(np.all(weights == 1.0))
        self.assertEqual(status, "Benign")

    def test_anomaly_detection(self):
        dim = 3
        groups = {'test_group': [0, 2]}
        wd = WeightDeterminator(dim, groups, alpha=0.0, beta=1.0, threshold=2.0)
        
        c_t = np.array([10.0, 10.0, 10.0])
        weights, score, status = wd.compute_weights(c_t)
        
        self.assertLess(weights[0], 1.0)
        self.assertIn("Attack Detected", status)

if __name__ == '__main__':
    unittest.main()