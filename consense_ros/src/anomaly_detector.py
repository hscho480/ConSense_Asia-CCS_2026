#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32, String
from modules.context_extractor import ContextExtractor
from modules.weight_determinator import WeightDeterminator
from modules.residual_corrector import ResidualCorrector

class ConSenseNode:
    def __init__(self):
        rospy.init_node('consense_node')
        
        # Load Parameters
        alpha = rospy.get_param("/consense/alpha", 0.01)
        beta = rospy.get_param("/consense/beta", 1.0)
        lam = rospy.get_param("/consense/lambda_rls", 0.995)
        threshold = rospy.get_param("/consense/threshold_chisq", 15.5)
        
        raw_groups = rospy.get_param("/sensor_groups")
        groups = {k: list(range(v[0], v[-1]+1)) for k,v in raw_groups.items()}
        
        feature_dim = 9 

        # Initialize Modules
        self.extractor = ContextExtractor()
        self.determinator = WeightDeterminator(feature_dim, groups, alpha, beta, threshold)
        self.corrector = ResidualCorrector(feature_dim, lam)

        # Publishers & Subscribers
        self.pub_res = rospy.Publisher('/consense/corrected_residual', Float32, queue_size=10)
        self.pub_status = rospy.Publisher('/consense/status', String, queue_size=10)
        
        # Subscribe to baseline residual (e.g., from EKF)
        rospy.Subscriber('/baseline/residual', Float32, self.pipeline_callback)
        
        rospy.loginfo(f"ConSense Initialized (Dim: {feature_dim}, Thresh: {threshold})")

    def pipeline_callback(self, msg):
        raw_residual = msg.data
        
        # Step 1: Context Extraction
        c_t = self.extractor.get_vector()
        
        # Step 2: Weight Determination
        weights, score, status = self.determinator.compute_weights(c_t)
        
        # Step 3: Residual Correction
        final_residual = self.corrector.correct(raw_residual, c_t, weights)
        
        # Publish
        self.pub_res.publish(final_residual)
        self.pub_status.publish(f"Score: {score:.2f} | {status}")

if __name__ == '__main__':
    try:
        ConSenseNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass