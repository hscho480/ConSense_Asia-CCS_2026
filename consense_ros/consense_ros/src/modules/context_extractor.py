import numpy as np
import rospy
from sensor_msgs.msg import Imu
from geometry_msgs.msg import TwistStamped

class ContextExtractor:
    """
    [Step 1] Context Feature Extraction
    Extracts 'Inconsistency Features' (residuals) between heterogeneous sensors.
    """
    def __init__(self):
        # Feature Vector Size: 9
        # [0-2]: GNSS-IMU Velocity Discrepancy (X, Y, Z)
        # [3-5]: Acceleration Residuals (Gravity removed)
        # [6-8]: Gyro Rates
        self.feature_vector = np.zeros(9)
        
        self.last_imu_time = None
        self.integrated_vel = np.zeros(3)
        
        # Subscribers
        rospy.Subscriber('/mavros/imu/data', Imu, self._imu_cb)
        rospy.Subscriber('/mavros/global_position/raw/vel', TwistStamped, self._gnss_vel_cb)

    def _imu_cb(self, msg):
        dt = 0.02
        if self.last_imu_time is not None:
            dt = (msg.header.stamp - self.last_imu_time).to_sec()
        self.last_imu_time = msg.header.stamp

        # Group 2: Acceleration Residuals [3-5]
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z - 9.81 # Remove gravity bias
        
        self.feature_vector[3] = acc_x
        self.feature_vector[4] = acc_y
        self.feature_vector[5] = acc_z

        # Group 3: Gyro Rates [6-8]
        self.feature_vector[6] = msg.angular_velocity.x
        self.feature_vector[7] = msg.angular_velocity.y
        self.feature_vector[8] = msg.angular_velocity.z

        # Dead Reckoning for Group 1 comparison
        self.integrated_vel[0] += acc_x * dt
        self.integrated_vel[1] += acc_y * dt
        self.integrated_vel[2] += acc_z * dt

    def _gnss_vel_cb(self, msg):
        gx = msg.twist.linear.x
        gy = msg.twist.linear.y
        gz = msg.twist.linear.z
        
        # Group 1: Velocity Discrepancy [0-2]
        # Diff = GNSS_Vel - IMU_Integrated_Vel
        self.feature_vector[0] = gx - self.integrated_vel[0]
        self.feature_vector[1] = gy - self.integrated_vel[1]
        self.feature_vector[2] = gz - self.integrated_vel[2]
        
        # Reset integration to sync with GNSS
        self.integrated_vel = np.array([gx, gy, gz])

    def get_vector(self):
        return self.feature_vector.copy()