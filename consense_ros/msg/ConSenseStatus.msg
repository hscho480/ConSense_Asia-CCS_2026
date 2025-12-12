Header header

# Core Metrics
float32 anomaly_score       # Mahalanobis Distance (D^2)
float32 corrected_residual  # Final output

# Detection Info
bool is_anomaly             # True if score > threshold
string compromised_sensor   # Name of the sensor group (e.g., "gnss", "imu")
float32[] weights           # Current weights assigned to features