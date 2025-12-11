# ConSense: Context-Aware Resilience for Anomaly Detection

ConSense is a ROS-based resilience framework designed to detect sensor attacks and mitigate state estimation errors in real-time for robotic vehicles (UAVs/UGVs).

## Getting Started
### Prerequisites
- **OS**: Ubuntu 20.04 (Focal)
- **ROS**: Noetic Ninjemys
- **Python**: 3.8+
- **Dependencies**: `numpy`, `scipy`, `rospy`, `std_msgs`, `geometry_msgs`


### Installation
1. Clone the repository

2. Build the package
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

3. Grant execution permissions
```bash
chmod +x src/consense/src/*.py
```

## Usage
1. Configuration (Optional)
You can modify parameters in `consense_ros/config/params.yaml`:
- `threshold_chisq`: Anomaly detection threshold (default: 15.5)
- `alpha`: Learning rate for context model
- `lambda_rls`: Forgetting factor for RLS filter

2. Execution
Launch the main detection node:
```bash
roslaunch consense_ros consense.launch
```

3. Monitoring
Check real-time detection status and anomaly scores:
```bash
rostopic echo /consense/status
# Output Example: "Score: 32.41 | Attack Detected: gnss_consistency"
```
