ConSense: Context-Aware Residual Correction for Enhanced Detection of Sensor Attacks in Robotic Vehicles

ConSense is a resilience framework designed to detect and mitigate sensor attacks (e.g., GPS Spoofing, Gyro Bias Injection) in robotic vehicles. By leveraging the contextual consistency among heterogeneous sensors, ConSense identifies compromised sensors and corrects state estimation errors in real-time.

This repository contains the ROS implementation of the ConSense framework, including the core detection logic, simulation environments, and analysis tools for reproducing the paper's results.

Core Mechanism
ConSense operates as an add-on module to standard state estimators (like EKF). It follows a 3-Step Pipeline:

1. Context Extraction (context_extractor.py)

Extracts "Inconsistency Features" ($c_t$) instead of raw sensor values.

Computes physical discrepancies, such as the difference between GNSS velocity and IMU-integrated velocity.

Captures the immediate impact of attacks on system dynamics.

2. Context Weight Determination (weight_determinator.py)

Assess the reliability of the current context using Mahalanobis Distance ($D_M^2$).

Anomaly Detection: Checks if $c_t$ deviates from the benign distribution (Zero-mean Gaussian).

Source Attribution: Performs Iterative Exclusion to identify the specific sensor group responsible for the anomaly.

Weight Adjustment: Assigns a reliability weight ($w_i$) to isolate the compromised sensor.

3. Residual Correction (residual_corrector.py)

Mitigates the attack impact using a Recursive Least Squares (RLS) filter.

Learns the correlation between the inconsistency ($c_t$) and the system residual ($r_t$).

Estimates and subtracts the attack bias from the residual before it affects the state estimator.
