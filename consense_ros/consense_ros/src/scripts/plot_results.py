#!/usr/bin/env python3
import rospy
import matplotlib.pyplot as plt
from std_msgs.msg import Float32
import sys

class ResultPlotter:
    """
    A utility node to record and visualize the performance of ConSense.
    It subscribes to both raw and corrected residuals and plots them
    for comparison upon termination.
    """
    def __init__(self):
        rospy.init_node('plot_results', anonymous=True)

        # Data buffers
        self.times = []
        self.raw_residuals = []
        self.corrected_residuals = []
        self.start_time = None

        # Load threshold for visualization line
        self.threshold = rospy.get_param("~consense/threshold_chisq", 15.5)

        # Subscribers
        # Note: Ensure these topic names match your launch file / system configuration
        rospy.Subscriber('/baseline/residual', Float32, self.raw_cb)
        rospy.Subscriber('/consense/corrected_residual', Float32, self.corrected_cb)

        rospy.loginfo("Result Plotter Started. Collect data and press Ctrl+C to plot.")

    def raw_cb(self, msg):
        if self.start_time is None:
            self.start_time = rospy.get_time()
        
        current_time = rospy.get_time() - self.start_time
        self.times.append(current_time)
        self.raw_residuals.append(msg.data)
        
        # To keep arrays synchronized for simple plotting, we append the last known 
        # corrected value if the lengths mismatch, or handle sync properly.
        # For this simple script, we assume topics come at similar rates or we just
        # plot raw vs time. (Corrected is handled in its own callback)

    def corrected_cb(self, msg):
        # Store corrected residuals. 
        # Note: In a rigorous setup, you should sync messages by header timestamp.
        # Here we just store data for visualization assuming roughly sync arrival.
        self.corrected_residuals.append(msg.data)

    def plot_and_save(self):
        if not self.times:
            rospy.logwarn("No data collected.")
            return

        # Ensure arrays are same length for plotting (trim to shortest)
        min_len = min(len(self.times), len(self.raw_residuals), len(self.corrected_residuals))
        t = self.times[:min_len]
        raw = self.raw_residuals[:min_len]
        corr = self.corrected_residuals[:min_len]

        plt.figure(figsize=(10, 6))
        
        # Plot Raw Residuals
        plt.plot(t, raw, 'r--', label='Baseline Residual (Raw)', alpha=0.6, linewidth=1.5)
        
        # Plot Corrected Residuals
        plt.plot(t, corr, 'g-', label='ConSense Residual (Corrected)', linewidth=2.0)

        # Plot Threshold Lines
        plt.axhline(y=self.threshold, color='k', linestyle=':', label='Threshold', alpha=0.5)
        plt.axhline(y=-self.threshold, color='k', linestyle=':', alpha=0.5)

        plt.title("ConSense Performance: Residual Correction", fontsize=14, fontweight='bold')
        plt.xlabel("Time (s)", fontsize=12)
        plt.ylabel("Residual Magnitude", fontsize=12)
        plt.legend(loc='upper right')
        plt.grid(True, linestyle='--', alpha=0.7)
        plt.tight_layout()

        # Save and Show
        filename = "consense_result.png"
        plt.savefig(filename)
        rospy.loginfo(f"Graph saved to {filename}")
        plt.show()

if __name__ == '__main__':
    plotter = ResultPlotter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # Plot when the node is shut down (Ctrl+C)
        rospy.loginfo("Plotting collected data...")
        plotter.plot_and_save()