#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlotter(Node):
    #Subscribing to the topic
    def __init__(self):
        super().__init__('trajectory_plotter')
        self.subscription = self.create_subscription(
            JointTrajectory,
            'joint_trajectory',
            self.trajectory_callback,
            10
        )
        self.get_logger().info('Trajectory Plotter Node started, waiting for trajectory...')
    # Converts JointTrajectory message to a NumPy array for efficient slicing and plotting.
    # Each joint's position is isolated across the time dimension for visualization. 
    def trajectory_callback(self, msg):
        plt.close('all')
        self.get_logger().info('Received trajectory, plotting...')
        
        joint_names = msg.joint_names
        num_joints = len(joint_names)
        num_points = len(msg.points)
        
        times = [point.time_from_start.sec + point.time_from_start.nanosec * 1e-9 
                 for point in msg.points]
        positions = np.array([point.positions for point in msg.points])
        
        fig, axes = plt.subplots(num_joints, 1, figsize=(10, 12))
        fig.suptitle('Joint Trajectory', fontsize=14)
        
        for i, name in enumerate(joint_names):
            axes[i].plot(times, positions[:, i], label=name)
            axes[i].set_ylabel(f'{name}\n(rad)')
            axes[i].legend()
            axes[i].grid(True)
        
        axes[-1].set_xlabel('Time (s)')
        plt.tight_layout()
        plt.show()

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    