#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import tkinter as tk
import sys

# Import PoseEditorGUI
from tg_posing.pose_editor_gui import PoseEditorGUI

class RobotControlNode(Node):
    """Robot control ROS node"""

    def __init__(self):
        super().__init__('robot_pose_editor')  # Update node name

def main(args=None):
    """Main program entry"""
    # Initialize ROS
    rclpy.init(args=args)

    # Create ROS node
    node = RobotControlNode()

    try:
        # Create and run GUI - using PoseEditorGUI
        app = PoseEditorGUI(node)
        app.update_ros()  # Start ROS loop
        app.mainloop()
    except KeyboardInterrupt:
        # If KeyboardInterrupt occurs, ensure window is closed
        node.get_logger().info("KeyboardInterrupt detected, shutting down...")
    finally:
        # Always clean up resources
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()