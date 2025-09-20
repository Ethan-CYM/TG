#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from bodyctrl_msgs.msg import MotorStatusMsg

class RosInterface:
    """Handles all ROS communications"""

    def __init__(self, node, body_part, cmd_topic, status_topic, status_callback):
        """Initialize ROS interface

        Args:
            node: ROS node
            body_part: Body part name (used as frame_id)
            cmd_topic: Command publishing topic
            status_topic: Status subscription topic
            status_callback: Status callback function
        """
        self.node = node
        self.body_part = body_part

        # Publisher: send control commands
        self.pub = node.create_publisher(CmdSetMotorPosition, cmd_topic, 10)

        # Subscriber: receive status
        self.sub = node.create_subscription(
            MotorStatusMsg,
            status_topic,
            status_callback,
            10
        )

        # Logging
        self.node.get_logger().info(f"Initialized ROS interface for {body_part}: "
                                    f"publishing to {cmd_topic}, "
                                    f"subscribing to {status_topic}")

    def send_motor_command(self, motor_id, pos, vel, cur):
        """Send motor control command"""
        try:
            # Construct command message
            cmd = CmdSetMotorPosition()
            cmd.header = Header()
            cmd.header.stamp = self.node.get_clock().now().to_msg()
            cmd.header.frame_id = self.body_part

            motor = SetMotorPosition()
            motor.name = motor_id
            motor.pos = pos
            motor.spd = vel  # Using spd field
            motor.cur = cur

            cmd.cmds = [motor]
            self.pub.publish(cmd)

            self.node.get_logger().info(
                f"Publishing motor command to {self.body_part}: "
                f"id={motor.name}, pos={motor.pos:.3f}, vel={motor.spd:.2f}, cur={motor.cur:.1f}")

            return True

        except Exception as e:
            self.node.get_logger().error(f"Error sending command: {str(e)}")
            return False