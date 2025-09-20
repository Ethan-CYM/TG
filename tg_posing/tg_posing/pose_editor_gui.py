#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk
import signal
import sys
import time

from tg_posing.motor_row import MotorRow
from tg_posing.ros_interface import RosInterface
from tg_posing.motor_config import BODY_PARTS_TOPICS


class PoseEditorGUI(tk.Tk):
    """Robot joint control GUI main class"""

    def __init__(self, node):
        """Initialize GUI

        Args:
            node: ROS node
        """
        super().__init__()

        self.node = node
        self.title("Robot Joint Position control")

        # Set window size to larger dimensions
        self.geometry("1000x700")  # Increase window size

        # Set background color to pure white
        self.configure(bg="white")

        # Create styles
        self.style = ttk.Style()
        self.style.configure('TFrame', background='white')
        self.style.configure('TLabel', background='white', font=('Arial', 10))
        self.style.configure('TButton', font=('Arial', 10))
        self.style.configure('TNotebook.Tab', padding=[20, 5], font=('Arial', 12))  # Increase Tab size

        # Set Ctrl+C handler
        self.setup_signal_handler()

        # Add command throttling variables
        self.last_command_time = {}
        self.command_throttle_interval = 0.1  # seconds

        # Initialize dictionaries to store body part control interfaces
        self.body_parts_frames = {}
        self.motor_widgets = {}
        self.ros_interfaces = {}

        # Create main frame and body part selection tabs
        self.main_notebook = ttk.Notebook(self)
        self.main_notebook.pack(fill="both", expand=True, padx=10, pady=10)

        # Create tabs and controllers for each body part
        for body_part, topics in BODY_PARTS_TOPICS.items():
            self.create_body_part_controller(body_part, topics)

        # Ensure all tabs are visible
        self.main_notebook.enable_traversal()

        # Log output
        self.node.get_logger().info(f"Created GUI with tabs for: {', '.join(BODY_PARTS_TOPICS.keys())}")

    def setup_signal_handler(self):
        """Set up signal handler to capture Ctrl+C"""

        def signal_handler(sig, frame):
            self.node.get_logger().info("Ctrl+C detected, shutting down...")
            self.quit()  # Stop Tkinter main loop

        # Register SIGINT (Ctrl+C) signal handler
        signal.signal(signal.SIGINT, signal_handler)

        # Let Tkinter check signals - every 100ms
        self.after(100, self.check_signal)

    def check_signal(self):
        """Periodically check signals, allowing signal handlers to work in Tkinter main loop"""
        # Reschedule next check
        self.after(100, self.check_signal)

    def create_body_part_controller(self, body_part, topics):
        """Create controller for body part

        Args:
            body_part: Body part name
            topics: Topic configuration for the body part
        """
        # Create tab for the body part - using white background
        body_part_frame = ttk.Frame(self.main_notebook, style='TFrame')
        self.main_notebook.add(body_part_frame, text=body_part.capitalize())
        self.body_parts_frames[body_part] = body_part_frame

        # Initialize motor widgets dictionary for this body part
        self.motor_widgets[body_part] = {}

        # Create ROS interface
        ros_interface = RosInterface(
            self.node,
            body_part,
            topics['cmd_topic'],
            topics['status_topic'],
            lambda msg: self.status_callback(body_part, msg)
        )
        self.ros_interfaces[body_part] = ros_interface

        # Create scrollable frame to handle many motors
        # Using Frame instead of Canvas to avoid potential scrolling issues
        main_frame = ttk.Frame(body_part_frame, style='TFrame')
        main_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Create reset all buttons frame
        reset_all_frame = ttk.Frame(main_frame, style='TFrame')
        reset_all_frame.pack(fill="x", padx=10, pady=5)

        # Add reset all button - using larger button
        reset_all_button = ttk.Button(
            reset_all_frame,
            text=f"Reset All {body_part.capitalize()} Joints to Zero",
            command=lambda bp=body_part: self.reset_all_joints(bp),
            style='TButton'
        )
        reset_all_button.pack(pady=5)

        # Create header row
        header_frame = ttk.Frame(main_frame, style='TFrame')
        header_frame.pack(fill="x", padx=10, pady=(5, 0))

        # Increase header row font size
        ttk.Label(header_frame, text="Motor Name", width=18, style='TLabel').grid(row=0, column=0, padx=5)
        ttk.Label(header_frame, text="Current Pos", width=10, style='TLabel').grid(row=0, column=1, padx=5)
        ttk.Label(header_frame, text="Position Control", width=55, style='TLabel').grid(row=0, column=2, padx=5)  # Increase width
        ttk.Label(header_frame, text="Target", width=8, style='TLabel').grid(row=0, column=3, padx=5)
        ttk.Label(header_frame, text="Velocity", width=6, style='TLabel').grid(row=0, column=4, padx=5)
        ttk.Label(header_frame, text="Current", width=6, style='TLabel').grid(row=0, column=5, padx=5)
        ttk.Label(header_frame, text="Reset", width=6, style='TLabel').grid(row=0, column=6, padx=5)

        # Create motor control rows frame
        controls_frame = ttk.Frame(main_frame, style='TFrame')
        controls_frame.pack(fill="both", expand=True, padx=5, pady=5)

        # Create control rows for each motor
        motors_config = topics['motors']

        # Sort motor IDs to ensure consistent order
        sorted_motor_ids = sorted(motors_config.keys())

        for motor_id in sorted_motor_ids:
            config = motors_config[motor_id]

            # Create independent send command functions for each motor
            def make_send_command(mid, bp):
                def send_cmd():
                    self.send_command(bp, mid)

                return send_cmd

            def make_throttled_send_command(mid, bp):
                def throttled_send_cmd():
                    self.throttled_send_command(bp, mid)

                return throttled_send_cmd

            # Create callbacks dictionary - using closures instead of lambda
            callbacks = {
                'send_command': lambda mid=motor_id, bp=body_part: self.send_command(bp, mid),
                'throttled_send_command': lambda mid=motor_id, bp=body_part: self.throttled_send_command(bp, mid),
            }

            # Create motor control row
            motor_row = MotorRow(controls_frame, motor_id, config, callbacks)
            self.motor_widgets[body_part][motor_id] = motor_row

        # Log output
        self.node.get_logger().info(f"Created {len(motors_config)} motor controls for {body_part}")

    def send_command(self, body_part, motor_id):
        """Send control command for specified motor"""
        if body_part not in self.motor_widgets or motor_id not in self.motor_widgets[body_part]:
            return

        motor_row = self.motor_widgets[body_part][motor_id]
        values = motor_row.get_command_values()

        # Send command through ROS interface
        self.ros_interfaces[body_part].send_motor_command(
            motor_id,
            values['pos'],
            values['vel'],
            values['cur']
        )

    def throttled_send_command(self, body_part, motor_id):
        """Throttled command sending"""
        current_time = time.time()

        # Check if throttle interval has passed
        key = f"{body_part}_{motor_id}"
        last_time = self.last_command_time.get(key, 0)
        if current_time - last_time >= self.command_throttle_interval:
            self.send_command(body_part, motor_id)
            self.last_command_time[key] = current_time

    def reset_joint(self, body_part, motor_id):
        """Reset specified motor to zero"""
        if body_part in self.motor_widgets and motor_id in self.motor_widgets[body_part]:
            self.motor_widgets[body_part][motor_id].reset_joint()
            self.node.get_logger().info(f"Reset joint {body_part}.{motor_id} to zero position")

    def reset_all_joints(self, body_part):
        """Reset all motors of specified body part to zero"""
        if body_part in self.motor_widgets:
            for motor_id in self.motor_widgets[body_part]:
                self.reset_joint(body_part, motor_id)

            self.node.get_logger().info(f"All {body_part} joints have been reset to zero positions")

    def status_callback(self, body_part, msg):
        """Update GUI when status message is received"""
        try:
            # Process received motor status
            for motor in msg.status:
                motor_id = motor.name

                # If we have a widget for this motor, update the display
                if body_part in self.motor_widgets and motor_id in self.motor_widgets[body_part]:
                    self.motor_widgets[body_part][motor_id].update_position_display(motor.pos)
        except Exception as e:
            self.node.get_logger().error(f"Error processing status message for {body_part}: {str(e)}")

    def update_ros(self):
        """Process ROS messages"""
        import rclpy
        rclpy.spin_once(self.node, timeout_sec=0.01)
        self.after(10, self.update_ros)