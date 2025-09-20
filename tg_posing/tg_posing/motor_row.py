#!/usr/bin/env python3

import tkinter as tk
from tkinter import ttk

class MotorRow:
    """Represents a row of motor control in the GUI"""

    def __init__(self, parent_frame, motor_id, config, callbacks):
        """Initialize motor control row

        Args:
            parent_frame: Parent frame
            motor_id: Motor ID
            config: Motor configuration
            callbacks: Callbacks dictionary
        """
        self.motor_id = motor_id
        self.config = config
        self.callbacks = callbacks

        # Create row frame
        self.row_frame = ttk.Frame(parent_frame)
        self.row_frame.pack(fill="x", padx=15, pady=3)  # Reduce vertical spacing, increase horizontal spacing

        # Motor name - increase width
        ttk.Label(self.row_frame, text=config['name'], width=18).grid(row=0, column=0, padx=5)

        # Current position
        self.position_label = ttk.Label(self.row_frame, text="--- rad", width=10)
        self.position_label.grid(row=0, column=1, padx=5)

        # Position control slider (Canvas) - increase width
        canvas_frame = ttk.Frame(self.row_frame)
        canvas_frame.grid(row=0, column=2, padx=5)

        self.canvas = tk.Canvas(canvas_frame, height=26, width=400, bg='white')  # Increase width, reduce height
        self.canvas.pack()

        # Draw slider track background on canvas
        self.slider_bg = self.canvas.create_rectangle(10, 10, 390, 20, fill='lightgray')  # Adjust slider width

        # Actual position indicator (from sensor) - red circle
        self.actual_indicator = self.canvas.create_oval(5, 5, 15, 25, fill='red', outline='darkred')

        # Target position indicator (user drag) - blue rectangle
        self.target_indicator = self.canvas.create_rectangle(5, 0, 15, 26, fill='blue')  # Adjust height

        # Create variable to store target position
        self.target_var = tk.DoubleVar(value=0.0)

        # Flag whether initial position has been received
        self.initial_pos_received = False

        # Add drag event handling for canvas
        self.canvas.bind("<Button-1>", self._on_canvas_click)
        self.canvas.bind("<B1-Motion>", self._on_canvas_drag)
        self.canvas.bind("<ButtonRelease-1>", self._on_canvas_release)

        # Target position entry
        self.target_entry = ttk.Entry(self.row_frame, width=8)  # Reduce width
        self.target_entry.grid(row=0, column=3, padx=5)
        self.target_entry.bind("<Return>", self._on_target_entry)

        # Velocity limit entry
        self.velocity_var = tk.DoubleVar(value=config['velocity'])
        self.velocity_entry = ttk.Entry(self.row_frame, width=6)  # Reduce width
        self.velocity_entry.grid(row=0, column=4, padx=5)
        self.velocity_entry.bind("<Return>", self._on_velocity_entry)
        self.velocity_entry.bind("<FocusOut>", self._on_velocity_entry)
        self.velocity_entry.insert(0, f"{config['velocity']:.2f}")

        # Current limit entry
        self.current_var = tk.DoubleVar(value=config['cur'])
        self.current_entry = ttk.Entry(self.row_frame, width=6)  # Reduce width
        self.current_entry.grid(row=0, column=5, padx=5)
        self.current_entry.bind("<Return>", self._on_current_entry)
        self.current_entry.bind("<FocusOut>", self._on_current_entry)
        self.current_entry.insert(0, f"{config['cur']:.1f}")

        # Reset button
        self.reset_button = ttk.Button(self.row_frame, text="Reset", width=6,
                                       command=self.reset_joint)
        self.reset_button.grid(row=0, column=6, padx=5)

    def _on_canvas_click(self, event):
        """Handle canvas click event"""
        min_val = self.config['min']
        max_val = self.config['max']

        # Calculate target value corresponding to click position - using 0.001 precision
        x_rel = max(10, min(event.x, 390))  # Adjust to new slider width
        normalized = (x_rel - 10) / 380.0    # Adjust to new slider width
        target = min_val + normalized * (max_val - min_val)

        # Round to 0.001 radian precision
        target = round(target, 3)

        # Update target variable and entry
        self.target_var.set(target)
        self.target_entry.delete(0, tk.END)
        self.target_entry.insert(0, f"{target:.3f}")
        self.update_target_indicator(target)

        # Call throttled send command callback
        if self.callbacks.get('throttled_send_command'):
            self.callbacks['throttled_send_command'](self.motor_id)

    def _on_canvas_drag(self, event):
        """Handle canvas drag event - send commands in real-time but throttled"""
        self._on_canvas_click(event)  # Reuse click handling logic

    def _on_canvas_release(self, event):
        """Handle mouse release event - always send final position"""
        min_val = self.config['min']
        max_val = self.config['max']

        # Calculate target value corresponding to click position - using 0.001 precision
        x_rel = max(10, min(event.x, 390))  # Adjust to new slider width
        normalized = (x_rel - 10) / 380.0    # Adjust to new slider width
        target = min_val + normalized * (max_val - min_val)

        # Round to 0.001 radian precision
        target = round(target, 3)

        # Update target variable and entry
        self.target_var.set(target)
        self.target_entry.delete(0, tk.END)
        self.target_entry.insert(0, f"{target:.3f}")
        self.update_target_indicator(target)

        # Call send command callback
        if self.callbacks.get('send_command'):
            self.callbacks['send_command'](self.motor_id)

    def _on_target_entry(self, event):
        """Handle target position entry change event"""
        min_val = self.config['min']
        max_val = self.config['max']

        try:
            # Get and limit input value
            target = float(self.target_entry.get())
            target = max(min_val, min(target, max_val))

            # Round to 0.001 radian precision
            target = round(target, 3)

            self.target_var.set(target)

            # Update target indicator position
            self.update_target_indicator(target)

            # Call send command callback
            if self.callbacks.get('send_command'):
                self.callbacks['send_command'](self.motor_id)
        except ValueError:
            # Input is not a valid number, restore original value
            self.target_entry.delete(0, tk.END)
            self.target_entry.insert(0, f"{self.target_var.get():.3f}")

    def _on_velocity_entry(self, event):
        """Handle velocity limit entry change event"""
        max_velocity = self.config['max_velocity']

        try:
            # Get and limit input value
            velocity = float(self.velocity_entry.get())
            velocity = max(0.01, min(velocity, max_velocity))  # Limit min 0.01, max to motor config max velocity

            # Round to 0.01 precision
            velocity = round(velocity, 2)

            self.velocity_var.set(velocity)

            # Update display
            self.velocity_entry.delete(0, tk.END)
            self.velocity_entry.insert(0, f"{velocity:.2f}")

        except ValueError:
            # Input is not a valid number, restore original value
            self.velocity_entry.delete(0, tk.END)
            self.velocity_entry.insert(0, f"{self.velocity_var.get():.2f}")

    def _on_current_entry(self, event):
        """Handle current limit entry change event"""
        max_cur = self.config['max_cur']

        try:
            # Get and limit input value
            current = float(self.current_entry.get())
            current = max(0.1, min(current, max_cur))  # Limit min 0.1, max to motor config max current

            # Round to 0.1 precision
            current = round(current, 1)

            self.current_var.set(current)

            # Update display
            self.current_entry.delete(0, tk.END)
            self.current_entry.insert(0, f"{current:.1f}")

        except ValueError:
            # Input is not a valid number, restore original value
            self.current_entry.delete(0, tk.END)
            self.current_entry.insert(0, f"{self.current_var.get():.1f}")

    def reset_joint(self):
        """Reset motor to zero position"""
        zero_pos = self.config.get('zero_pos', 0.0)

        # Update target position variable and display
        self.target_var.set(zero_pos)
        self.target_entry.delete(0, tk.END)
        self.target_entry.insert(0, f"{zero_pos:.3f}")

        # Update target indicator position
        self.update_target_indicator(zero_pos)

        # Call send command callback
        if self.callbacks.get('send_command'):
            self.callbacks['send_command'](self.motor_id)

    def update_target_indicator(self, target_position):
        """Update target position indicator"""
        min_val = self.config['min']
        max_val = self.config['max']

        # Ensure position is within range
        position = max(min_val, min(target_position, max_val))

        # Calculate canvas position
        normalized = (position - min_val) / (max_val - min_val)
        x_pos = 10 + normalized * 380  # Adjust to new slider width

        # Update target position indicator
        self.canvas.coords(self.target_indicator, x_pos - 5, 0, x_pos + 5, 26)  # Adjust height

    def update_actual_position(self, position):
        """Update actual position indicator"""
        min_val = self.config['min']
        max_val = self.config['max']

        # Ensure position is within range
        position = max(min_val, min(position, max_val))

        # Calculate canvas position
        normalized = (position - min_val) / (max_val - min_val)
        x_pos = 10 + normalized * 380  # Adjust to new slider width

        # Update actual position indicator
        self.canvas.coords(self.actual_indicator, x_pos - 5, 5, x_pos + 5, 25)

    def update_position_display(self, position):
        """Update position display"""
        # Update position label - using 3 decimal places
        self.position_label.config(text=f"{position:.3f} rad")

        # Update actual position indicator
        self.update_actual_position(position)

        # If this is the first position received for this motor, align target indicator with actual position
        if not self.initial_pos_received:
            rounded_pos = round(position, 3)
            self.target_var.set(rounded_pos)
            self.target_entry.delete(0, tk.END)
            self.target_entry.insert(0, f"{rounded_pos:.3f}")
            self.update_target_indicator(position)
            self.initial_pos_received = True

    def get_command_values(self):
        """Get current control values"""
        return {
            "pos": self.target_var.get(),
            "vel": self.velocity_var.get(),
            "cur": self.current_var.get()
        }