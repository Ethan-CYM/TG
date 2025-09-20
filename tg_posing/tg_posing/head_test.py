#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from bodyctrl_msgs.msg import CmdSetMotorPosition, SetMotorPosition
from bodyctrl_msgs.msg import MotorStatusMsg

import tkinter as tk
from tkinter import ttk
import signal
import sys

# 头部电机配置
HEAD_MOTORS = {
    1: {"name": "Head Roll", "min": -0.45, "max": 0.45, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 1.5, "zero_pos": 0.0},
    2: {"name": "Head Pitch", "min": -0.43, "max": 0.43, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 1.5, "zero_pos": 0.0},
    3: {"name": "Head Yaw", "min": -1.57, "max": 1.57, "velocity": 0.15, "max_velocity": 0.3, "max_cur": 4.0,
        "cur": 1.5, "zero_pos": 0.0}
}




class HeadJointGUI(Node):
    def __init__(self):
        super().__init__('head_joint_gui')

        # 发布器：发送控制命令
        self.pub = self.create_publisher(CmdSetMotorPosition, '/head/cmd_pos', 10)

        # 订阅器：接收头部状态
        self.sub = self.create_subscription(
            MotorStatusMsg,
            '/head/status',
            self.status_callback,
            10
        )

        # 初始化 GUI
        self.root = tk.Tk()
        self.root.title("Head Joint Controller")

        # 设置Ctrl+C处理器
        self.setup_signal_handler()

        # 添加命令节流变量
        self.last_command_time = {}
        self.command_throttle_interval = 0.1  # 秒

        # 创建总归零按钮框架
        reset_all_frame = ttk.Frame(self.root)
        reset_all_frame.pack(fill="x", padx=10, pady=5)

        # 添加总归零按钮
        reset_all_button = ttk.Button(reset_all_frame, text="Reset All Joints to Zero", command=self.reset_all_joints)
        reset_all_button.pack(pady=5)

        # 创建标题行
        header_frame = ttk.Frame(self.root)
        header_frame.pack(fill="x", padx=10, pady=(5, 0))

        ttk.Label(header_frame, text="Motor Name", width=15).grid(row=0, column=0, padx=5)
        ttk.Label(header_frame, text="Current Pos", width=10).grid(row=0, column=1, padx=5)
        ttk.Label(header_frame, text="Position Control", width=40).grid(row=0, column=2, padx=5)
        ttk.Label(header_frame, text="Target Pos", width=10).grid(row=0, column=3, padx=5)
        ttk.Label(header_frame, text="Velocity Limit", width=10).grid(row=0, column=4, padx=5)
        ttk.Label(header_frame, text="Current Limit", width=10).grid(row=0, column=5, padx=5)
        ttk.Label(header_frame, text="Reset", width=10).grid(row=0, column=6, padx=5)

        # 为每个电机创建控制行
        self.motor_widgets = {}

        for motor_id, config in HEAD_MOTORS.items():
            self.create_motor_row(motor_id, config)

        # 定时器：每隔 100ms 更新 GUI 和 ROS
        self.root.after(100, self.tk_loop)

        # 存储最后接收的电机状态
        self.motor_status = {}

    def setup_signal_handler(self):
        """设置信号处理器以捕获Ctrl+C"""

        def signal_handler(sig, frame):
            self.get_logger().info("Ctrl+C detected, shutting down...")
            if self.root:
                self.root.quit()  # 停止Tkinter主循环

        # 注册SIGINT (Ctrl+C)信号处理器
        signal.signal(signal.SIGINT, signal_handler)

        # 让Tkinter检查信号 - 每100ms检查一次
        self.root.after(100, self.check_signal)

    def check_signal(self):
        """定期检查信号，允许信号处理器在Tkinter主循环中工作"""
        # 重新安排下一次检查
        self.root.after(100, self.check_signal)

    def create_motor_row(self, motor_id, config):
        """为一个电机创建控制行"""
        row_frame = ttk.Frame(self.root)
        row_frame.pack(fill="x", padx=10, pady=5)

        # 电机名称
        ttk.Label(row_frame, text=config['name'], width=15).grid(row=0, column=0, padx=5)

        # 当前位置
        position_label = ttk.Label(row_frame, text="--- rad", width=10)
        position_label.grid(row=0, column=1, padx=5)

        # 位置控制滑块 (Canvas)
        canvas_frame = ttk.Frame(row_frame)
        canvas_frame.grid(row=0, column=2, padx=5)

        canvas = tk.Canvas(canvas_frame, height=30, width=300, bg='white')
        canvas.pack()

        # 在画布上绘制滑轨背景
        slider_bg = canvas.create_rectangle(10, 10, 290, 20, fill='lightgray')

        # 实际位置指示器（来自传感器）- 红色圆形
        actual_indicator = canvas.create_oval(5, 5, 15, 25, fill='red', outline='darkred')

        # 目标位置指示器（用户拖动）- 蓝色矩形
        target_indicator = canvas.create_rectangle(5, 0, 15, 30, fill='blue')

        # 创建变量以存储目标位置和初始位置标志
        target_var = tk.DoubleVar(value=0.0)

        # 标记是否已接收到初始位置
        initial_pos_received = False

        # 为画布添加拖动事件处理
        canvas.bind("<Button-1>", lambda event, id=motor_id: self.on_canvas_click(event, id))
        canvas.bind("<B1-Motion>", lambda event, id=motor_id: self.on_canvas_drag(event, id))
        canvas.bind("<ButtonRelease-1>", lambda event, id=motor_id: self.on_canvas_release(event, id))

        # 目标位置输入框
        target_entry = ttk.Entry(row_frame, width=10, textvariable=target_var)
        target_entry.grid(row=0, column=3, padx=5)
        target_entry.bind("<Return>", lambda event, id=motor_id: self.on_target_entry(event, id))

        # 速度限制输入框
        velocity_var = tk.DoubleVar(value=config['velocity'])
        velocity_entry = ttk.Entry(row_frame, width=10, textvariable=velocity_var)
        velocity_entry.grid(row=0, column=4, padx=5)
        velocity_entry.bind("<Return>", lambda event, id=motor_id: self.on_velocity_entry(event, id))
        velocity_entry.bind("<FocusOut>", lambda event, id=motor_id: self.on_velocity_entry(event, id))

        # 电流限制输入框
        current_var = tk.DoubleVar(value=config['cur'])
        current_entry = ttk.Entry(row_frame, width=10, textvariable=current_var)
        current_entry.grid(row=0, column=5, padx=5)
        current_entry.bind("<Return>", lambda event, id=motor_id: self.on_current_entry(event, id))
        current_entry.bind("<FocusOut>", lambda event, id=motor_id: self.on_current_entry(event, id))

        # 归零按钮
        reset_button = ttk.Button(row_frame, text="Reset", width=8,
                                  command=lambda id=motor_id: self.reset_joint(id))
        reset_button.grid(row=0, column=6, padx=5)

        # 将控件保存到字典中
        self.motor_widgets[motor_id] = {
            "position_label": position_label,
            "canvas": canvas,
            "actual_indicator": actual_indicator,
            "target_indicator": target_indicator,
            "target_var": target_var,
            "target_entry": target_entry,
            "velocity_var": velocity_var,
            "velocity_entry": velocity_entry,
            "current_var": current_var,
            "current_entry": current_entry,
            "reset_button": reset_button,
            "min_val": config['min'],
            "max_val": config['max'],
            "max_velocity": config['max_velocity'],
            "max_cur": config['max_cur'],
            "zero_pos": config['zero_pos'],
            "initial_pos_received": initial_pos_received
        }

    def reset_joint(self, motor_id):
        """将指定电机归零"""
        if motor_id not in self.motor_widgets:
            return

        widgets = self.motor_widgets[motor_id]
        zero_pos = widgets.get("zero_pos", 0.0)

        # 更新目标位置变量和显示
        widgets["target_var"].set(zero_pos)
        widgets["target_entry"].delete(0, tk.END)
        widgets["target_entry"].insert(0, f"{zero_pos:.3f}")

        # 更新目标指示器位置
        self.update_target_indicator(motor_id, zero_pos)

        # 发送命令
        self.send_command(motor_id)

        self.get_logger().info(f"Reset joint {motor_id} to zero position: {zero_pos:.3f}")

    def reset_all_joints(self):
        """将所有电机归零"""
        for motor_id in self.motor_widgets:
            self.reset_joint(motor_id)

        self.get_logger().info("All joints have been reset to zero positions")

    def on_velocity_entry(self, event, motor_id):
        """处理速度限制输入框更改事件"""
        widgets = self.motor_widgets[motor_id]
        max_velocity = widgets["max_velocity"]

        try:
            # 获取并限制输入值
            velocity = float(widgets["velocity_entry"].get())
            velocity = max(0.01, min(velocity, max_velocity))  # 限制最小0.01，最大为电机配置的最大速度

            # 以0.01精度舍入
            velocity = round(velocity, 2)

            widgets["velocity_var"].set(velocity)

            # 更新显示
            widgets["velocity_entry"].delete(0, tk.END)
            widgets["velocity_entry"].insert(0, f"{velocity:.2f}")

        except ValueError:
            # 输入不是有效数字，恢复原值
            widgets["velocity_entry"].delete(0, tk.END)
            widgets["velocity_entry"].insert(0, f"{widgets['velocity_var'].get():.2f}")

    def on_current_entry(self, event, motor_id):
        """处理电流限制输入框更改事件"""
        widgets = self.motor_widgets[motor_id]
        max_cur = widgets["max_cur"]

        try:
            # 获取并限制输入值
            current = float(widgets["current_entry"].get())
            current = max(0.1, min(current, max_cur))  # 限制最小0.1，最大为电机配置的最大电流

            # 以0.1精度舍入
            current = round(current, 1)

            widgets["current_var"].set(current)

            # 更新显示
            widgets["current_entry"].delete(0, tk.END)
            widgets["current_entry"].insert(0, f"{current:.1f}")

        except ValueError:
            # 输入不是有效数字，恢复原值
            widgets["current_entry"].delete(0, tk.END)
            widgets["current_entry"].insert(0, f"{widgets['current_var'].get():.1f}")

    def on_canvas_click(self, event, motor_id):
        """处理画布点击事件"""
        widgets = self.motor_widgets[motor_id]
        canvas = widgets["canvas"]
        min_val = widgets["min_val"]
        max_val = widgets["max_val"]

        # 计算点击位置对应的目标值 - 使用0.001精度
        x_rel = max(10, min(event.x, 290))
        normalized = (x_rel - 10) / 280.0
        target = min_val + normalized * (max_val - min_val)

        # 以0.001弧度精度舍入
        target = round(target, 3)

        # 更新目标变量和输入框
        widgets["target_var"].set(target)
        widgets["target_entry"].delete(0, tk.END)
        widgets["target_entry"].insert(0, f"{target:.3f}")
        self.update_target_indicator(motor_id, target)

        # 节流发送命令
        self.throttled_send_command(motor_id)

    def on_canvas_drag(self, event, motor_id):
        """处理画布拖动事件 - 实时发送命令但有节流"""
        self.on_canvas_click(event, motor_id)  # 复用点击处理逻辑

    def on_canvas_release(self, event, motor_id):
        """处理鼠标释放事件 - 无论何时都要发送最终位置"""
        widgets = self.motor_widgets[motor_id]
        canvas = widgets["canvas"]
        min_val = widgets["min_val"]
        max_val = widgets["max_val"]

        # 计算点击位置对应的目标值 - 使用0.001精度
        x_rel = max(10, min(event.x, 290))
        normalized = (x_rel - 10) / 280.0
        target = min_val + normalized * (max_val - min_val)

        # 以0.001弧度精度舍入
        target = round(target, 3)

        # 更新目标变量和输入框
        widgets["target_var"].set(target)
        widgets["target_entry"].delete(0, tk.END)
        widgets["target_entry"].insert(0, f"{target:.3f}")
        self.update_target_indicator(motor_id, target)

        # 鼠标释放时总是发送命令，不受节流限制
        self.send_command(motor_id)

    def on_target_entry(self, event, motor_id):
        """处理目标位置输入框更改事件"""
        widgets = self.motor_widgets[motor_id]
        min_val = widgets["min_val"]
        max_val = widgets["max_val"]

        try:
            # 获取并限制输入值
            target = float(widgets["target_entry"].get())
            target = max(min_val, min(target, max_val))

            # 以0.001弧度精度舍入
            target = round(target, 3)

            widgets["target_var"].set(target)

            # 更新目标指示器位置
            self.update_target_indicator(motor_id, target)

            # 自动发送命令（输入框Enter事件不受节流限制）
            self.send_command(motor_id)
        except ValueError:
            # 输入不是有效数字，恢复原值
            widgets["target_entry"].delete(0, tk.END)
            widgets["target_entry"].insert(0, f"{widgets['target_var'].get():.3f}")

    def send_command(self, motor_id):
        """为指定电机发送控制命令"""
        if motor_id not in self.motor_widgets:
            return

        widgets = self.motor_widgets[motor_id]

        try:
            # 获取目标位置、速度和电流
            target_pos = widgets["target_var"].get()
            velocity_limit = widgets["velocity_var"].get()
            current_limit = widgets["current_var"].get()

            # 构造命令消息
            cmd = CmdSetMotorPosition()
            cmd.header = Header()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = "head"

            motor = SetMotorPosition()
            motor.name = motor_id
            motor.pos = target_pos
            motor.spd = velocity_limit  # 保持使用spd字段，因为消息定义中使用的是spd
            motor.cur = current_limit

            cmd.cmds = [motor]
            self.pub.publish(cmd)

            self.get_logger().info(
                f"Publishing motor command: id={motor.name}, pos={motor.pos:.3f}, vel={motor.spd:.2f}, cur={motor.cur:.1f}")

        except Exception as e:
            self.get_logger().error(f"Error sending command: {str(e)}")

    def throttled_send_command(self, motor_id):
        """节流控制的命令发送"""
        current_time = self.get_clock().now().seconds_nanoseconds()
        current_seconds = current_time[0] + current_time[1] / 1e9

        # 检查是否已经过了节流间隔
        last_time = self.last_command_time.get(motor_id, 0)
        if current_seconds - last_time >= self.command_throttle_interval:
            self.send_command(motor_id)
            self.last_command_time[motor_id] = current_seconds

    def update_target_indicator(self, motor_id, target_position):
        """更新目标位置指示器"""
        if motor_id not in self.motor_widgets:
            return

        widgets = self.motor_widgets[motor_id]
        canvas = widgets["canvas"]
        min_val = widgets["min_val"]
        max_val = widgets["max_val"]

        # 确保位置在范围内
        position = max(min_val, min(target_position, max_val))

        # 计算画布位置
        normalized = (position - min_val) / (max_val - min_val)
        x_pos = 10 + normalized * 280

        # 更新目标位置指示器
        canvas.coords(widgets["target_indicator"], x_pos - 5, 0, x_pos + 5, 30)

    def update_actual_position(self, motor_id, position):
        """更新实际位置指示器"""
        if motor_id not in self.motor_widgets:
            return

        widgets = self.motor_widgets[motor_id]
        canvas = widgets["canvas"]
        min_val = widgets["min_val"]
        max_val = widgets["max_val"]

        # 确保位置在范围内
        position = max(min_val, min(position, max_val))

        # 计算画布位置
        normalized = (position - min_val) / (max_val - min_val)
        x_pos = 10 + normalized * 280

        # 更新实际位置指示器
        canvas.coords(widgets["actual_indicator"], x_pos - 5, 5, x_pos + 5, 25)

    def status_callback(self, msg):
        """接收到头部状态时更新 GUI"""
        try:
            # 处理接收到的电机状态
            for motor in msg.status:
                motor_id = motor.name

                # 如果有这个电机的控件，更新显示
                if motor_id in self.motor_widgets:
                    widgets = self.motor_widgets[motor_id]

                    # 更新位置标签 - 使用3位小数
                    widgets["position_label"].config(text=f"{motor.pos:.3f} rad")

                    # 更新实际位置指示器
                    self.update_actual_position(motor_id, motor.pos)

                    # 如果这是首次接收到该电机的位置，则使目标指示器与实际位置重合
                    if not widgets.get("initial_pos_received", False):
                        widgets["target_var"].set(round(motor.pos, 3))
                        widgets["target_entry"].delete(0, tk.END)
                        widgets["target_entry"].insert(0, f"{round(motor.pos, 3):.3f}")
                        self.update_target_indicator(motor_id, motor.pos)
                        widgets["initial_pos_received"] = True

                    # 记录最新的电机状态
                    self.motor_status[motor_id] = motor
        except Exception as e:
            self.get_logger().error(f"Error processing status message: {str(e)}")

    def tk_loop(self):
        """Tkinter 主循环 + ROS spin 一起运行"""
        rclpy.spin_once(self, timeout_sec=0.01)
        self.root.after(10, self.tk_loop)


def main(args=None):
    rclpy.init(args=args)
    gui = HeadJointGUI()

    try:
        gui.root.mainloop()
    except KeyboardInterrupt:
        # 如果发生KeyboardInterrupt，确保关闭窗口
        gui.get_logger().info("KeyboardInterrupt detected, shutting down...")
        if gui.root:
            gui.root.quit()
    finally:
        # 无论如何都清理资源
        gui.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()