# turtlebot_full_gui.py
#!/usr/bin/env python3
"""
Integrated TurtleBot4 GUI:
* Start/Stop control with real‑time IMU and Odometry display
* Key‑based navigation:
  - **A** →  (10, 10)  face +x (yaw = 0)
  - **B** → (-10,-10) face –x (yaw = π)
  - **Origin** → (0, 0) face +x

ROS Interfaces
--------------
Publishers
  /turtlebot_state    std_msgs/String          (textual state)
  /gui_cmd_vel        geometry_msgs/Twist      (manual velocity command)
  /simple_goal        geometry_msgs/PoseStamped (navigation goal)

Subscribers
  /default_vel        std_msgs/Float32MultiArray  (default velocities)
  /c3_14/odom         nav_msgs/Odometry            (odometry)
  /c3_14/imu          sensor_msgs/Imu              (IMU data)
"""
import sys, signal, math, subprocess
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

from PyQt5 import QtWidgets, QtCore

from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

class TurtleBotGUI(Node, QtWidgets.QMainWindow):
    def __init__(self):
        Node.__init__(self, 'turtlebot_gui')
        QtWidgets.QMainWindow.__init__(self)

        # ---------------- ROS Publishers / Subscribers ----------------
        self.pub_state  = self.create_publisher(String, '/turtlebot_state', 10)
        self.pub_twist  = self.create_publisher(Twist, '/gui_cmd_vel', 10)
        self.pub_goal   = self.create_publisher(PoseStamped, '/simple_goal', 10)  # 新增：導航目標

        self.create_subscription(Float32MultiArray, '/default_vel', self.update_defaults, 10)
        self.create_subscription(Odometry, '/c3_14/odom', self.odom_callback, qos_profile_sensor_data)
        self.create_subscription(Imu, '/c3_14/imu', self.imu_callback, qos_profile_sensor_data)

        # ---------------- GUI base settings ----------------
        self.setWindowTitle('TurtleBot4 GUI')
        self.resize(1000, 600)

        self.current_action = None  # A / B Actions

        # ---------------- layout ----------------
        central_widget = QtWidgets.QWidget(); self.setCentralWidget(central_widget)
        main_layout = QtWidgets.QGridLayout(); central_widget.setLayout(main_layout)

        # --- Logging panel ---
        self.log_text = QtWidgets.QTextEdit(); self.log_text.setReadOnly(True)
        main_layout.addWidget(QtWidgets.QLabel('Logging'), 0, 0)
        main_layout.addWidget(self.log_text, 1, 0, 1, 1)

        # --- IMU Plot ---
        self.imu_data = deque(maxlen=2000)
        self.figure = Figure(figsize=(8, 6), dpi=100)
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_title('Linear Acceleration (ax vs time)')
        self.ax.set_xlabel('Time (s)'); self.ax.set_ylabel('ax (m/s²)')
        self.raw_line, = self.ax.plot([], [], label='raw ax', color='blue', alpha=0.3)
        self.filtered_line, = self.ax.plot([], [], label='filtered ax', color='red', linewidth=2)
        self.ax.legend(loc='upper right')
        main_layout.addWidget(self.canvas, 2, 0, 1, 1)

        # --- Velocity display ---
        self.current_ang = QtWidgets.QLineEdit(); self.current_lin = QtWidgets.QLineEdit()
        self.current_ang.setReadOnly(True); self.current_lin.setReadOnly(True)
        main_layout.addWidget(QtWidgets.QLabel('Linear Velocity'), 3, 0)
        main_layout.addWidget(self.current_lin, 4, 0)
        main_layout.addWidget(QtWidgets.QLabel('Angular Velocity'), 5, 0)
        main_layout.addWidget(self.current_ang, 6, 0)

        # --- Velocity control panel ---
        ctrl_box = QtWidgets.QGroupBox(); ctrl_layout = QtWidgets.QVBoxLayout(); ctrl_box.setLayout(ctrl_layout)
        self.cmd_ang = QtWidgets.QDoubleSpinBox(); self.cmd_lin = QtWidgets.QDoubleSpinBox()
        self.cmd_ang.setSingleStep(0.05); self.cmd_lin.setSingleStep(0.05)
        self.cmd_ang.setRange(-5.0, 5.0); self.cmd_lin.setRange(-5.0, 5.0)
        ctrl_layout.addWidget(QtWidgets.QLabel('Linear Velocity'));  ctrl_layout.addWidget(self.cmd_lin)
        ctrl_layout.addWidget(QtWidgets.QLabel('Angular Velocity')); ctrl_layout.addWidget(self.cmd_ang)
        btn_layout = QtWidgets.QHBoxLayout()
        self.btn_reset = QtWidgets.QPushButton('Reset'); self.btn_send = QtWidgets.QPushButton('Send')
        btn_layout.addWidget(self.btn_reset); btn_layout.addWidget(self.btn_send)
        ctrl_layout.addLayout(btn_layout)
        main_layout.addWidget(ctrl_box, 0, 2, 2, 1)

        # --- START / STOP button ---
        self.btn_start = QtWidgets.QPushButton('START'); self.btn_start.setFixedSize(120, 120)
        self.btn_start.setStyleSheet(self.get_start_button_style())
        main_layout.addWidget(self.btn_start, 2, 2, alignment=QtCore.Qt.AlignHCenter)

        # --- A / B / Origin buttons ---
        ab_layout = QtWidgets.QHBoxLayout()
        self.btn_a = QtWidgets.QPushButton('A'); self.btn_b = QtWidgets.QPushButton('B'); self.btn_origin = QtWidgets.QPushButton('Origin')
        ab_layout.addWidget(self.btn_a); ab_layout.addWidget(self.btn_b); ab_layout.addWidget(self.btn_origin)
        main_layout.addLayout(ab_layout, 3, 2)

        # ---------------- Clicking signal connections ----------------
        self.btn_send.clicked.connect(self.send_cmd_input)
        self.btn_reset.clicked.connect(self.reset_inputs)
        self.btn_start.clicked.connect(self.toggle_start_stop)
        self.btn_a.clicked.connect(lambda: self.handle_action_button('A'))
        self.btn_b.clicked.connect(lambda: self.handle_action_button('B'))
        self.btn_origin.clicked.connect(self.handle_origin_button)

        # ---------------- timers ----------------
        self.N = 5; self.fir_coeffs = np.ones(self.N) / self.N
        self.plot_timer = QtCore.QTimer(); self.plot_timer.timeout.connect(self.update_plot); self.plot_timer.start(200)

        # ---------------- default values ----------------
        self.running = False
        self.default_lin = 0.3; self.default_ang = 2.0
        self.cmd_lin.setValue(self.default_lin); self.cmd_ang.setValue(self.default_ang)

    # -----------------------------------------------------------------
    # ------------------------ UI / ROS callbacks ---------------------
    # -----------------------------------------------------------------
    def handle_action_button(self, action: str):
        if not self.running:
            self.log_with_timestamp(f"Start Action {action} failed: Press START first.")
            return
        if self.current_action and self.current_action != action:
            self.log_with_timestamp(f'Cannot start Action {action} while {self.current_action} is active.')
            return
        self.current_action = action
        self.btn_start.setText('STOP'); self.btn_start.setStyleSheet(self.get_stop_button_style())
        self.publish_state(f'Action {action}')
        self.log_with_timestamp(f'Action {action} in progress.')

        # == Publish PoseStamped to /simple_goal ==
        if action == 'A':
            self.publish_goal_pose(10.0, 10.0, 0.0)
        elif action == 'B':
            self.publish_goal_pose(-10.0, -10.0, math.pi)

    def handle_origin_button(self):
        if not self.running:
            self.log_with_timestamp('Origin command failed: Press START first.')
            return
        self.log_with_timestamp('Origin pressed. Going home (0,0).')
        self.publish_state('Origin')
        self.publish_goal_pose(0.0, 0.0, 0.0)

    # ---------------- START / STOP ----------------
    def toggle_start_stop(self):
        if not self.running:
            self.running = True
            self.btn_start.setText('STOP'); self.btn_start.setStyleSheet(self.get_stop_button_style())
            self.log_with_timestamp('Idle state'); self.publish_state('Idle state')
        else:
            self.running = False; self.current_action = None
            self.btn_start.setText('START'); self.btn_start.setStyleSheet(self.get_start_button_style())
            self.log_with_timestamp('Emergency stop issued.'); self.publish_state('Emergency Stop')

    # ---------------- utility ----------------
    def publish_goal_pose(self, x: float, y: float, yaw: float):
        msg = PoseStamped(); msg.header.frame_id = 'map'; msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = x; msg.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w = q
        self.pub_goal.publish(msg)
        self.log_with_timestamp(f'Published Goal → ({x:.1f}, {y:.1f}, yaw={yaw:.2f})')

    # -----------------  Velocity  -----------------
    def odom_callback(self, msg: Odometry):
        lin = msg.twist.twist.linear.x; ang = msg.twist.twist.angular.z
        self.current_lin.setText(f'{lin:.2f}'); self.current_ang.setText(f'{ang:.2f}')

    def send_cmd_input(self, is_reset: bool=False):
        if not self.running:
            self.log_with_timestamp('Send cmd failed: Press START first.'); return
        twist = Twist(); twist.linear.x = self.cmd_lin.value(); twist.angular.z = self.cmd_ang.value()
        self.pub_twist.publish(twist)
        tag = '[RESET]' if is_reset else '[SEND]'
        self.log_with_timestamp(f'{tag} lin={twist.linear.x:.2f}  ang={twist.angular.z:.2f}')
        self.publish_state(f'{tag} Sent twist cmd')

    def reset_inputs(self):
        if not self.running:
            self.log_with_timestamp('Reset cmd failed: Press START first.'); return
        self.cmd_ang.setValue(self.default_ang); self.cmd_lin.setValue(self.default_lin)
        self.send_cmd_input(is_reset=True)

    def update_defaults(self, msg: Float32MultiArray):
        if len(msg.data) >= 2:
            self.default_lin, self.default_ang = msg.data[0], msg.data[1]
            self.get_logger().info(f'Default vel updated lin={self.default_lin} ang={self.default_ang}')

    # -----------------  IMU Plot ------------------
    def imu_callback(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if not hasattr(self, 't0'):
            self.t0 = t
        self.imu_data.append([t-self.t0, msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])

    def update_plot(self):
        if len(self.imu_data) <= self.N: return
        data = np.array(self.imu_data)
        t = data[:,0]; ax = data[:,1]
        current_time = t[-1]; mask = t >= current_time - 10.0
        t_window = t[mask]; ax_window = ax[mask]
        if len(ax_window) < self.N: return
        filt_ax = np.convolve(ax_window, self.fir_coeffs, 'valid')
        self.raw_line.set_data(t_window, ax_window)
        self.filtered_line.set_data(t_window[self.N-1:], filt_ax)
        self.ax.set_xlim(t_window[0], t_window[-1])
        self.ax.set_ylim(min(ax_window)-1, max(ax_window)+1)
        self.canvas.draw_idle()

    # -----------------  Display & Logging ----------------
    def log_with_timestamp(self, msg: str):
        ts = QtCore.QDateTime.currentDateTime().toString('yyyy-MM-dd HH:mm:ss')
        self.log_text.append(f'[{ts}] {msg}')
        self.log_text.verticalScrollBar().setValue(self.log_text.verticalScrollBar().maximum())

    # -----------------  Style ---------------------
    def get_start_button_style(self):
        return 'QPushButton {background-color: green; font-size:16pt; font-weight:bold; border-radius:60px; }'

    def get_stop_button_style(self):
        return 'QPushButton {background-color: red;   font-size:16pt; font-weight:bold; border-radius:60px; }'

# -----------------------------------------------------------------------------
#                                    main                                      
# -----------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    signal.signal(signal.SIGINT, signal.SIG_DFL)

    app = QtWidgets.QApplication(sys.argv)
    gui = TurtleBotGUI(); gui.show()

    executor = MultiThreadedExecutor(); executor.add_node(gui)

    # Qt timer --dirve-- rclpy.spin_once()
    ros_timer = QtCore.QTimer(); ros_timer.timeout.connect(lambda: rclpy.spin_once(gui, timeout_sec=0.01)); ros_timer.start(10)

    try:
        sys.exit(app.exec_())
    finally:
        executor.shutdown(); gui.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
