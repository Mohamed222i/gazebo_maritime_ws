#!/usr/bin/env python3
"""
SMC CSV Recorder (ROS2, Python)

Subscribes to key SMC and odometry topics and writes aligned rows to CSV at a fixed sample rate.

Outputs a CSV with columns (one row per sample):
 t, target_x_m, target_y_m, target_yaw_rad, odom_x_m, odom_y_m, odom_yaw_rad, odom_r_rps,
 r_cmd_rps, M_cmd_Nm, left_cmd_N, right_cmd_N, F_total_N, cross_track_m, yaw_err_rad, desired_speed

Usage:
 1. Place this file in a ROS2 Python package's scripts/ or src/ folder and make executable.
 2. Run with: ros2 run <your_package> smc_csv_recorder.py  OR python3 smc_csv_recorder.py

Parameters:
 - output_csv (string): path to CSV file (default /tmp/smc_run.csv)
 - sample_rate (double): sampling frequency in Hz (default 20.0)
 - use_ros_time (bool): prefer message header stamps when available (default True)
 - odom_topic, target_topic, r_cmd_topic, M_cmd_topic, left_cmd_topic, right_cmd_topic, f_total_topic, cross_track_topic, yaw_err_topic, desired_speed_topic
 - thruster_half (double): lever arm (m) used to compute M_actual from thrusters (default 1.027135)

The node is robust to missing topics (writes NaN for missing fields) and writes the header if the file is new.

"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import csv
import os
import math
import time


def quat_to_yaw(q):
    # q: geometry_msgs.msg.Quaternion-like with x,y,z,w
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SmcCsvRecorder(Node):
    def __init__(self):
        super().__init__('csv_rec')

        # --- parameters ---
        self.declare_parameter('output_csv', '/tmp/smc_run.csv')
        self.declare_parameter('sample_rate', 20.0)
        self.declare_parameter('use_ros_time', True)

        # topic names (defaults match the SMC node)
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('target_topic', '/smc/target_point')
        self.declare_parameter('r_cmd_topic', '/smc/debug/r_cmd')
        self.declare_parameter('M_cmd_topic', '/smc/debug/M_cmd')
        self.declare_parameter('left_cmd_topic', '/smc/debug/left_cmd')
        self.declare_parameter('right_cmd_topic', '/smc/debug/right_cmd')
        self.declare_parameter('f_total_topic', '/smc/debug/F_total')
        self.declare_parameter('cross_track_topic', '/smc/debug/cross_track')
        self.declare_parameter('yaw_err_topic', '/smc/debug/yaw_err')
        self.declare_parameter('desired_speed_topic', '/smc/debug/desired_speed')
        self.declare_parameter('thruster_half', 1.027135)

        self.output_csv = self.get_parameter('output_csv').get_parameter_value().string_value
        self.sample_rate = float(self.get_parameter('sample_rate').get_parameter_value().double_value)
        self.use_ros_time = bool(self.get_parameter('use_ros_time').get_parameter_value().bool_value)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.target_topic = self.get_parameter('target_topic').get_parameter_value().string_value
        self.r_cmd_topic = self.get_parameter('r_cmd_topic').get_parameter_value().string_value
        self.M_cmd_topic = self.get_parameter('M_cmd_topic').get_parameter_value().string_value
        self.left_cmd_topic = self.get_parameter('left_cmd_topic').get_parameter_value().string_value
        self.right_cmd_topic = self.get_parameter('right_cmd_topic').get_parameter_value().string_value
        self.f_total_topic = self.get_parameter('f_total_topic').get_parameter_value().string_value
        self.cross_track_topic = self.get_parameter('cross_track_topic').get_parameter_value().string_value
        self.yaw_err_topic = self.get_parameter('yaw_err_topic').get_parameter_value().string_value
        self.desired_speed_topic = self.get_parameter('desired_speed_topic').get_parameter_value().string_value
        self.thruster_half = float(self.get_parameter('thruster_half').get_parameter_value().double_value)

        qos = QoSProfile(depth=10)

        # latest message storage
        self.latest = {
            't': 0.0,
            'target_x': float('nan'), 'target_y': float('nan'), 'target_yaw': float('nan'),
            'odom_x': float('nan'), 'odom_y': float('nan'), 'odom_yaw': float('nan'), 'odom_r': float('nan'),
            'r_cmd': float('nan'), 'M_cmd': float('nan'),
            'left_cmd': float('nan'), 'right_cmd': float('nan'), 'F_total': float('nan'),
            'cross_track': float('nan'), 'yaw_err': float('nan'), 'desired_speed': float('nan')
        }

        # subscribers
        self.create_subscription(Odometry, self.odom_topic, self.cb_odom, qos)
        self.create_subscription(PoseStamped, self.target_topic, self.cb_target, qos)
        # many debug topics are Float64
        self.create_subscription(Float64, self.r_cmd_topic, self.cb_float('r_cmd'), qos)
        self.create_subscription(Float64, self.M_cmd_topic, self.cb_float('M_cmd'), qos)
        self.create_subscription(Float64, self.left_cmd_topic, self.cb_float('left_cmd'), qos)
        self.create_subscription(Float64, self.right_cmd_topic, self.cb_float('right_cmd'), qos)
        self.create_subscription(Float64, self.f_total_topic, self.cb_float('F_total'), qos)
        self.create_subscription(Float64, self.cross_track_topic, self.cb_float('cross_track'), qos)
        self.create_subscription(Float64, self.yaw_err_topic, self.cb_float('yaw_err'), qos)
        self.create_subscription(Float64, self.desired_speed_topic, self.cb_float('desired_speed'), qos)

        # open CSV and write header if needed
        write_header = not os.path.exists(self.output_csv) or os.path.getsize(self.output_csv) == 0
        self.csv_file = open(self.output_csv, 'a', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        if write_header:
            header = [
                't',
                'target_x_m', 'target_y_m', 'target_yaw_rad',
                'odom_x_m', 'odom_y_m', 'odom_yaw_rad', 'odom_r_rps',
                'r_cmd_rps', 'M_cmd_Nm',
                'left_cmd_N', 'right_cmd_N', 'M_actual_Nm',
                'F_total_N', 'cross_track_m', 'yaw_err_rad', 'desired_speed_mps'
            ]
            self.csv_writer.writerow(header)
            self.csv_file.flush()

        # sampling timer
        period = 1.0 / max(1e-3, self.sample_rate)
        self.timer = self.create_timer(period, self.timer_cb)

        self.get_logger().info(f"SMC CSV recorder started, writing to: {self.output_csv} @ {self.sample_rate} Hz")

    # === callbacks ===
    def cb_odom(self, msg: Odometry):
        # pose
        self.latest['odom_x'] = msg.pose.pose.position.x
        self.latest['odom_y'] = msg.pose.pose.position.y
        yaw = quat_to_yaw(msg.pose.pose.orientation)
        self.latest['odom_yaw'] = yaw
        self.latest['odom_r'] = msg.twist.twist.angular.z
        if self.use_ros_time:
            self.latest['t'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def cb_target(self, msg: PoseStamped):
        self.latest['target_x'] = msg.pose.position.x
        self.latest['target_y'] = msg.pose.position.y
        tyaw = quat_to_yaw(msg.pose.orientation)
        self.latest['target_yaw'] = tyaw
        if self.use_ros_time and (self.latest.get('t', 0.0) == 0.0):
            self.latest['t'] = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def cb_float(self, key):
        def _cb(msg: Float64):
            try:
                self.latest[key] = float(msg.data)
            except Exception:
                self.latest[key] = float('nan')
        return _cb

    def timer_cb(self):
        # ensure timestamp
        t = self.latest.get('t', 0.0)
        if t == 0.0:
            t = self.get_clock().now().nanoseconds * 1e-9

        # compute derived values
        left = self.latest.get('left_cmd', float('nan'))
        right = self.latest.get('right_cmd', float('nan'))
        M_actual = float('nan')
        try:
            if not math.isnan(left) and not math.isnan(right):
                M_actual = (right - left) * self.thruster_half
        except Exception:
            M_actual = float('nan')

        row = [
            t,
            self.latest.get('target_x', float('nan')), self.latest.get('target_y', float('nan')), self.latest.get('target_yaw', float('nan')),
            self.latest.get('odom_x', float('nan')), self.latest.get('odom_y', float('nan')), self.latest.get('odom_yaw', float('nan')), self.latest.get('odom_r', float('nan')),
            self.latest.get('r_cmd', float('nan')), self.latest.get('M_cmd', float('nan')),
            left, right, M_actual,
            self.latest.get('F_total', float('nan')), self.latest.get('cross_track', float('nan')), self.latest.get('yaw_err', float('nan')), self.latest.get('desired_speed', float('nan'))
        ]

        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SmcCsvRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
