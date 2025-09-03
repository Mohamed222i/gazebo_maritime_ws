#!/usr/bin/env python3
"""
smc_data_logger_simple.py

A simple logger for USV SMC data.
- Saves CSV in the same folder as this script.
- One CSV per run (timestamped).
- Subscribes to odometry, target, thrusts, and SMC debug topics.

Usage:
  ros2 run <your_pkg> smc_data_logger_simple
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64
import csv, os
from datetime import datetime
import math

class SimpleSMCLogger(Node):
    def __init__(self):
        super().__init__('data_logger')

        # prepare output path (same folder as script)
        script_dir = os.path.dirname(os.path.realpath(__file__))
        tstamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(script_dir, f"usv_log_{tstamp}.csv")

        # open file
        self.csv_file = open(self.csv_path, "w", newline="")
        self.writer = csv.writer(self.csv_file)

        # header
        self.header = [
            "t",
            "odom_x", "odom_y", "odom_yaw", "odom_u",
            "target_x", "target_y", "target_yaw",
            "left_cmd", "right_cmd",
            "F_total", "M_cmd",
            "s", "yaw_err", "cross_track",
            "r_cmd", "switching", "desired_speed"
        ]
        self.writer.writerow(self.header)

        # storage
        self.data = {k: float("nan") for k in self.header}

        # subscribers
        self.create_subscription(Odometry, "/odometry/filtered", self.odom_cb, 10)
        self.create_subscription(PoseStamped, "/smc/target_point", self.target_cb, 10)
        self.create_subscription(Float64, "/wamv/left_thrust", lambda m: self.float_cb(m, "left_cmd"), 10)
        self.create_subscription(Float64, "/wamv/right_thrust", lambda m: self.float_cb(m, "right_cmd"), 10)

        # smc debug topics
        self._sub_debug("/smc/debug/s", "s")
        self._sub_debug("/smc/debug/yaw_err", "yaw_err")
        self._sub_debug("/smc/debug/cross_track", "cross_track")
        self._sub_debug("/smc/debug/switching", "switching")
        self._sub_debug("/smc/debug/r_cmd", "r_cmd")
        self._sub_debug("/smc/debug/F_total", "F_total")
        self._sub_debug("/smc/debug/M_cmd", "M_cmd")
        self._sub_debug("/smc/debug/desired_speed", "desired_speed")

        # timer to write rows at 10 Hz
        self.create_timer(0.1, self.write_row)

        self.get_logger().info(f"Logging to {self.csv_path}")

    def _sub_debug(self, topic, key):
        self.create_subscription(Float64, topic, lambda m, k=key: self.float_cb(m, k), 10)

    def odom_cb(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.data["t"] = t
        self.data["odom_x"] = msg.pose.pose.position.x
        self.data["odom_y"] = msg.pose.pose.position.y
        # yaw from quaternion
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.data["odom_yaw"] = math.atan2(siny, cosy)
        self.data["odom_u"] = msg.twist.twist.linear.x

    def target_cb(self, msg: PoseStamped):
        self.data["target_x"] = msg.pose.position.x
        self.data["target_y"] = msg.pose.position.y
        q = msg.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y*q.y + q.z*q.z)
        self.data["target_yaw"] = math.atan2(siny, cosy)

    def float_cb(self, msg: Float64, key: str):
        self.data[key] = msg.data

    def write_row(self):
        row = [self.data.get(k, float("nan")) for k in self.header]
        self.writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimpleSMCLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
