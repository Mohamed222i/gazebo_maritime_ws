#!/usr/bin/env python3
"""
record_executed_path.py

Publishes the robot's executed trajectory from Odometry as:
 - /smc/executed_path         (nav_msgs/Path)
 - /smc/executed_path_marker  (visualization_msgs/Marker LINE_STRIP)

Provides service:
 - /smc/clear_executed_path   (std_srvs/srv/Empty)  -> clears recorded path

Parameters:
 - odom_topic (string)   default: /odometry/filtered
 - frame_id   (string)   default: odom
 - publish_rate (double) default: 10.0
 - downsample (int)      keep every Nth odom (default 1)
 - max_points (int)      max stored path points (default 2000)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

import math
from typing import List, Tuple

class ExecutedPathRecorder(Node):
    def __init__(self):
        super().__init__('path_rec')

        # params
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('downsample', 1)
        self.declare_parameter('max_points', 2000)
        self.declare_parameter('path_topic', '/smc/executed_path')
        self.declare_parameter('marker_topic', '/smc/executed_path_marker')

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_rate = float(self.get_parameter('publish_rate').get_parameter_value().double_value)
        self.downsample = int(self.get_parameter('downsample').get_parameter_value().integer_value)
        self.max_points = int(self.get_parameter('max_points').get_parameter_value().integer_value)
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.marker_topic = self.get_parameter('marker_topic').get_parameter_value().string_value

        # internal storage
        self.path_poses: List[PoseStamped] = []
        self.counter = 0

        qos = QoSProfile(depth=10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)
        self.path_pub = self.create_publisher(Path, self.path_topic, qos)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, qos)

        # clear service
        self.clear_srv = self.create_service(Empty, '/smc/clear_executed_path', self.handle_clear_path)

        # timer to publish at desired rate (publishes latest path)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_publish)

        self.get_logger().info(f'ExecutedPathRecorder running. Subscribing: {self.odom_topic}')

    def odom_cb(self, msg: Odometry):
        # Downsample if requested
        self.counter += 1
        if self.downsample > 1 and (self.counter % self.downsample) != 0:
            return

        ps = PoseStamped()
        ps.header = msg.header
        # ensure frame is the configured one (if odom header uses different frame, we keep msg.header.frame_id)
        # most robust: keep incoming header.frame_id, but set path.header.frame_id to user configured frame.
        # We'll store poses with their stamped header and use path.header for RViz fixed frame consistency.
        ps.pose = msg.pose.pose

        # Append and clamp size
        self.path_poses.append(ps)
        if len(self.path_poses) > self.max_points:
            # remove oldest to keep memory bounded
            excess = len(self.path_poses) - self.max_points
            self.path_poses = self.path_poses[excess:]

    def timer_publish(self):
        if len(self.path_poses) == 0:
            return

        # Publish nav_msgs/Path
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        # Use configured frame_id so RViz fixed frame can match; ensure ODOM/EKf frames match when using RViz
        path.header.frame_id = self.frame_id
        # Copy poses but adjust their header frame to the path.frame (optionally you can leave original header)
        # WARNING: if frames differ you must transform poses into path.frame (not done here).
        # We will simply set pose headers to path frame (works if odom already in same frame).
        for p in self.path_poses:
            ps = PoseStamped()
            ps.header.stamp = p.header.stamp
            ps.header.frame_id = self.frame_id
            ps.pose = p.pose
            path.poses.append(ps)
        self.path_pub.publish(path)

        # Publish marker LINE_STRIP for nicer styling in RViz
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'executed_path'
        mk.id = 0
        mk.type = Marker.LINE_STRIP
        mk.action = Marker.ADD
        mk.scale = Vector3(x=0.15, y=0.0, z=0.0)  # line width
        mk.color.a = 1.0
        mk.color.r = 0.9
        mk.color.g = 0.2
        mk.color.b = 0.2
        mk.points = []
        for p in self.path_poses:
            pt = Point()
            pt.x = p.pose.position.x
            pt.y = p.pose.position.y
            pt.z = p.pose.position.z if hasattr(p.pose.position, 'z') else 0.0
            mk.points.append(pt)
        self.marker_pub.publish(mk)

    def handle_clear_path(self, request, response):
        self.path_poses = []
        self.get_logger().info('Cleared executed path buffer via service.')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ExecutedPathRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
