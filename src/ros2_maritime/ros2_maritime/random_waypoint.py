#!/usr/bin/env python3
import rclpy
import random
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker
import math

class RandomWaypointPublisher(Node):
    def __init__(self):
        super().__init__('random_waypoint_publisher')

        # Publisher for waypoint (for controllers)
        self.pose_pub = self.create_publisher(PoseStamped, '/random_waypoint', 10)

        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(Marker, '/random_waypoint_marker', 10)

        # Timer to generate new waypoint every 10s
        self.timer = self.create_timer(10.0, self.publish_waypoint)

        self.frame_id = "map"   # Change to "odom" if needed
        self.get_logger().info("Random Waypoint Publisher started.")

    def publish_waypoint(self):
        # Generate random x, y within bounds
        x = random.uniform(-10.0, 10.0)
        y = random.uniform(-10.0, 10.0)

        # Random yaw orientation
        yaw = random.uniform(-math.pi, math.pi)
        qz = math.sin(yaw * 0.5)
        qw = math.cos(yaw * 0.5)

        # PoseStamped for control
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self.pose_pub.publish(pose)

        # Marker for visualization
        marker = Marker()
        marker.header.stamp = pose.header.stamp
        marker.header.frame_id = self.frame_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.id = 0

        marker.pose = pose.pose
        marker.scale.x = 1.0   # Arrow length
        marker.scale.y = 0.2   # Arrow width
        marker.scale.z = 0.2
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

        self.get_logger().info(f"New waypoint: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RandomWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
