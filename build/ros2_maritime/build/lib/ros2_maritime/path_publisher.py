#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')
        self.pub = self.create_publisher(Path, '/reference_path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

        # Example waypoints in odom frame
        self.waypoints = [
            (0.0, 0.0),
            (10.0, 0.0),
            (10.0, 10.0),
            (0.0, 10.0)
        ]

    def publish_path(self):
        path = Path()
        path.header.frame_id = 'odom'  # match EKF output frame
        path.header.stamp = self.get_clock().now().to_msg()

        for x, y in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            path.poses.append(ps)

        self.pub.publish(path)
        self.get_logger().info(f"Published path with {len(path.poses)} waypoints")

def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
