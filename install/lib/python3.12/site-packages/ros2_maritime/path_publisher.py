#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        self.spline_pub = self.create_publisher(Path, '/reference_path', 10)
        self.waypoints_pub = self.create_publisher(Path, '/waypoints_path', 10)

        self.timer = self.create_timer(1.0, self.publish_paths)

        # Define waypoints (e.g., for a dam survey lawnmower pattern)
        self.waypoints = np.array([
            (0.0, 0.0),
            (20.0, 20.0),
            (40.0, 30.0),
            (70.0, 40.0),
            (90.0, 50.0)

        ])

    def publish_paths(self):
        now = self.get_clock().now().to_msg()

        # --- 1. Publish Waypoints Path ---
        waypoint_path = Path()
        waypoint_path.header.frame_id = 'odom'
        waypoint_path.header.stamp = now

        for x, y in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            waypoint_path.poses.append(ps)

        self.waypoints_pub.publish(waypoint_path)
        self.get_logger().info(f"Published raw waypoint path with {len(waypoint_path.poses)} points")

        # --- 2. Publish Catmull-Rom Smoothed Path ---
        smoothed_path = Path()
        smoothed_path.header.frame_id = 'odom'
        smoothed_path.header.stamp = now

        interpolated_points = self.catmull_rom_spline(self.waypoints, num_points=20)

        for x, y in interpolated_points:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            smoothed_path.poses.append(ps)

        self.spline_pub.publish(smoothed_path)
        self.get_logger().info(f"Published Catmull-Rom path with {len(smoothed_path.poses)} points")

    def catmull_rom_spline(self, waypoints, num_points=20):
        """Generate Catmull-Rom spline between waypoints."""
        def interpolate(p0, p1, p2, p3, t):
            t2 = t * t
            t3 = t2 * t
            x = 0.5 * ((2 * p1[0]) +
                       (-p0[0] + p2[0]) * t +
                       (2*p0[0] - 5*p1[0] + 4*p2[0] - p3[0]) * t2 +
                       (-p0[0] + 3*p1[0] - 3*p2[0] + p3[0]) * t3)

            y = 0.5 * ((2 * p1[1]) +
                       (-p0[1] + p2[1]) * t +
                       (2*p0[1] - 5*p1[1] + 4*p2[1] - p3[1]) * t2 +
                       (-p0[1] + 3*p1[1] - 3*p2[1] + p3[1]) * t3)

            return (x, y)

        result = []

        for i in range(1, len(waypoints) - 2):
            p0, p1, p2, p3 = waypoints[i - 1], waypoints[i], waypoints[i + 1], waypoints[i + 2]
            for j in range(num_points):
                t = j / num_points
                result.append(interpolate(p0, p1, p2, p3, t))
        # Add final waypoint
        result.append(tuple(waypoints[-2]))
        result.append(tuple(waypoints[-1]))
        return result


def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


'''



#below is the cubic B spline method



#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import numpy as np
from scipy.interpolate import splprep, splev  # Spline tools

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_publisher')

        # Publisher for the smoothed spline path
        self.spline_pub = self.create_publisher(Path, '/reference_path', 10)
        # Publisher for the original waypoints path
        self.waypoints_pub = self.create_publisher(Path, '/waypoints_path', 10)

        self.timer = self.create_timer(1.0, self.publish_paths)

        # Define waypoints as a closed loop
        self.waypoints = np.array([
            (0.0, 0.0),
            (100.0, 0.0),
            (100.0, 20.0),
            (0.0, 20.0),
            (0.0, 40.0),
            (100.0, 40.0),
            (100.0, 60.0),
            (0.0, 60.0),
            (0.0, 80.0),
            (100.0, 80.0),
            (100.0, 100.0),
            (0.0, 100.0)
        ])

    def publish_paths(self):
        now = self.get_clock().now().to_msg()

        # --- 1. Publish Waypoints Path (raw) ---
        waypoint_path = Path()
        waypoint_path.header.frame_id = 'odom'
        waypoint_path.header.stamp = now

        for x, y in self.waypoints:
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0  # Default orientation
            waypoint_path.poses.append(ps)

        self.waypoints_pub.publish(waypoint_path)
        self.get_logger().info(f"Published raw waypoint path with {len(waypoint_path.poses)} points")

        # --- 2. Publish Smoothed Spline Path ---
        spline_path = Path()
        spline_path.header.frame_id = 'odom'
        spline_path.header.stamp = now

        # Extract x and y from waypoints
        x = self.waypoints[:, 0]
        y = self.waypoints[:, 1]

        # Fit B-spline (closed loop)
        tck, _ = splprep([x, y], s=0.5, per=True)
        u_fine = np.linspace(0, 1, 100)
        x_smooth, y_smooth = splev(u_fine, tck)

        for x, y in zip(x_smooth, y_smooth):
            ps = PoseStamped()
            ps.header.frame_id = 'odom'
            ps.header.stamp = now
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0  # Default orientation
            spline_path.poses.append(ps)

        self.spline_pub.publish(spline_path)
        self.get_logger().info(f"Published B-spline path with {len(spline_path.poses)} points")

def main():
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

'''