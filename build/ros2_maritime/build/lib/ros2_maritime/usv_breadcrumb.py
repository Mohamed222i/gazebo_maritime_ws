#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

class USVBreadcrumb(Node):
    def __init__(self):
        super().__init__('usv_breadcrumb')

        # Parameters
        self.declare_parameter('thrust', 50.0)
        self.declare_parameter('duration', 5.0)  # seconds of thrust

        self.thrust_val = float(self.get_parameter('thrust').value)
        self.duration   = float(self.get_parameter('duration').value)
        self.start_time = time.time()

        # Publishers
        self.left_pub  = self.create_publisher(Float64, '/wamv/left_thrust', 10)
        self.right_pub = self.create_publisher(Float64, '/wamv/right_thrust', 10)
        self.path_pub  = self.create_publisher(Path, '/viz/actual_path', 10)

        # Subscriber
        self.create_subscription(Odometry, '/odometry/filtered', self.odom_cb, 10)

        # Path storage
        self.path = Path()
        self.path.header.frame_id = 'map'

        # Timers
        self.create_timer(0.1, self.send_thrust)  # 10 Hz thrust
        self.create_timer(0.1, self.publish_path) # 10 Hz path

    def send_thrust(self):
        elapsed = time.time() - self.start_time
        msg = Float64()
        if elapsed < self.duration:
            msg.data = self.thrust_val
        else:
            msg.data = 0.0
        self.left_pub.publish(msg)
        self.right_pub.publish(msg)

    def odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header = msg.header
        ps.header.frame_id = 'map'
        ps.pose = msg.pose.pose
        self.path.poses.append(ps)
        if len(self.path.poses) > 2000:
            self.path.poses = self.path.poses[-2000:]

    def publish_path(self):
        self.path.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path)

def main():
    rclpy.init()
    rclpy.spin(USVBreadcrumb())
    rclpy.shutdown()

if __name__ == '__main__':
    main()
