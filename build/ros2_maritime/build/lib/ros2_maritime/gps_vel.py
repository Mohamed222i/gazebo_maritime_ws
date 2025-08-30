# gps_vel_from_odom.py
import rclpy, numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped

class GPSVel(Node):
    def __init__(self):
        super().__init__('gps_vel')
        self.alpha = self.declare_parameter('alpha', 0.6).value
        self.beta  = self.declare_parameter('beta',  0.2).value
        self.last_t = None
        self.x = self.y = self.vx = self.vy = 0.0
        self.sub = self.create_subscription(Odometry, '/odometry/gps', self.cb, 10)
        self.pub = self.create_publisher(TwistWithCovarianceStamped, '/gps/twist', 10)

    def cb(self, odom: Odometry):
        t = odom.header.stamp.sec + odom.header.stamp.nanosec*1e-9
        if self.last_t is None:
            self.last_t = t
            self.x = odom.pose.pose.position.x
            self.y = odom.pose.pose.position.y
            return
        dt = t - self.last_t
        if dt <= 0: return
        x_pred = self.x + self.vx*dt
        y_pred = self.y + self.vy*dt
        xm = odom.pose.pose.position.x
        ym = odom.pose.pose.position.y
        rx, ry = xm - x_pred, ym - y_pred
        self.x = x_pred + self.alpha*rx
        self.y = y_pred + self.alpha*ry
        self.vx = self.vx + (self.beta*rx)/dt
        self.vy = self.vy + (self.beta*ry)/dt
        self.last_t = t

        msg = TwistWithCovarianceStamped()
        msg.header = odom.header
        msg.twist.twist.linear.x = self.vx
        msg.twist.twist.linear.y = self.vy
        cov = [0.0]*36
        cov[0] = 0.09   # (0.3 m/s)^2 to start; tune down if clean
        cov[7] = 0.09
        cov[14] = cov[21] = cov[28] = cov[35] = 1e9
        msg.twist.covariance = cov
        self.pub.publish(msg)

def main():
    rclpy.init(); rclpy.spin(GPSVel()); rclpy.shutdown()
