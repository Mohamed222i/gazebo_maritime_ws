#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

class GPSCovarianceSetter(Node):
    def __init__(self):
        super().__init__('gps_cov')
        self.declare_parameter('input_topic', '/navsat')
        self.declare_parameter('output_topic', '/sensor/gps')
        self.declare_parameter('fixed_var_xy', 4.0)
        self.declare_parameter('fixed_var_z', 16.0)

        in_topic = self.get_parameter('input_topic').value
        out_topic = self.get_parameter('output_topic').value

        self.pub = self.create_publisher(NavSatFix, out_topic, 10)
        self.sub = self.create_subscription(NavSatFix, in_topic, self.cb, 10)

    def cb(self, msg: NavSatFix):
        msg.position_covariance = [0.0]*9
        msg.position_covariance[0] = self.get_parameter('fixed_var_xy').value
        msg.position_covariance[4] = self.get_parameter('fixed_var_xy').value
        msg.position_covariance[8] = self.get_parameter('fixed_var_z').value
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = GPSCovarianceSetter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
