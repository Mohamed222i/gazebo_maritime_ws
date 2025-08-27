# altimeter_converter.py
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from geometry_msgs.msg import PoseWithCovarianceStamped

class AltimeterConverter(Node):
    def __init__(self):
        super().__init__('altimeter_converter')
        self.pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/sensor/altimeter', 
            10
        )
        self.sub = self.create_subscription(
            Altimeter,
            '/Altimeter',
            self.convert_cb,
            10
        )
    
    def convert_cb(self, msg):
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = msg.header
        pose_msg.pose.pose.position.z = msg.vertical_position
        pose_msg.pose.covariance[14] = 0.1  # Z covariance
        self.pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AltimeterConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()