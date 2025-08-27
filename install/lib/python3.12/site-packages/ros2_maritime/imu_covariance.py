import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuCovarianceFix(Node):
    def __init__(self):
        super().__init__('imu_covariance')
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/sensor/imu', 10)

    def cb(self, msg):
        if all(v == 0.0 for v in msg.orientation_covariance):
            msg.orientation_covariance = [0.01, 0, 0,
                                          0, 0.01, 0,
                                          0, 0, 0.02]
        self.pub.publish(msg)

rclpy.init()
rclpy.spin(ImuCovarianceFix())
