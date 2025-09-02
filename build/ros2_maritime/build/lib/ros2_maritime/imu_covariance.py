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






#USE THIS AS  A NOISED IMU alternative
'''

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random

class ImuNoiseInjector(Node):
    def __init__(self):
        super().__init__('imu_covariance')
        self.sub = self.create_subscription(Imu, '/imu', self.cb, 10)
        self.pub = self.create_publisher(Imu, '/sensor/imu', 10)

        # Standard deviations for noise
        self.orientation_noise = 0.0005      # Quaternion units
        self.angular_velocity_noise = 0.001  # rad/s
        self.linear_accel_noise = 0.01       # m/s^2

    def cb(self, msg):
        # Add noise to orientation
        msg.orientation.x += random.gauss(0, self.orientation_noise)
        msg.orientation.y += random.gauss(0, self.orientation_noise)
        msg.orientation.z += random.gauss(0, self.orientation_noise)
        msg.orientation.w += random.gauss(0, self.orientation_noise)

        # Add noise to angular velocity
        msg.angular_velocity.x += random.gauss(0, self.angular_velocity_noise)
        msg.angular_velocity.y += random.gauss(0, self.angular_velocity_noise)
        msg.angular_velocity.z += random.gauss(0, self.angular_velocity_noise)

        # Add noise to linear acceleration
        msg.linear_acceleration.x += random.gauss(0, self.linear_accel_noise)
        msg.linear_acceleration.y += random.gauss(0, self.linear_accel_noise)
        msg.linear_acceleration.z += random.gauss(0, self.linear_accel_noise)

        # Set orientation covariance
        msg.orientation_covariance = [
            self.orientation_noise ** 2, 0, 0,
            0, self.orientation_noise ** 2, 0,
            0, 0, self.orientation_noise ** 2
        ]

        # Set angular velocity covariance
        msg.angular_velocity_covariance = [
            self.angular_velocity_noise ** 2, 0, 0,
            0, self.angular_velocity_noise ** 2, 0,
            0, 0, self.angular_velocity_noise ** 2
        ]

        # Set linear acceleration covariance
        msg.linear_acceleration_covariance = [
            self.linear_accel_noise ** 2, 0, 0,
            0, self.linear_accel_noise ** 2, 0,
            0, 0, self.linear_accel_noise ** 2
        ]

        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ImuNoiseInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

    
    '''