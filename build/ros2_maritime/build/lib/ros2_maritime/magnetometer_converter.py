# magnetometer_converter.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import MagneticField, Imu

class MagnetometerConverter(Node):
    def __init__(self):
        super().__init__('magnetometer_converter')
        self.pub = self.create_publisher(
            Imu, 
            '/magnetometer_imu', 
            10
        )
        self.sub = self.create_subscription(
            MagneticField,
            '/magnetometer',
            self.convert_cb,
            10
        )
    
    def convert_cb(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        # Convert magnetic field to orientation (simplified)
        # In real systems, use sensor fusion like Madgwick
        imu_msg.orientation_covariance[0] = -1  # Indicate no orientation
        imu_msg.orientation_covariance[4] = -1
        imu_msg.orientation_covariance[8] = 0.05  # Yaw covariance
        
        # For EKF, we only need the magnetic field for yaw reference
        self.pub.publish(imu_msg)

def main(args=None):
    rclpy.init(args=args)
    node = MagnetometerConverter()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
