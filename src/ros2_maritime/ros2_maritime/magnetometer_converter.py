import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, MagneticField
import numpy as np
from tf_transformations import quaternion_from_euler

class MagToImu(Node):
    def __init__(self):
        super().__init__('magnetometer_converter')

        # Subscribe to accelerometer (from IMU)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_cb, 10)

        # Subscribe to magnetometer
        self.mag_sub = self.create_subscription(MagneticField, '/magnetometer', self.mag_cb, 10)

        # Publisher for fused IMU
        self.imu_pub = self.create_publisher(Imu, '/sensor/magnetometer', 10)

        # Store latest accel and mag
        self.last_accel = None
        self.last_mag = None

        # --- NEW: tracking time for rate limit ---
        self.target_period = 1.0 / 30.0   # seconds
        self.last_pub_time = self.get_clock().now()



    def imu_cb(self, imu_msg: Imu):
        # Save latest linear acceleration
        self.last_accel = imu_msg.linear_acceleration
        # Try to fuse if we also have mag
        self.try_publish(imu_msg.header)
    
    def mag_cb(self, mag_msg: MagneticField):
        # Save latest magnetic field
        self.last_mag = mag_msg.magnetic_field
        # Try to fuse if we also have accel
        self.try_publish(mag_msg.header)

    def try_publish(self, header):
        if self.last_accel is None or self.last_mag is None:
            return  # Need both accel and mag

        # --- NEW: check elapsed time ---
        now = self.get_clock().now()
        if (now - self.last_pub_time).nanoseconds * 1e-9 < self.target_period:
            return  # Too soon, skip this one
        self.last_pub_time = now


        # --- Compute roll & pitch from accelerometer ---
        ax = self.last_accel.x
        ay = self.last_accel.y
        az = self.last_accel.z

        # Avoid division by zero
        if (ay**2 + az**2) == 0:
            return

        roll = np.arctan2(ay, az)
        pitch = np.arctan2(-ax, np.sqrt(ay**2 + az**2))

        # --- Compute yaw from magnetometer ---
        mx = self.last_mag.x
        my = self.last_mag.y
        mz = self.last_mag.z

        # Tilt compensation for yaw
        mag_x_comp = mx * np.cos(pitch) + mz * np.sin(pitch)
        mag_y_comp = mx * np.sin(roll) * np.sin(pitch) + my * np.cos(roll) - mz * np.sin(roll) * np.cos(pitch)
        yaw = np.arctan2(-mag_y_comp, mag_x_comp)

        # --- Convert to quaternion ---
        q = quaternion_from_euler(roll, pitch, yaw)

        # --- Publish fused IMU ---
        imu_msg = Imu()
        imu_msg.header = header
        imu_msg.orientation.x = q[0]
        imu_msg.orientation.y = q[1]
        imu_msg.orientation.z = q[2]
        imu_msg.orientation.w = q[3]
        imu_msg.orientation_covariance = [0.05, 0.0, 0.0,
                                          0.0, 0.05, 0.0,
                                          0.0, 0.0, 0.05]

        self.imu_pub.publish(imu_msg)

def main():
    rclpy.init()
    node = MagToImu()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
