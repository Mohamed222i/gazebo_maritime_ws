#!/usr/bin/env python3
# altimeter_converter_improved.py
import math
import rclpy
from rclpy.node import Node
from ros_gz_interfaces.msg import Altimeter
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu
from rclpy.time import Time

class AltimeterConverter(Node):
    def __init__(self):
        super().__init__('altimeter_converter_improved')

        # PARAMETERS (tune these)
        self.declare_parameter('input_topic', '/Altimeter')
        self.declare_parameter('output_topic', '/sensor/altimeter')
        self.declare_parameter('frame_id', 'base_link')         # publish in base_link
        self.declare_parameter('is_range', True)                # True if vertical_position is distance to water
        self.declare_parameter('sensor_mount_height', 0.5)      # meters: height of sensor above base_link origin
        self.declare_parameter('invert_sign', False)            # True if vertical_position has inverted sign
        self.declare_parameter('zz_variance_stationary', 0.5)   # variance (m^2) when stationary
        self.declare_parameter('zz_variance_moving', 4.0)      # variance when moving (trust less)
        self.declare_parameter('lowpass_alpha', 0.3)           # smoothing: 0..1 (higher = more weight to new sample)
        self.declare_parameter('accel_threshold', 0.5)         # m/s^2 to consider "moving"
        self.declare_parameter('use_imu_for_motion', True)     # set False to detect motion via other method

        in_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        out_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # publisher and subscriber
        self.pub = self.create_publisher(PoseWithCovarianceStamped, out_topic, 10)
        self.sub = self.create_subscription(Altimeter, in_topic, self.alt_cb, 10)

        # optional IMU subscriber for motion detection
        self.using_imu = self.get_parameter('use_imu_for_motion').get_parameter_value().bool_value
        self.imu_accel_mag = 0.0
        if self.using_imu:
            self.create_subscription(Imu, '/sensor/imu', self.imu_cb, 20)

        # internal state for smoothing
        self.z_smoothed = None

    def imu_cb(self, msg: Imu):
        # compute accel magnitude (linear_accel)
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        self.imu_accel_mag = math.sqrt(ax*ax + ay*ay + az*az)

    def alt_cb(self, msg: Altimeter):
        now = self.get_clock().now()
        pose_msg = PoseWithCovarianceStamped()

        # timestamp: use incoming if recent, else now
        try:
            incoming = Time.from_msg(msg.header.stamp)
            age = (now - incoming).nanoseconds / 1e9
        except Exception:
            age = float('inf')
        if age <= 1.0:
            pose_msg.header.stamp = msg.header.stamp
        else:
            pose_msg.header.stamp = now.to_msg()

        pose_msg.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # read input value
        raw = float(msg.vertical_position)

        # apply sign invert if necessary
        if self.get_parameter('invert_sign').get_parameter_value().bool_value:
            raw = -raw

        # convert range -> absolute z if needed
        is_range = self.get_parameter('is_range').get_parameter_value().bool_value
        mount_h = self.get_parameter('sensor_mount_height').get_parameter_value().double_value
        if is_range:
            # assume vertical_position is distance from sensor down to water surface
            # absolute z of base_link = (sensor_mount_height) - range (if sensor mounted above base_link)
            z_abs = mount_h - raw
        else:
            # assume message already contains absolute z in base_link frame
            z_abs = raw

        # smoothing (exponential)
        alpha = self.get_parameter('lowpass_alpha').get_parameter_value().double_value
        if self.z_smoothed is None:
            self.z_smoothed = z_abs
        else:
            self.z_smoothed = alpha * z_abs + (1.0 - alpha) * self.z_smoothed

        pose_msg.pose.pose.position.z = float(self.z_smoothed)

        # dynamic covariance: raise variance when moving (IMU accel magnitude)
        accel_thresh = self.get_parameter('accel_threshold').get_parameter_value().double_value
        zvar_station = self.get_parameter('zz_variance_stationary').get_parameter_value().double_value
        zvar_moving = self.get_parameter('zz_variance_moving').get_parameter_value().double_value

        moving = False
        if self.using_imu:
            # subtract gravity effect (approx) - check magnitude relative to 9.81
            # if the accel vector differs from gravity by more than threshold -> moving/rough
            mag = self.imu_accel_mag
            if abs(mag - 9.81) > accel_thresh:
                moving = True

        cov = [0.0]*36
        cov[0] = 1e6   # big x var (not used)
        cov[7] = 1e6   # big y var
        cov[14] = zvar_moving if moving else zvar_station
        pose_msg.pose.covariance = cov

        self.pub.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = AltimeterConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
