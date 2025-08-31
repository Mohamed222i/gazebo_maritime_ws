import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, Pose
import tf_transformations

class HeadingArrow(Node):
    def __init__(self):
        super().__init__('heading')
        self.sub = self.create_subscription(Odometry, '/odometry/filtered', self.cb, 10)
        self.pub = self.create_publisher(PoseArray, '/heading_arrow', 10)

    def cb(self, msg):
        # Extract quaternion
        q = msg.pose.pose.orientation
        quat = [q.x, q.y, q.z, q.w]
        _, _, yaw = tf_transformations.euler_from_quaternion(quat)

        # Build a pose for the arrow
        p = Pose()
        p.position = msg.pose.pose.position
        p.orientation = q  # keep the full quaternion

        pa = PoseArray()
        pa.header = msg.header
        pa.poses.append(p)

        self.pub.publish(pa)

def main():
    rclpy.init()
    node = HeadingArrow()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
