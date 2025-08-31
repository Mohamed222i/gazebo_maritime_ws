#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64


def wrap_to_pi(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def sat(x):
    return max(-1.0, min(1.0, x))


def yaw_from_quat(q):
    # ZYX yaw from quaternion
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SMCUSV(Node):
    def __init__(self):
        super().__init__('smc_control')

        # Parameters (tune these)
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('reference_path_topic', '/reference_path')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('left_thrust_topic', '/wamv/left_thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/right_thrust')

        # Speed control
        self.declare_parameter('v_ref', 1.0)        # desired surge speed [m/s]
        self.declare_parameter('v_ref_max', 2.0)    # maximum speed corresponding to max thrust
        self.declare_parameter('k_v', 20.0)         # speed SMC gain
        self.declare_parameter('phi_v', 0.2)        # speed boundary layer

        # Yaw / path-following control
        self.declare_parameter('lookahead_dist', 3.0)  # path lookahead [m]
        self.declare_parameter('k_ct', 1.0)            # cross-track weight in surface
        self.declare_parameter('k_s', 0.8)             # switching gain for yaw surface
        self.declare_parameter('k_d', 0.3)             # damping on yaw rate
        self.declare_parameter('phi_yaw', 0.1)         # yaw boundary layer

        # Thrust mapping & limits
        self.declare_parameter('thrust_min', 0.0)
        self.declare_parameter('thrust_max', 100.0)
        self.declare_parameter('diff_gain', 30.0)      # yaw-to-differential thrust scaling
        self.declare_parameter('max_diff', 80.0)       # limit differential to avoid inverted thrusts

        # estimation/filtering params (for vx/vy/omega when twist unreliable)
        self.declare_parameter('v_est_alpha', 0.3)     # low-pass alpha for velocity estimate (0..1)
        self.declare_parameter('yaw_est_alpha', 0.3)   # low-pass for yaw rate estimate (0..1)

        # Read params
        self.rate_hz = float(self.get_parameter('control_rate_hz').value)
        self.path_topic = self.get_parameter('reference_path_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.left_topic = self.get_parameter('left_thrust_topic').value
        self.right_topic = self.get_parameter('right_thrust_topic').value

        self.v_ref = float(self.get_parameter('v_ref').value)
        self.v_ref_max = float(self.get_parameter('v_ref_max').value)
        self.k_v = float(self.get_parameter('k_v').value)
        self.phi_v = float(self.get_parameter('phi_v').value)

        self.Ld = float(self.get_parameter('lookahead_dist').value)
        self.k_ct = float(self.get_parameter('k_ct').value)
        self.k_s = float(self.get_parameter('k_s').value)
        self.k_d = float(self.get_parameter('k_d').value)
        self.phi_yaw = float(self.get_parameter('phi_yaw').value)

        self.u_min = float(self.get_parameter('thrust_min').value)
        self.u_max = float(self.get_parameter('thrust_max').value)
        self.diff_gain = float(self.get_parameter('diff_gain').value)
        self.max_diff = float(self.get_parameter('max_diff').value)

        self.v_est_alpha = float(self.get_parameter('v_est_alpha').value)
        self.yaw_est_alpha = float(self.get_parameter('yaw_est_alpha').value)

        # IO
        self.left_pub = self.create_publisher(Float64, self.left_topic, 10)
        self.right_pub = self.create_publisher(Float64, self.right_topic, 10)
        self.path_sub = self.create_subscription(Path, self.path_topic, self.path_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 10)

        # State
        self.path_pts = []   # list of (x, y)
        self.have_odom = False
        self.x = self.y = 0.0
        self.yaw = 0.0
        # velocity/yawrate estimated from finite diff of pose (not trusting twist)
        self.v_est = 0.0
        self.omega_est = 0.0

        # previous state for finite difference
        self._prev_time = None
        self._prev_x = None
        self._prev_y = None
        self._prev_yaw = None

        # Control loop
        dt = 1.0 / max(1.0, self.rate_hz)
        self.timer = self.create_timer(dt, self.control_step)

        self.get_logger().info(
            f'SMC controller running at {self.rate_hz} Hz. '
            f'Path: {self.path_topic}, Odom: {self.odom_topic}'
        )

    def path_cb(self, msg: Path):
        # store path points (x,y)
        self.path_pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if len(self.path_pts) < 2:
            self.get_logger().warning('Received path with fewer than 2 points.')

    def odom_cb(self, msg: Odometry):
        # update absolute pose
        # try to use header stamp for dt; fallback to node time
        try:
            t = Time.from_msg(msg.header.stamp).nanoseconds / 1e9
        except Exception:
            t = self.get_clock().now().nanoseconds / 1e9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)

        # if we have previous pose/time, compute finite-difference derivatives
        if self._prev_time is not None and t > self._prev_time + 1e-6:
            dt = t - self._prev_time
            dx = x - self._prev_x
            dy = y - self._prev_y
            # distance-based speed estimate (m/s)
            v_new = math.hypot(dx, dy) / dt
            # yaw rate (wrap yaw difference)
            dyaw = wrap_to_pi(yaw - self._prev_yaw)
            omega_new = dyaw / dt

            # low-pass filter estimates
            self.v_est = self.v_est_alpha * v_new + (1.0 - self.v_est_alpha) * self.v_est
            self.omega_est = self.yaw_est_alpha * omega_new + (1.0 - self.yaw_est_alpha) * self.omega_est
        else:
            # first sample: set estimates to zero or to twist if available
            # If odom.twist is present and looks sane, we could use it as fallback.
            try:
                v_twist = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
                omega_twist = msg.twist.twist.angular.z
                self.v_est = v_twist
                self.omega_est = omega_twist
            except Exception:
                pass

        # store current as previous
        self._prev_time = t
        self._prev_x = x
        self._prev_y = y
        self._prev_yaw = yaw

        # publish state
        self.x = x
        self.y = y
        self.yaw = yaw
        self.have_odom = True

    def control_step(self):
        if not self.have_odom:
            return
        if len(self.path_pts) < 2:
            return

        # 1) choose closest point and lookahead target by accumulated arc-length
        ix_closest = self._closest_index(self.x, self.y, self.path_pts)
        ix_target = self._lookahead_index(ix_closest, self.Ld)
        px, py = self.path_pts[ix_closest]
        tx, ty = self.path_pts[ix_target]

        # reference tangent and desired heading (from closest->target or from segment)
        dx = tx - px
        dy = ty - py
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return
        psi_d = math.atan2(dy, dx)

        # 2) errors
        e_psi = wrap_to_pi(psi_d - self.yaw)

        # signed cross-track distance to the line through (px,py)->(tx,ty)
        t_norm = math.hypot(dx, dy)
        tx_h = dx / t_norm
        ty_h = dy / t_norm
        ex = self.x - px
        ey = self.y - py
        e_ct = (-ty_h) * ex + (tx_h) * ey  # left-positive signed distance

        # 3) Sliding surfaces and controls
        # Speed control (surge) — use estimated speed from finite difference
        v = float(self.v_est)
        s_v = (v - self.v_ref)  # drive to zero
        nominal_thrust = self._map_thrust(self.v_ref)
        u_surge = nominal_thrust - self.k_v * sat(s_v / max(1e-6, self.phi_v))
        u_surge = self._clamp(self.u_min, self.u_max, u_surge)

        # Yaw/path control (combine heading and cross-track into one surface)
        s_yaw = e_psi + self.k_ct * math.atan2(e_ct, max(1e-3, self.Ld))
        u_yaw = - self.k_s * sat(s_yaw / max(1e-6, self.phi_yaw)) - self.k_d * float(self.omega_est)

        # Map yaw control to differential thrust
        diff = self.diff_gain * u_yaw
        # clamp differential to avoid huge opposite-sign thrusts
        diff = max(-self.max_diff, min(self.max_diff, diff))

        left_cmd = self._clamp(self.u_min, self.u_max, u_surge - diff)
        right_cmd = self._clamp(self.u_min, self.u_max, u_surge + diff)

        # 4) Publish commands
        self.left_pub.publish(Float64(data=float(left_cmd)))
        self.right_pub.publish(Float64(data=float(right_cmd)))

    def _closest_index(self, x, y, pts):
        best_i = 0
        best_d2 = float('inf')
        for i, (px, py) in enumerate(pts):
            d2 = (x - px) * (x - px) + (y - py) * (y - py)
            if d2 < best_d2:
                best_i, best_d2 = i, d2
        return best_i

    def _lookahead_index(self, idx_start, lookahead_dist):
        # accumulate along path until distance >= lookahead_dist
        if idx_start >= len(self.path_pts) - 1:
            return idx_start
        acc = 0.0
        last_x, last_y = self.path_pts[idx_start]
        for i in range(idx_start + 1, len(self.path_pts)):
            x_i, y_i = self.path_pts[i]
            seg = math.hypot(x_i - last_x, y_i - last_y)
            acc += seg
            last_x, last_y = x_i, y_i
            if acc >= lookahead_dist:
                return i
        # if not enough length, return last point
        return len(self.path_pts) - 1

    def _map_thrust(self, v_ref):
        # Linear feedforward: maps v_ref in [0, v_ref_max] to thrust in [u_min, u_max]
        v_ref_max = max(1e-3, float(self.v_ref_max))
        frac = max(0.0, min(1.0, v_ref / v_ref_max))
        return self.u_min + frac * (self.u_max - self.u_min)

    @staticmethod
    def _clamp(a, b, x):
        lo, hi = (a, b) if a <= b else (b, a)
        return max(lo, min(hi, x))


def main():
    rclpy.init()
    node = SMCUSV()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
