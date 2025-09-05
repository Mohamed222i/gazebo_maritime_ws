#!/usr/bin/env python3
"""
LOS + PD/PI WAM-V Controller (ROS2, Python)

Guidance: distance-based LOS target selection (same projection + lookahead).
Heading control: PD on heading (P on yaw_err, D on measured yaw rate).
Surge control: PI on forward speed with anti-windup.
Thruster mapping: same mapping F_total + M_cmd -> left/right thrusts.
Rate limiting and clipping included. Debug topics published for tuning.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3
from std_msgs.msg import Float64
from visualization_msgs.msg import Marker
from std_srvs.srv import Empty
import math
from typing import List, Tuple

def wrap_to_pi(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi

class LOSWamV(Node):
    def __init__(self):
        super().__init__('los_control')

        # --- parameters ---
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('path_topic', '/reference_path')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('control_rate', 20.0)

        # physical params
        self.declare_parameter('mass', 180.0)
        self.declare_parameter('Izz', 446.0)
        self.declare_parameter('thruster_separation', 2.05427)
        self.declare_parameter('thruster_half', 1.027135)

        # actuation / safety
        self.declare_parameter('left_thrust_topic', '/wamv/left_thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/right_thrust')
        self.declare_parameter('max_thrust', 200.0)
        self.declare_parameter('max_thrust_rate', 100.0)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # guidance / control gains
        self.declare_parameter('desired_speed', 0.8)
        self.declare_parameter('k_speed_p', 70.0)
        self.declare_parameter('k_speed_i', 20.0)
        self.declare_parameter('k_psi', 5.0)   # heading P
        self.declare_parameter('k_d', 0.2)     # heading D (on angular rate)
        self.declare_parameter('k_moment', 10.0) # maps r_des to moment

        self.declare_parameter('lookahead_distance', 0.4)
        self.declare_parameter('odom_yaw_compensation', 0.0)

        # executed path publishing
        self.declare_parameter('executed_path_topic', '/viz/actual_path')
        self.declare_parameter('executed_path_frame', 'map')
        self.declare_parameter('executed_path_max_points', 2000)
        self.declare_parameter('executed_path_publish_rate', 5.0)
        self.declare_parameter('publish_executed_marker', True)
        self.declare_parameter('executed_marker_topic', '/los/executed_path_marker')

        # anti-windup integrator limits
        self.declare_parameter('speed_integrator_limit', 200.0)

        # --- read parameters ---
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.path_topic = self.get_parameter('path_topic').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.rate_hz = float(self.get_parameter('control_rate').get_parameter_value().double_value)

        self.mass = float(self.get_parameter('mass').get_parameter_value().double_value)
        self.Izz = float(self.get_parameter('Izz').get_parameter_value().double_value)
        self.thruster_separation = float(self.get_parameter('thruster_separation').get_parameter_value().double_value)
        self.thruster_half = float(self.get_parameter('thruster_half').get_parameter_value().double_value)

        self.left_topic = self.get_parameter('left_thrust_topic').get_parameter_value().string_value
        self.right_topic = self.get_parameter('right_thrust_topic').get_parameter_value().string_value
        self.max_thrust = float(self.get_parameter('max_thrust').get_parameter_value().double_value)
        self.max_thrust_rate = float(self.get_parameter('max_thrust_rate').get_parameter_value().double_value)
        self.invert_left = bool(self.get_parameter('invert_left').get_parameter_value().bool_value)
        self.invert_right = bool(self.get_parameter('invert_right').get_parameter_value().bool_value)

        self.desired_speed = float(self.get_parameter('desired_speed').get_parameter_value().double_value)
        self.k_speed_p = float(self.get_parameter('k_speed_p').get_parameter_value().double_value)
        self.k_speed_i = float(self.get_parameter('k_speed_i').get_parameter_value().double_value)
        self.k_psi = float(self.get_parameter('k_psi').get_parameter_value().double_value)
        self.k_d = float(self.get_parameter('k_d').get_parameter_value().double_value)
        self.k_moment = float(self.get_parameter('k_moment').get_parameter_value().double_value)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').get_parameter_value().double_value)
        self.odom_yaw_compensation = float(self.get_parameter('odom_yaw_compensation').get_parameter_value().double_value)

        self.executed_path_topic = self.get_parameter('executed_path_topic').get_parameter_value().string_value
        self.executed_path_frame = self.get_parameter('executed_path_frame').get_parameter_value().string_value
        self.executed_path_max_points = int(self.get_parameter('executed_path_max_points').get_parameter_value().integer_value)
        self.executed_path_publish_rate = float(self.get_parameter('executed_path_publish_rate').get_parameter_value().double_value)
        self.publish_executed_marker = bool(self.get_parameter('publish_executed_marker').get_parameter_value().bool_value)
        self.executed_marker_topic = self.get_parameter('executed_marker_topic').get_parameter_value().string_value

        self.speed_integrator_limit = float(self.get_parameter('speed_integrator_limit').get_parameter_value().double_value)

        # --- internal state ---
        self.path_points: List[Tuple[float,float]] = []
        self.path_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_u = 0.0
        self.current_r = 0.0
        self.odom_received = False

        # executed path buffer
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.executed_path_frame
        self.executed_path.poses = []

        # last commands (for rate limiting)
        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        # speed integrator
        self.speed_integrator = 0.0

        qos = QoSProfile(depth=10)
        # subscribers
        self.create_subscription(Path, self.path_topic, self.path_cb, qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)

        # publishers
        self.left_pub = self.create_publisher(Float64, self.left_topic, qos)
        self.right_pub = self.create_publisher(Float64, self.right_topic, qos)
        self.target_pub = self.create_publisher(PoseStamped, '/los/target_point', qos)
        self.marker_pub = self.create_publisher(Marker, '/los/control_marker', qos)
        self.executed_pub = self.create_publisher(Path, self.executed_path_topic, qos)
        self.executed_marker_pub = self.create_publisher(Marker, self.executed_marker_topic, qos) if self.publish_executed_marker else None

        # service to clear executed path
        self.create_service(Empty, '/los/clear_executed_path', self.clear_executed_path_cb)

        # timers
        self.control_timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.exec_pub_timer = self.create_timer(1.0 / self.executed_path_publish_rate, self.publish_executed_path)

        self.get_logger().info('LOS WAM-V controller started.')

        # debug publishers
        self.pub_yaw_err = self.create_publisher(Float64, '/los/debug/yaw_err', 10)
        self.pub_cross_track = self.create_publisher(Float64, '/los/debug/cross_track', 10)
        self.pub_r_des = self.create_publisher(Float64, '/los/debug/r_des', 10)
        self.pub_F_total = self.create_publisher(Float64, '/los/debug/F_total', 10)
        self.pub_left_cmd = self.create_publisher(Float64, '/los/debug/left_cmd', 10)
        self.pub_right_cmd = self.create_publisher(Float64, '/los/debug/right_cmd', 10)
        self.pub_current_u = self.create_publisher(Float64, '/los/debug/current_u', 10)
        self.pub_desired_speed = self.create_publisher(Float64, '/los/debug/desired_speed', 10)

    # ---------------- callbacks ----------------
    def path_cb(self, msg: Path):
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        if len(pts) >= 2:
            self.path_points = pts
            self.path_received = True
            self.get_logger().debug(f"Received path with {len(pts)} pts")
        else:
            self.get_logger().warn('Reference path contains fewer than 2 points.')

    def odom_cb(self, msg: Odometry):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        self.current_yaw = wrap_to_pi(raw_yaw + self.odom_yaw_compensation)
        self.current_u = msg.twist.twist.linear.x
        self.current_r = msg.twist.twist.angular.z
        self.odom_received = True

        # record executed path
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.executed_path_frame
        ps.pose = msg.pose.pose
        cz = math.sin(self.current_yaw / 2.0); cw = math.cos(self.current_yaw / 2.0)
        ps.pose.orientation.z = cz
        ps.pose.orientation.w = cw
        self.executed_path.poses.append(ps)
        if len(self.executed_path.poses) > self.executed_path_max_points:
            self.executed_path.poses = self.executed_path.poses[-self.executed_path_max_points:]

    def clear_executed_path_cb(self, req, res):
        self.executed_path.poses = []
        self.get_logger().info('Cleared executed path.')
        return res

    # ---------------- utility: distance-based lookahead ----------------
    def find_projection_and_along(self):
        if len(self.path_points) < 2:
            return (self.current_x, self.current_y, 0, 0.0)
        best_seg = 0
        best_dist = float('inf')
        best_proj = (self.path_points[0][0], self.path_points[0][1])
        for i in range(len(self.path_points)-1):
            x1,y1 = self.path_points[i]
            x2,y2 = self.path_points[i+1]
            dx = x2 - x1; dy = y2 - y1
            seg_len2 = dx*dx + dy*dy
            if seg_len2 == 0:
                continue
            t = ((self.current_x - x1)*dx + (self.current_y - y1)*dy) / seg_len2
            t_clamped = max(0.0, min(1.0, t))
            proj_x = x1 + t_clamped * dx
            proj_y = y1 + t_clamped * dy
            d = math.hypot(self.current_x - proj_x, self.current_y - proj_y)
            if d < best_dist:
                best_dist = d
                best_seg = i
                best_proj = (proj_x, proj_y)
        return (best_proj[0], best_proj[1], best_seg, best_dist)

    def compute_target_along_lookahead(self):
        if len(self.path_points) < 2:
            return (self.current_x, self.current_y, self.current_yaw, 0.0)
        proj_x, proj_y, seg_idx, proj_dist = self.find_projection_and_along()

        # compute along
        along = 0.0
        for i in range(seg_idx):
            x1,y1 = self.path_points[i]; x2,y2 = self.path_points[i+1]
            along += math.hypot(x2-x1, y2-y1)
        # add projection fraction
        x1,y1 = self.path_points[seg_idx]; x2,y2 = self.path_points[seg_idx+1]
        seg_len = math.hypot(x2-x1, y2-y1)
        if seg_len > 1e-9:
            t = ((proj_x - x1)*(x2-x1) + (proj_y - y1)*(y2-y1)) / (seg_len*seg_len)
            t = max(0.0, min(1.0, t))
            along += t*seg_len

        target_along = along + self.lookahead_distance

        seg_lengths = [math.hypot(self.path_points[i+1][0]-self.path_points[i][0],
                                  self.path_points[i+1][1]-self.path_points[i][1])
                       for i in range(len(self.path_points)-1)]
        total_len = sum(seg_lengths)
        if total_len <= 0.0:
            return (self.path_points[-1][0], self.path_points[-1][1], 0.0, proj_dist)
        if target_along >= total_len:
            tx,ty = self.path_points[-1]
            prevx,prevy = self.path_points[-2]
            path_yaw = math.atan2(ty-prevy, tx-prevx)
            return (tx, ty, path_yaw, proj_dist)
        accum = 0.0
        for i,L in enumerate(seg_lengths):
            if accum + L >= target_along:
                seg_offset = (target_along - accum) / max(1e-9, L)
                x1,y1 = self.path_points[i]; x2,y2 = self.path_points[i+1]
                tx = x1 + seg_offset * (x2-x1)
                ty = y1 + seg_offset * (y2-y1)
                path_yaw = math.atan2(y2-y1, x2-x1)
                return (tx, ty, path_yaw, proj_dist)
            accum += L
        tx,ty = self.path_points[-1]
        prevx,prevy = self.path_points[-2]
        path_yaw = math.atan2(ty-prevy, tx-prevx)
        return (tx, ty, path_yaw, proj_dist)

    # ---------------- publish executed path ----------------
    def publish_executed_path(self):
        if len(self.executed_path.poses) == 0:
            return
        self.executed_path.header.stamp = self.get_clock().now().to_msg()
        self.executed_pub.publish(self.executed_path)
        if self.executed_marker_pub is not None:
            mk = Marker()
            mk.header.frame_id = self.executed_path_frame
            mk.header.stamp = self.get_clock().now().to_msg()
            mk.ns = 'executed_path'
            mk.id = 0
            mk.type = Marker.LINE_STRIP
            mk.action = Marker.ADD
            mk.scale = Vector3(x=0.12, y=0.0, z=0.0)
            mk.color.a = 1.0
            mk.color.r = 0.9; mk.color.g = 0.2; mk.color.b = 0.2
            mk.points = []
            for p in self.executed_path.poses:
                pt = Point()
                pt.x = p.pose.position.x; pt.y = p.pose.position.y; pt.z = p.pose.position.z if hasattr(p.pose.position,'z') else 0.0
                mk.points.append(pt)
            self.executed_marker_pub.publish(mk)

    # ---------------- core control loop ----------------
    def control_loop(self):
        if not (self.odom_received and self.path_received and len(self.path_points) >= 2):
            return

        # target
        tx,ty,path_yaw, offtrack = self.compute_target_along_lookahead()

        # heading error
        yaw_err = wrap_to_pi(path_yaw - self.current_yaw)

        # cross track (signed) for debugging
        vx = self.current_x - tx
        vy = self.current_y - ty
        cross_track = -math.sin(path_yaw)*vx + math.cos(path_yaw)*vy

        # ---------------- heading PD -> desired yaw-rate r_des ----------------
        # P on yaw error, D on measured yaw rate (damping)
        r_des = self.k_psi * yaw_err - self.k_d * self.current_r

        # map to moment
        M_cmd = self.k_moment * r_des

        # ---------------- surge PI ----------------
        speed_err = self.desired_speed - self.current_u
        dt = max(1e-6, self.get_clock().now().nanoseconds * 1e-9 - self.last_time)
        # integrate with anti-windup
        self.speed_integrator += speed_err * dt
        # clamp integrator
        if self.speed_integrator > self.speed_integrator_limit:
            self.speed_integrator = self.speed_integrator_limit
        elif self.speed_integrator < -self.speed_integrator_limit:
            self.speed_integrator = -self.speed_integrator_limit
        F_total = self.k_speed_p * speed_err + self.k_speed_i * self.speed_integrator

        # ---------------- safety and thruster mapping ----------------
        # clamp M_cmd (optional)
        M_cmd_limit = 80.0
        if M_cmd > M_cmd_limit: M_cmd = M_cmd_limit
        if M_cmd < -M_cmd_limit: M_cmd = -M_cmd_limit

        d = self.thruster_half if abs(self.thruster_half) > 1e-6 else 1.0
        delta = M_cmd / d
        left_cmd = (F_total / 2.0) - (delta / 2.0)
        right_cmd = (F_total / 2.0) + (delta / 2.0)

        if self.invert_left:
            left_cmd = -left_cmd
        if self.invert_right:
            right_cmd = -right_cmd

        # clip to max thrust
        left_cmd = max(-self.max_thrust, min(self.max_thrust, left_cmd))
        right_cmd = max(-self.max_thrust, min(self.max_thrust, right_cmd))

        # rate limit thrusters
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = max(1e-6, now - self.last_time)
        max_delta = self.max_thrust_rate * dt
        left_cmd = self.last_left_cmd + max(-max_delta, min(max_delta, left_cmd - self.last_left_cmd))
        right_cmd = self.last_right_cmd + max(-max_delta, min(max_delta, right_cmd - self.last_right_cmd))
        self.last_time = now
        self.last_left_cmd = left_cmd
        self.last_right_cmd = right_cmd

        # publish thrusters
        lmsg = Float64(); rmsg = Float64()
        lmsg.data = float(left_cmd); rmsg.data = float(right_cmd)
        self.left_pub.publish(lmsg); self.right_pub.publish(rmsg)

        # publish target pose for RViz
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(tx); ps.pose.position.y = float(ty); ps.pose.position.z = 0.0
        cz = math.sin(path_yaw/2.0); cw = math.cos(path_yaw/2.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=cz, w=cw)
        self.target_pub.publish(ps)

        # control marker
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'los'
        mk.id = 0
        mk.type = Marker.ARROW
        mk.action = Marker.ADD
        mk.pose.position.x = self.current_x
        mk.pose.position.y = self.current_y
        mk.pose.position.z = 0.2
        mk.pose.orientation = Quaternion(x=0.0,y=0.0,z=math.sin(self.current_yaw/2.0), w=math.cos(self.current_yaw/2.0))
        length = max(0.2, min(3.0, abs(M_cmd) / 10.0))
        mk.scale = Vector3(x=length, y=0.07, z=0.07)
        mk.color.a = 1.0
        mk.color.r = 1.0 if M_cmd >= 0 else 0.2
        mk.color.g = 0.2 if M_cmd >= 0 else 1.0
        mk.color.b = 0.2
        self.marker_pub.publish(mk)

        # publish debug floats
        def pub(name, value, pub_obj):
            m = Float64(); m.data = float(value); pub_obj.publish(m)
        pub('yaw_err', yaw_err, self.pub_yaw_err)
        pub('cross_track', cross_track, self.pub_cross_track)
        pub('r_des', r_des, self.pub_r_des)
        pub('F_total', F_total, self.pub_F_total)
        pub('left_cmd', left_cmd, self.pub_left_cmd)
        pub('right_cmd', right_cmd, self.pub_right_cmd)
        pub('current_u', self.current_u, self.pub_current_u)
        pub('desired_speed', self.desired_speed, self.pub_desired_speed)

    # ---------------- end control_loop ----------------

def main(args=None):
    rclpy.init(args=args)
    node = LOSWamV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
