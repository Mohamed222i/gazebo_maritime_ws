"""
SMC WAM-V Controller (ROS2, Python)

Features:
 - Sliding Mode Controller (SMC) for waypoint/path following (2D)
 - Distance-based lookahead target selection (smooth)
 - Thruster mapping: surge + yaw moment -> left/right thrusts
 - Rate-limited and clipped thruster outputs
 - Executed path recording & publishing for RViz (/viz/actual_path)
 - Odometry yaw compensation param (useful because SDF odom plugin applied rpy_offset = +pi/2)
 - Conservative physical defaults derived from your SDF

Added in this version (damping):
 - Linear viscous damping on surge command (k_d_speed)
 - Rotational viscous damping on yaw moment (k_d_yaw), applied using a filtered yaw-rate
 - First-order low-pass on measured yaw-rate to avoid noisy derivative feedback (r_filter_tau)
 - Debug publishers for damping contributions

Default tuned parameters (physically-informed) kept and extended with damping defaults:
 - thruster_separation = 2.05427 m (extracted from SDF)
 - thruster_half d = 1.027135 m
 - mass = 180 kg (for reference)
 - Izz = 446 kg·m²
 - desired_speed = 0.6 m/s
 - max_thrust = 140.0 (safe)
 - max_thrust_rate = 40.0 N/s
 - k_speed = 120.0
 - k_psi = 0.6
 - k_s = 6.0
 - epsilon = 0.3
 - k_moment = 5.0
 - lookahead_distance = 4.0 m
 - odom_yaw_compensation = -1.570796 (subtract pi/2 from raw odom yaw)
 - k_d_speed = 20.0  # linear viscous damping (higher -> stronger suppression of speed oscillations)
 - k_d_yaw = 60.0    # rotational viscous damping (Nm per rad/s)
 - r_filter_tau = 0.15 # seconds for yaw-rate low-pass

Notes:
 - Set k_d_speed or k_d_yaw to 0.0 to disable the corresponding damping.
 - Tune gradually: increase damping until oscillations reduced, then reduce if response becomes sluggish.
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


def sign_sat(s: float) -> float:
    if s > 0.0:
        return 1.0
    elif s < 0.0:
        return -1.0
    else:
        return 0.0


class SMCWamV(Node):
    def __init__(self):
        super().__init__('smc_control')

        # --- declare parameters (defaults chosen from SDF + safe tuning) ---
        self.declare_parameter('odom_topic', '/odometry/filtered')
        self.declare_parameter('path_topic', '/reference_path')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('control_rate', 20.0)

        # Physical params from SDF
        self.declare_parameter('mass', 180.0)
        self.declare_parameter('Izz', 446.0)
        self.declare_parameter('thruster_separation', 2.05427)  # meters (from SDF)
        self.declare_parameter('thruster_half', 1.027135)      # d

        # Actuation + safety
        self.declare_parameter('left_thrust_topic', '/wamv/left_thrust')
        self.declare_parameter('right_thrust_topic', '/wamv/right_thrust')
        self.declare_parameter('max_thrust', 140.0)
        self.declare_parameter('max_thrust_rate', 40.0)   # per second
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)

        # Controller defaults (conservative / physically-informed)
        self.declare_parameter('desired_speed', 0.5)
        self.declare_parameter('k_speed', 80.0)
        self.declare_parameter('k_psi', 1.0)
        self.declare_parameter('k_s', 5.0)
        self.declare_parameter('epsilon', 0.3)
        self.declare_parameter('k_moment', 2.0)
        self.declare_parameter('lambda_ct', 2.0)


        # Damping (new)
        self.declare_parameter('k_d_speed', 50.0)   # linear viscous damping (applied to surge)
        self.declare_parameter('k_d_yaw', 20.0)     # rotational viscous damping (applied as torque)
        self.declare_parameter('r_filter_tau', 0.2) # low-pass time constant (s) for yaw-rate

        # Lookahead
        self.declare_parameter('lookahead_distance', 0.4)

        # Odometry compensation (SDF odom rpy_offset = +pi/2); set 0 to disable
        self.declare_parameter('odom_yaw_compensation', 3.14)  # subtract π/2 by default

        # Executed path publishing
        self.declare_parameter('executed_path_topic', '/viz/actual_path')
        self.declare_parameter('executed_path_frame', 'map')   # stamp published path in this frame
        self.declare_parameter('executed_path_max_points', 2000)
        self.declare_parameter('executed_path_publish_rate', 5.0)
        self.declare_parameter('publish_executed_marker', True)
        self.declare_parameter('executed_marker_topic', '/smc/executed_path_marker')

        # --- read params into attributes ---
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
        self.k_speed = float(self.get_parameter('k_speed').get_parameter_value().double_value)
        self.k_psi = float(self.get_parameter('k_psi').get_parameter_value().double_value)
        self.k_s = float(self.get_parameter('k_s').get_parameter_value().double_value)
        self.epsilon = float(self.get_parameter('epsilon').get_parameter_value().double_value)
        self.k_moment = float(self.get_parameter('k_moment').get_parameter_value().double_value)
        self.lambda_ct = float(self.get_parameter('lambda_ct').get_parameter_value().double_value)

        # damping params
        self.k_d_speed = float(self.get_parameter('k_d_speed').get_parameter_value().double_value)
        self.k_d_yaw = float(self.get_parameter('k_d_yaw').get_parameter_value().double_value)
        self.r_filter_tau = float(self.get_parameter('r_filter_tau').get_parameter_value().double_value)

        self.lookahead_distance = float(self.get_parameter('lookahead_distance').get_parameter_value().double_value)
        self.odom_yaw_compensation = float(self.get_parameter('odom_yaw_compensation').get_parameter_value().double_value)

        self.executed_path_topic = self.get_parameter('executed_path_topic').get_parameter_value().string_value
        self.executed_path_frame = self.get_parameter('executed_path_frame').get_parameter_value().string_value
        self.executed_path_max_points = int(self.get_parameter('executed_path_max_points').get_parameter_value().integer_value)
        self.executed_path_publish_rate = float(self.get_parameter('executed_path_publish_rate').get_parameter_value().double_value)
        self.publish_executed_marker = bool(self.get_parameter('publish_executed_marker').get_parameter_value().bool_value)
        self.executed_marker_topic = self.get_parameter('executed_marker_topic').get_parameter_value().string_value

        # --- internal state ---
        self.path_points: List[Tuple[float,float]] = []
        self.path_received = False
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.current_u = 0.0
        self.current_r = 0.0  # raw yaw-rate from odom
        self.filtered_r = 0.0 # low-pass filtered yaw-rate used for damping
        self.odom_received = False

        # executed path buffer
        self.executed_path = Path()
        self.executed_path.header.frame_id = self.executed_path_frame
        self.executed_path.poses = []

        # last commands (for rate limiting)
        self.last_left_cmd = 0.0
        self.last_right_cmd = 0.0
        self.last_time = self.get_clock().now().nanoseconds * 1e-9

        qos = QoSProfile(depth=10)
        # subscribers
        self.create_subscription(Path, self.path_topic, self.path_cb, qos)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, qos)

        # publishers
        self.left_pub = self.create_publisher(Float64, self.left_topic, qos)
        self.right_pub = self.create_publisher(Float64, self.right_topic, qos)
        self.target_pub = self.create_publisher(PoseStamped, '/smc/target_point', qos)
        self.marker_pub = self.create_publisher(Marker, '/smc/control_marker', qos)
        self.executed_pub = self.create_publisher(Path, self.executed_path_topic, qos)
        self.executed_marker_pub = self.create_publisher(Marker, self.executed_marker_topic, qos) if self.publish_executed_marker else None

        # service to clear executed path
        self.create_service(Empty, '/smc/clear_executed_path', self.clear_executed_path_cb)

        # timers
        self.control_timer = self.create_timer(1.0 / self.rate_hz, self.control_loop)
        self.exec_pub_timer = self.create_timer(1.0 / self.executed_path_publish_rate, self.publish_executed_path)

        self.get_logger().info('SMC WAM-V controller started (physically-informed defaults loaded). Damping enabled.')



        # debug publishers for plotting / tuning
        self.pub_s = self.create_publisher(Float64, '/smc/debug/s', 10)
        self.pub_yaw_err = self.create_publisher(Float64, '/smc/debug/yaw_err', 10)
        self.pub_cross_track = self.create_publisher(Float64, '/smc/debug/cross_track', 10)
        self.pub_switching = self.create_publisher(Float64, '/smc/debug/switching', 10)
        self.pub_r_cmd = self.create_publisher(Float64, '/smc/debug/r_cmd', 10)
        self.pub_F_total = self.create_publisher(Float64, '/smc/debug/F_total', 10)
        self.pub_M_cmd = self.create_publisher(Float64, '/smc/debug/M_cmd', 10)
        self.pub_left_cmd = self.create_publisher(Float64, '/smc/debug/left_cmd', 10)
        self.pub_right_cmd = self.create_publisher(Float64, '/smc/debug/right_cmd', 10)
        self.pub_current_u = self.create_publisher(Float64, '/smc/debug/current_u', 10)
        self.pub_desired_speed = self.create_publisher(Float64, '/smc/debug/desired_speed', 10)
        self.pub_offtrack = self.create_publisher(Float64, '/smc/debug/offtrack', 10)
        self.pub_lookahead = self.create_publisher(Float64, '/smc/debug/lookahead', 10)
        self.pub_epsilon = self.create_publisher(Float64, '/smc/debug/eps', 10)
        # damping debug pubs
        self.pub_damping_F = self.create_publisher(Float64, '/smc/debug/damping_F', 10)
        self.pub_damping_M = self.create_publisher(Float64, '/smc/debug/damping_M', 10)
        self.pub_r_filtered = self.create_publisher(Float64, '/smc/debug/r_filtered', 10)


    # ---------------- callbacks ----------------
    def path_cb(self, msg: Path):
        pts = []
        for ps in msg.poses:
            pts.append((ps.pose.position.x, ps.pose.position.y))
        if len(pts) >= 2:
            self.path_points = pts
            self.path_received = True
            self.get_logger().debug(f"Received reference path with {len(pts)} pts (frame={msg.header.frame_id})")
        else:
            self.get_logger().warn('Reference path contains fewer than 2 points.')

    def odom_cb(self, msg: Odometry):
        # extract pose and apply optional yaw compensation (to fix rpy_offset in SDF)
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # quaternion -> yaw
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)
        # apply compensation (add value; default is -pi/2 because SDF odom plugin added +pi/2)
        self.current_yaw = wrap_to_pi(raw_yaw + self.odom_yaw_compensation)
        # linear speed (forward in body frame)
        self.current_u = msg.twist.twist.linear.x
        # raw yaw-rate from odometry (body z)
        self.current_r = msg.twist.twist.angular.z
        self.odom_received = True

        # record executed path (PoseStamped)
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.executed_path_frame
        ps.pose = msg.pose.pose
        # optionally fix pose orientation yaw to compensated yaw (so robot arrow in RViz matches)
        # recompute quaternion for compensated yaw:
        cz = math.sin(self.current_yaw / 2.0)
        cw = math.cos(self.current_yaw / 2.0)
        ps.pose.orientation.z = cz
        ps.pose.orientation.w = cw
        self.executed_path.poses.append(ps)
        if len(self.executed_path.poses) > self.executed_path_max_points:
            self.executed_path.poses = self.executed_path.poses[-self.executed_path_max_points:]

    # clear executed path service
    def clear_executed_path_cb(self, req, res):
        self.executed_path.poses = []
        self.get_logger().info('Cleared executed path buffer.')
        return res

    # ---------------- utility: distance-based lookahead ----------------
    def find_projection_and_along(self):
        # returns (proj_x, proj_y, seg_idx, proj_dist)
        if len(self.path_points) < 2:
            return (self.current_x, self.current_y, 0, 0.0)
        best_seg = 0
        best_dist = float('inf')
        best_proj = (self.path_points[0][0], self.path_points[0][1])
        best_t = 0.0
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
                best_t = t_clamped
        return (best_proj[0], best_proj[1], best_seg, best_dist)

    def compute_target_along_lookahead(self):
        # compute along coordinate of projection then add lookahead_distance to find target point
        if len(self.path_points) < 2:
            return (self.current_x, self.current_y, self.current_yaw, 0.0)

        proj_x, proj_y, seg_idx, proj_dist = self.find_projection_and_along()

        # compute along coordinate up to projection
        along = 0.0
        for i in range(seg_idx):
            x1,y1 = self.path_points[i]; x2,y2 = self.path_points[i+1]
            along += math.hypot(x2-x1, y2-y1)
        # add projection distance along its segment
        x1,y1 = self.path_points[seg_idx]; x2,y2 = self.path_points[seg_idx+1]
        seg_len = math.hypot(x2-x1, y2-y1)
        # compute projection t on this segment
        if seg_len > 1e-9:
            t = ((proj_x - x1)* (x2-x1) + (proj_y - y1)*(y2-y1)) / (seg_len*seg_len)
            t = max(0.0, min(1.0, t))
            along += t * seg_len

        target_along = along + self.lookahead_distance

        # compute segment lengths
        seg_lengths = [math.hypot(self.path_points[i+1][0]-self.path_points[i][0],
                                  self.path_points[i+1][1]-self.path_points[i][1])
                       for i in range(len(self.path_points)-1)]
        total_len = sum(seg_lengths)
        if total_len <= 0.0:
            return (self.path_points[-1][0], self.path_points[-1][1], 0.0, proj_dist)

        if target_along >= total_len:
            # clamp to last point
            tx,ty = self.path_points[-1]
            prevx,prevy = self.path_points[-2]
            path_yaw = math.atan2(ty-prevy, tx-prevx)
            return (tx, ty, path_yaw, proj_dist)

        # find the segment containing target_along
        accum = 0.0
        for i, L in enumerate(seg_lengths):
            if accum + L >= target_along:
                seg_offset = (target_along - accum) / max(1e-9, L)
                x1,y1 = self.path_points[i]; x2,y2 = self.path_points[i+1]
                tx = x1 + seg_offset * (x2-x1)
                ty = y1 + seg_offset * (y2-y1)
                path_yaw = math.atan2(y2-y1, x2-x1)
                return (tx, ty, path_yaw, proj_dist)
            accum += L

        # fallback
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
            mk.color.r = 0.9
            mk.color.g = 0.2
            mk.color.b = 0.2
            mk.points = []
            for p in self.executed_path.poses:
                pt = Point()
                pt.x = p.pose.position.x
                pt.y = p.pose.position.y
                pt.z = p.pose.position.z if hasattr(p.pose.position, 'z') else 0.0
                mk.points.append(pt)
            self.executed_marker_pub.publish(mk)

    # ---------------- core SMC control loop ----------------
    def control_loop(self):
        if not (self.odom_received and self.path_received and len(self.path_points) >= 2):
            return

        # get lookahead target and projection distance (offtrack)
        tx, ty, path_yaw, offtrack = self.compute_target_along_lookahead()

        # heading error: desired - current
        yaw_err = wrap_to_pi(path_yaw - self.current_yaw)

        # compute cross-track approx using projection onto local tangent (signed)
        vx = self.current_x - tx
        vy = self.current_y - ty
        cross_track = -math.sin(path_yaw) * vx + math.cos(path_yaw) * vy

        # sliding variable
        s = yaw_err + self.lambda_ct * cross_track

        # switching term with boundary layer
        if abs(s) > self.epsilon:
            switching = self.k_s *  math.tanh(s / self.epsilon)
        else:
            switching = self.k_s * (s / self.epsilon)

        # desired yaw-rate like command (rad/s)
        r_cmd = - self.k_psi * yaw_err - switching

        # surge: simple P controller mapping speed error to thrust (units of node's thrust command)
        speed_err = self.desired_speed - self.current_u

        # linear viscous damping term: -k_d_speed * u
        damping_F = self.k_d_speed * self.current_u

        F_total = self.k_speed * speed_err - damping_F

        # update filtered yaw-rate (first-order LPF) using DT from last loop
        now = self.get_clock().now().nanoseconds * 1e-9
        dt = max(1e-6, now - self.last_time)
        alpha = dt / (self.r_filter_tau + dt)
        # filtered_r tracks the measured current_r
        self.filtered_r += alpha * (self.current_r - self.filtered_r)

        # yaw moment command
        # damping torque = -k_d_yaw * filtered_r
        damping_M = self.k_d_yaw * self.filtered_r
        M_cmd = self.k_moment * r_cmd - damping_M

        # safety limits (insert after r_cmd and M_cmd computed)
        r_cmd_limit = 2.0   # rad/s (safe)
        M_cmd_limit = 80.0  # N*m (safe-ish in sim)


        
        # clamp r_cmd
        #if r_cmd > r_cmd_limit:
        #    r_cmd = r_cmd_limit
        #elif r_cmd < -r_cmd_limit:
        #    r_cmd = -r_cmd_limit
        #
        ## recompute M_cmd from (possibly) clamped r_cmd
        #M_cmd = self.k_moment * r_cmd
        ## clamp M_cmd just in case
        #if M_cmd > M_cmd_limit:
        #    M_cmd = M_cmd_limit
        #elif M_cmd < -M_cmd_limit:
        #    M_cmd = -M_cmd_limit




        # thruster mapping: F_total = FL + FR ; M_cmd = (FR - FL) * d
        d = self.thruster_half
        if abs(d) < 1e-6:
            d = 1.0
        delta = M_cmd / d  # FR - FL
        left_cmd = (F_total / 2.0) - (delta / 2.0)
        right_cmd = (F_total / 2.0) + (delta / 2.0)

        # apply inversion flags
        if self.invert_left:
            left_cmd = -left_cmd
        if self.invert_right:
            right_cmd = -right_cmd

        # clip to max thrust
        left_cmd = max(-self.max_thrust, min(self.max_thrust, left_cmd))
        right_cmd = max(-self.max_thrust, min(self.max_thrust, right_cmd))

        # rate limit thrusters
        max_delta = self.max_thrust_rate * dt
        left_cmd = self.last_left_cmd + max(-max_delta, min(max_delta, left_cmd - self.last_left_cmd))
        right_cmd = self.last_right_cmd + max(-max_delta, min(max_delta, right_cmd - self.last_right_cmd))
        self.last_time = now
        self.last_left_cmd = left_cmd
        self.last_right_cmd = right_cmd

        # publish thruster commands
        lmsg = Float64(); rmsg = Float64()
        lmsg.data = float(left_cmd); rmsg.data = float(right_cmd)
        self.left_pub.publish(lmsg)
        self.right_pub.publish(rmsg)

        # publish a target pose for RViz
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.frame_id
        ps.pose.position.x = float(tx)
        ps.pose.position.y = float(ty)
        ps.pose.position.z = 0.0
        cz = math.sin(path_yaw/2.0); cw = math.cos(path_yaw/2.0)
        ps.pose.orientation = Quaternion(x=0.0, y=0.0, z=cz, w=cw)
        self.target_pub.publish(ps)

        # publish control marker (arrow) indicating yaw moment
        mk = Marker()
        mk.header.frame_id = self.frame_id
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns = 'smc'
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

        # (optional) light debug logging
        self.get_logger().debug(f"s={s:.3f}, yaw_err={yaw_err:.3f}, ct={cross_track:.3f}, F={F_total:.1f}, M={M_cmd:.1f}, L={left_cmd:.1f}, R={right_cmd:.1f}")

        # Publish debug floats for plotting
        msg = Float64()
        msg.data = float(s); self.pub_s.publish(msg)

        msg.data = float(yaw_err); self.pub_yaw_err.publish(msg)
        msg.data = float(cross_track); self.pub_cross_track.publish(msg)
        msg.data = float(switching); self.pub_switching.publish(msg)
        msg.data = float(r_cmd); self.pub_r_cmd.publish(msg)
        msg.data = float(F_total); self.pub_F_total.publish(msg)
        msg.data = float(M_cmd); self.pub_M_cmd.publish(msg)
        msg.data = float(left_cmd); self.pub_left_cmd.publish(msg)
        msg.data = float(right_cmd); self.pub_right_cmd.publish(msg)
        msg.data = float(self.current_u); self.pub_current_u.publish(msg)
        msg.data = float(self.desired_speed); self.pub_desired_speed.publish(msg)
        msg.data = float(offtrack); self.pub_offtrack.publish(msg)
        msg.data = float(self.lookahead_distance); self.pub_lookahead.publish(msg)
        msg.data = float(self.epsilon); self.pub_epsilon.publish(msg)

        # damping debug
        msg.data = float(damping_F); self.pub_damping_F.publish(msg)
        msg.data = float(damping_M); self.pub_damping_M.publish(msg)
        msg.data = float(self.filtered_r); self.pub_r_filtered.publish(msg)



def main(args=None):
    rclpy.init(args=args)
    node = SMCWamV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
