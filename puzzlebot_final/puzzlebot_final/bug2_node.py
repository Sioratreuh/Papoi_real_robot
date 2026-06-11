#!/usr/bin/env python3
# Bug2: reactive navigation with M-line return condition.
# States: WAITING → GO_TO_GOAL ↔ WALL_FOLLOWING → STOP
#
# Wall-following sub-states (tracked inside WALL_FOLLOWING):
#   acquiring  → searching for wall (wall_acquired = False)
#   following  → proportional control against wall distance
#   recovery   → wall lost: advance + turn sequence to reacquire
#   corner     → front blocked: tracked 90-deg turn, then resume
import math
import signal
import sys
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')

        # Latched QoS: late-connecting node receives last published goal
        latched = qos.QoSProfile(depth=1)
        latched.durability = qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self.mline_pub = self.create_publisher(Marker, 'mline_marker', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, latched)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos.qos_profile_sensor_data)

        signal.signal(signal.SIGINT, self.shutdown_function)

        # --- State machine ---
        self.state         = 'WAITING'
        self.goal_received = False

        # --- Odometry pose ---
        self.x = self.y = self.theta = 0.0

        # --- Goal & M-line ---
        self.target_x = self.target_y = 0.0
        self.start_x  = self.start_y  = 0.0   # M-line origin (pose at goal reception)
        self.hit_x    = self.hit_y    = 0.0   # pose at first wall contact
        self.hit_distance     = float('inf')  # dist-to-goal when wall was first hit
        self.left_hit_region  = False          # True once robot moved min_hit_separation from hit point
        self.best_dist_to_goal = float('inf')

        # --- Sensor timing ---
        self.last_odom_time       = None
        self.last_scan_time       = None
        self.last_diagnostic_time = self.get_clock().now()

        # --- Scan derived data ---
        self.angle_ranges        = []   # (angle_rad, distance_m) after scan_front_angle rotation
        self.closest_range       = None
        self.closest_angle       = 0.0
        self.closest_front_range = None
        self.closest_front_angle = 0.0
        self.regions = {
            'front': 10.0, 'fright': 10.0, 'right': 10.0, 'bright': 10.0,
            'back': 10.0,  'bleft':  10.0, 'left':  10.0, 'fleft':  10.0,
        }

        # ── Parameters ────────────────────────────────────────────────────────
        # Goal capture
        self.declare_parameter('goal_tolerance',               0.05)
        self.declare_parameter('wall_follow_goal_tolerance',   0.08)   # looser capture during WALL_FOLLOWING
        self.declare_parameter('goal_pass_margin',             0.02)   # stop if we move this far past closest approach
        self.declare_parameter('goal_pass_lateral_tolerance',  0.22)   # lateral corridor for goal-plane crossing
        self.declare_parameter('goal_priority_distance',       0.35)   # within this dist, only enter wall-follow if truly blocked
        self.declare_parameter('near_goal_slow_distance',      0.35)
        self.declare_parameter('near_goal_v_max',              0.025)

        # M-line
        self.declare_parameter('m_line_tolerance',             0.20)   # max perp. distance to M-line to re-enter GO_TO_GOAL
        self.declare_parameter('min_hit_separation',           0.15)   # must move this far from hit before left_hit_region=True
        self.declare_parameter('hit_return_tolerance',         0.18)   # within this of hit point → declare full loop
        self.declare_parameter('m_line_goal_improvement',      0.05)   # must be this much closer than hit_distance to leave wall

        # Navigation gains
        self.declare_parameter('k_rho',                        0.6)    # distance → forward speed
        self.declare_parameter('k_alpha',                      1.5)    # heading error → angular speed
        self.declare_parameter('v_max',                        0.08)
        self.declare_parameter('w_max',                        0.40)
        self.declare_parameter('heading_tolerance',            0.15)   # rad; below this → drive forward
        self.declare_parameter('min_forward_speed',            0.02)
        self.declare_parameter('front_stop_distance',          0.18)   # hard stop threshold
        self.declare_parameter('front_slow_distance',          0.24)   # start slowing down
        self.declare_parameter('avoidance_start_distance',     0.30)   # trigger avoidance in GO_TO_GOAL

        # Wall following geometry
        self.declare_parameter('wall_follow_start_distance',   0.24)   # obstacle distance that triggers WALL_FOLLOWING
        self.declare_parameter('wall_distance',                0.16)   # desired lateral clearance
        self.declare_parameter('wall_follow_side',             'right')  # 'right' or 'left'
        self.declare_parameter('start_with_wall_acquisition',  False)   # if True, acquire wall on goal reception before navigating

        # Wall following control
        self.declare_parameter('wall_acquire_distance',        0.16)   # must be within this distance to count as acquired
        self.declare_parameter('wall_too_close',               0.11)   # emergency slow when closer than this
        self.declare_parameter('wall_lost_distance',           0.27)   # side distance above which wall is considered lost
        self.declare_parameter('wall_follow_speed',            0.10)
        self.declare_parameter('wall_follow_kp',               1.2)    # proportional gain: side distance error → angular
        self.declare_parameter('wall_front_kp',                0.7)    # proportional gain: front-side error → angular
        self.declare_parameter('wall_follow_deadband',         0.025)  # ignore side errors smaller than this
        self.declare_parameter('wall_search_angular_speed',    0.18)   # slow search rotation when acquiring
        self.declare_parameter('wall_command_alpha',           0.35)   # EMA smoothing on wall velocity commands (0=frozen, 1=instant)

        # Wall recovery (wall lost: advance/turn sequence)
        self.declare_parameter('wall_recovery_forward_distance',        0.10)
        self.declare_parameter('wall_recovery_first_forward_distance',  0.07)
        self.declare_parameter('wall_recovery_next_forward_distance',   0.15)
        self.declare_parameter('wall_recovery_forward_speed',           0.035)
        self.declare_parameter('wall_recovery_turn_angle',              math.pi / 2.0)
        self.declare_parameter('wall_recovery_turn_speed',              0.30)

        # Wall corner (front blocked: tracked 90-deg rotation)
        self.declare_parameter('wall_corner_angular_speed',    0.30)
        self.declare_parameter('wall_corner_forward_speed',    0.025)

        # Avoidance
        self.declare_parameter('avoidance_kv',     0.5)
        self.declare_parameter('avoidance_kw',     0.7)

        # Sensor watchdog
        self.declare_parameter('sensor_timeout',   1.0)
        self.declare_parameter('require_scan',     True)
        self.declare_parameter('require_odom',     True)
        self.declare_parameter('scan_front_angle', 0.0)   # deg; 0=LiDAR already aligned with robot front

        # ── Fetch parameter values ─────────────────────────────────────────
        self.goal_tolerance              = self.get_parameter('goal_tolerance').value
        self.wall_follow_goal_tolerance  = self.get_parameter('wall_follow_goal_tolerance').value
        self.goal_pass_margin            = self.get_parameter('goal_pass_margin').value
        self.goal_pass_lateral_tolerance = self.get_parameter('goal_pass_lateral_tolerance').value
        self.goal_priority_distance      = self.get_parameter('goal_priority_distance').value
        self.near_goal_slow_distance     = self.get_parameter('near_goal_slow_distance').value
        self.near_goal_v_max             = self.get_parameter('near_goal_v_max').value
        self.m_line_tolerance            = self.get_parameter('m_line_tolerance').value
        self.min_hit_separation          = self.get_parameter('min_hit_separation').value
        self.hit_return_tolerance        = self.get_parameter('hit_return_tolerance').value
        self.m_line_goal_improvement     = self.get_parameter('m_line_goal_improvement').value
        self.k_rho                       = self.get_parameter('k_rho').value
        self.k_alpha                     = self.get_parameter('k_alpha').value
        self.v_max                       = self.get_parameter('v_max').value
        self.w_max                       = self.get_parameter('w_max').value
        self.heading_tolerance           = self.get_parameter('heading_tolerance').value
        self.min_forward_speed           = self.get_parameter('min_forward_speed').value
        self.front_stop_distance         = self.get_parameter('front_stop_distance').value
        self.front_slow_distance         = self.get_parameter('front_slow_distance').value
        self.avoidance_start_distance    = self.get_parameter('avoidance_start_distance').value
        self.wall_follow_start_distance  = self.get_parameter('wall_follow_start_distance').value
        self.wall_distance               = self.get_parameter('wall_distance').value
        self.wall_follow_side            = self.get_parameter('wall_follow_side').value.lower()
        if self.wall_follow_side not in ('right', 'left'):
            self.get_logger().warn(
                f'Invalid wall_follow_side="{self.wall_follow_side}". Defaulting to "right".')
            self.wall_follow_side = 'right'
        self.start_with_wall_acquisition = self.get_parameter('start_with_wall_acquisition').value
        self.wall_acquire_distance       = self.get_parameter('wall_acquire_distance').value
        self.wall_too_close              = self.get_parameter('wall_too_close').value
        self.wall_lost_distance          = self.get_parameter('wall_lost_distance').value
        self.wall_follow_speed           = self.get_parameter('wall_follow_speed').value
        self.wall_follow_kp              = self.get_parameter('wall_follow_kp').value
        self.wall_front_kp               = self.get_parameter('wall_front_kp').value
        self.wall_follow_deadband        = self.get_parameter('wall_follow_deadband').value
        self.wall_search_angular_speed   = self.get_parameter('wall_search_angular_speed').value
        self.wall_command_alpha          = self.get_parameter('wall_command_alpha').value
        self.wall_recovery_forward_distance              = self.get_parameter('wall_recovery_forward_distance').value
        self.wall_recovery_first_forward_distance        = self.get_parameter('wall_recovery_first_forward_distance').value
        self.wall_recovery_next_forward_distance         = self.get_parameter('wall_recovery_next_forward_distance').value
        self.wall_recovery_forward_speed                 = self.get_parameter('wall_recovery_forward_speed').value
        self.wall_recovery_turn_angle                    = self.get_parameter('wall_recovery_turn_angle').value
        self.wall_recovery_turn_speed                    = self.get_parameter('wall_recovery_turn_speed').value
        self.wall_corner_angular_speed                   = self.get_parameter('wall_corner_angular_speed').value
        self.wall_corner_forward_speed                   = self.get_parameter('wall_corner_forward_speed').value
        self.avoidance_kv      = self.get_parameter('avoidance_kv').value
        self.avoidance_kw      = self.get_parameter('avoidance_kw').value
        self.sensor_timeout    = self.get_parameter('sensor_timeout').value
        self.require_scan      = self.get_parameter('require_scan').value
        self.require_odom      = self.get_parameter('require_odom').value
        self.scan_front_angle  = self.get_parameter('scan_front_angle').value

        # ── Wall following runtime state ──────────────────────────────────
        self.last_wall_linear   = 0.0  # EMA state for smooth_wall_command
        self.last_wall_angular  = 0.0
        self.wall_acquired      = False  # True once robot is within wall_acquire_distance of side wall
        self.initial_wall_acquisition = False  # True during startup wall-find phase

        # Recovery state (wall lost → advance+turn sequence)
        self.wall_recovery_phase    = 'none'   # 'none' | 'advance' | 'turn'
        self.wall_recovery_advance_count = 0
        self.wall_recovery_current_forward_distance = self.wall_recovery_first_forward_distance
        self.wall_recovery_start_x     = 0.0
        self.wall_recovery_start_y     = 0.0
        self.wall_recovery_start_theta = 0.0

        # Corner state (front blocked → tracked 90-deg rotation)
        self.wall_corner_phase               = 'none'   # 'none' | 'turn'
        self.wall_corner_start_theta         = 0.0
        self.wall_corner_suppress_until_clear = False   # debounce after completing a corner

        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info(
            f'[BUG2] ready | wall_side={self.wall_follow_side} '
            f'start_with_acq={self.start_with_wall_acquisition} '
            f'v_max={self.v_max} w_max={self.w_max}')

    # ─── Utilities ────────────────────────────────────────────────────────────
    def publish_mline_marker(self):
        """Publish M-line as a LINE_STRIP marker for RViz visualization."""
        if not self.goal_received:
            return
        marker = Marker()
        marker.header.frame_id = 'odom'
        marker.header.stamp    = self.get_clock().now().to_msg()
        marker.ns     = 'mline'
        marker.id     = 0
        marker.type   = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02          # line width in meters
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8
        marker.lifetime.sec = 0        # 0 = persist forever

        p1 = Point()
        p1.x, p1.y, p1.z = self.start_x, self.start_y, 0.0

        p2 = Point()
        p2.x, p2.y, p2.z = self.target_x, self.target_y, 0.0

        marker.points = [p1, p2]
        self.mline_pub.publish(marker)

    def normalize_angle(self, angle):
        while angle >  math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'[STATE] {self.state} → {new_state}')
            self.state = new_state

    def sector_min(self, center_angle, half_width, default=10.0):
        """Min distance in the angular sector [center ± half_width]."""
        values = [d for a, d in self.angle_ranges
                  if abs(self.normalize_angle(a - center_angle)) <= half_width]
        return min(values) if values else default

    def get_closest_object_info(self):
        """(min_distance, angle) across all scan points."""
        if not self.angle_ranges:
            return None, 0.0
        return min(((d, a) for a, d in self.angle_ranges), key=lambda x: x[0])

    def get_closest_front_object_info(self):
        """(min_distance, angle) within ±40° of front."""
        front = [(d, a) for a, d in self.angle_ranges if abs(a) <= math.radians(40)]
        if not front:
            return None, 0.0
        return min(front, key=lambda x: x[0])

    # ─── M-line geometry ──────────────────────────────────────────────────────

    def distance_to_m_line(self):
        """Perpendicular distance from robot to M-line (start → goal segment)."""
        num = abs((self.target_y - self.start_y) * self.x
                  - (self.target_x - self.start_x) * self.y
                  + self.target_x * self.start_y
                  - self.target_y * self.start_x)
        den = math.hypot(self.target_y - self.start_y, self.target_x - self.start_x)
        return num / den if den > 0.0 else 0.0

    def goal_line_progress(self):
        """Scalar projection of robot onto M-line; returns (progress, length, lateral_err)."""
        gdx, gdy = self.target_x - self.start_x, self.target_y - self.start_y
        length = math.hypot(gdx, gdy)
        if length == 0.0:
            return 0.0, 0.0, 0.0
        rdx, rdy = self.x - self.start_x, self.y - self.start_y
        return (rdx * gdx + rdy * gdy) / length, length, self.distance_to_m_line()

    # ─── Goal detection ───────────────────────────────────────────────────────

    def should_stop_for_goal(self, dist):
        """Returns (True, reason) for any goal-capture condition."""
        if dist <= self.goal_tolerance:
            return True, f'within tolerance dist={dist:.2f}m'
        if self.state == 'WALL_FOLLOWING' and dist <= self.wall_follow_goal_tolerance:
            return True, f'wall_follow capture dist={dist:.2f}m'
        if (self.best_dist_to_goal <= self.wall_follow_goal_tolerance
                and dist > self.best_dist_to_goal + self.goal_pass_margin):
            return True, f'passed closest approach best={self.best_dist_to_goal:.2f}m'
        progress, length, lat_err = self.goal_line_progress()
        if (progress >= (length - self.goal_tolerance)
                and lat_err <= self.goal_pass_lateral_tolerance):
            return True, f'crossed goal plane progress={progress:.2f}/{length:.2f}m lat={lat_err:.2f}m'
        return False, ''

    def stop_at_goal(self, reason):
        self.change_state('STOP')
        self.get_logger().info(f'[GOAL REACHED] {reason}')
        self.cmd_pub.publish(Twist())
        self.goal_received = False

    def check_goal_priority(self, odom_age):
        """Check + update best_dist; stop immediately if goal is reached. Returns True if stopped."""
        if odom_age is None or odom_age > self.sensor_timeout:
            return False
        dist = math.hypot(self.target_x - self.x, self.target_y - self.y)
        self.best_dist_to_goal = min(self.best_dist_to_goal, dist)
        ok, reason = self.should_stop_for_goal(dist)
        if ok:
            self.stop_at_goal(reason)
            return True
        return False

    def is_path_to_goal_clear(self, err_theta):
        """True if front sector and the goal direction sector are both obstacle-free."""
        return (self.sector_min(err_theta, math.radians(15)) > self.front_slow_distance
                and self.regions['front'] > self.front_slow_distance)

    def goal_path_distance(self, err_theta):
        """Min distance in the direction of the goal (used for wall-following entry threshold)."""
        return min(self.regions['front'],
                   self.sector_min(err_theta, math.radians(15)))

    def should_enter_wall_following(self, dist_to_goal, err_theta):
        """Near goal: only enter if truly blocked. Far: enter at wall_follow_start_distance."""
        obstacle = self.goal_path_distance(err_theta)
        if dist_to_goal <= self.goal_priority_distance:
            return obstacle < self.front_stop_distance
        return obstacle < self.wall_follow_start_distance

    # ─── Wall following: state transitions ────────────────────────────────────

    def enter_wall_following(self, dist_to_goal, initial_acquisition=False):
        """Record hit point, reset wall sub-states, switch to WALL_FOLLOWING."""
        self.hit_distance    = dist_to_goal
        self.hit_x = self.x
        self.hit_y = self.y
        self.left_hit_region        = False
        self.last_wall_linear       = 0.0
        self.last_wall_angular      = 0.0
        self.wall_acquired          = False
        self.initial_wall_acquisition = initial_acquisition
        self.reset_wall_recovery()
        if initial_acquisition:
            self.get_logger().info(
                f'[ACQUIRE] initial wall acquisition {self.wall_follow_side} '
                f'from ({self.hit_x:.2f},{self.hit_y:.2f}) goal={self.hit_distance:.2f}m')
        else:
            self.get_logger().info(
                f'[HIT] wall contact at ({self.hit_x:.2f},{self.hit_y:.2f}) '
                f'dist_to_goal={self.hit_distance:.2f}m side={self.wall_follow_side}')
        self.change_state('WALL_FOLLOWING')

    def finish_initial_wall_acquisition(self):
        """Once wall is acquired for the first time, reset hit point to current pose."""
        if not self.initial_wall_acquisition:
            return
        self.initial_wall_acquisition = False
        self.hit_x = self.x
        self.hit_y = self.y
        self.hit_distance    = math.hypot(self.target_x - self.x, self.target_y - self.y)
        self.left_hit_region = False
        self.get_logger().info(
            f'[ACQUIRE] wall {self.wall_follow_side} acquired. '
            f'Hit point reset to ({self.hit_x:.2f},{self.hit_y:.2f}) '
            f'dist={self.hit_distance:.2f}m')

    # ─── Wall following: recovery ──────────────────────────────────────────────

    def reset_wall_recovery(self):
        self.wall_recovery_phase    = 'none'
        self.wall_recovery_advance_count = 0
        self.wall_recovery_current_forward_distance = self.wall_recovery_first_forward_distance
        self.wall_recovery_start_x     = self.x
        self.wall_recovery_start_y     = self.y
        self.wall_recovery_start_theta = self.theta

    def start_wall_recovery_advance(self):
        self.wall_recovery_phase = 'advance'
        if self.wall_recovery_advance_count == 0:
            self.wall_recovery_current_forward_distance = self.wall_recovery_first_forward_distance
        else:
            self.wall_recovery_current_forward_distance = self.wall_recovery_next_forward_distance
        self.wall_recovery_advance_count += 1
        self.wall_recovery_start_x = self.x
        self.wall_recovery_start_y = self.y
        self.wall_recovery_start_theta = self.theta
        self.get_logger().info(
            f'[RECOVERY] advance {self.wall_recovery_current_forward_distance:.2f}m '
            f'(attempt {self.wall_recovery_advance_count})')

    def start_wall_recovery_turn(self):
        self.wall_recovery_phase = 'turn'
        self.wall_recovery_start_theta = self.theta
        self.get_logger().info(
            f'[RECOVERY] turn {math.degrees(self.wall_recovery_turn_angle):.0f}deg '
            f'side={self.wall_follow_side}')

    def wall_recovery_turn_direction(self):
        # Recovery turns toward the wall side to reacquire it
        return 1.0 if self.wall_follow_side == 'left' else -1.0

    def finish_wall_recovery_if_wall_found(self, side_region, front_side_region):
        """Returns True (and resets recovery) if wall is within acquisition range."""
        threshold = max(self.wall_lost_distance, self.wall_acquire_distance) + self.wall_follow_deadband
        if min(side_region, front_side_region) > threshold:
            return False
        self.wall_acquired     = True
        self.last_wall_linear  = 0.0
        self.last_wall_angular = 0.0
        self.reset_wall_recovery()
        self.finish_initial_wall_acquisition()
        self.get_logger().info(
            f'[RECOVERY] wall reacquired '
            f'side={side_region:.2f} front_side={front_side_region:.2f}')
        return True

    def handle_wall_recovery(self, msg, side_region, front_side_region, front_distance):
        """Drive advance+turn sequence until wall is reacquired. Returns True while active."""
        if self.finish_wall_recovery_if_wall_found(side_region, front_side_region):
            self.smooth_wall_command(msg, 0.0, 0.0)
            return False

        if self.wall_recovery_phase == 'none':
            self.start_wall_recovery_advance()

        if self.wall_recovery_phase == 'turn':
            turned = abs(self.normalize_angle(self.theta - self.wall_recovery_start_theta))
            if turned < self.wall_recovery_turn_angle:
                self.smooth_wall_command(
                    msg, 0.0,
                    self.wall_recovery_turn_direction() * self.wall_recovery_turn_speed)
                return True
            self.start_wall_recovery_advance()
            self.smooth_wall_command(msg, 0.0, 0.0)
            return True

        if self.wall_recovery_phase == 'advance':
            if front_distance < self.front_stop_distance:
                self.start_wall_recovery_turn()
                self.smooth_wall_command(msg, 0.0, 0.0)
                return True
            advanced = math.hypot(self.x - self.wall_recovery_start_x,
                                  self.y - self.wall_recovery_start_y)
            if advanced < self.wall_recovery_current_forward_distance:
                self.smooth_wall_command(msg, self.wall_recovery_forward_speed, 0.0)
                return True
            if self.finish_wall_recovery_if_wall_found(side_region, front_side_region):
                self.smooth_wall_command(msg, 0.0, 0.0)
                return True
            self.start_wall_recovery_turn()
            self.smooth_wall_command(msg, 0.0, 0.0)
            return True

        return False

    # ─── Wall following: corner handling ──────────────────────────────────────

    def reset_wall_corner_turn(self, suppress_until_clear=False):
        self.wall_corner_phase               = 'none'
        self.wall_corner_start_theta         = self.theta
        self.wall_corner_suppress_until_clear = suppress_until_clear

    def start_wall_corner_turn(self):
        self.wall_corner_phase       = 'turn'
        self.wall_corner_start_theta = self.theta
        self.wall_corner_suppress_until_clear = False
        self.get_logger().info(f'[CORNER] 90-deg turn side={self.wall_follow_side}')

    def wall_corner_turn_direction(self):
        # Corner turns away from the followed wall (away = opposite side)
        return -1.0 if self.wall_follow_side == 'left' else 1.0

    def handle_wall_corner_turn(self, msg, front_distance):
        """Execute a tracked 90-deg corner turn. Returns True while active."""
        if self.wall_corner_phase == 'none':
            self.start_wall_corner_turn()

        if front_distance >= self.front_stop_distance:
            # Front cleared before completing turn: abort
            self.reset_wall_corner_turn()
            return False

        turned = abs(self.normalize_angle(self.theta - self.wall_corner_start_theta))
        if turned < math.pi / 2.0:
            self.smooth_wall_command(
                msg, 0.0,
                self.wall_corner_turn_direction() * self.wall_corner_angular_speed)
            return True

        # Turn complete: suppress corner re-trigger until front clears
        self.reset_wall_corner_turn(suppress_until_clear=True)
        self.last_wall_linear  = 0.0
        self.last_wall_angular = 0.0
        return False

    # ─── Wall following: velocity commands ────────────────────────────────────

    def wall_follow_geometry(self):
        """Returns (front_side_region, side_region, side_sign).
        side_sign: +1 = left wall, -1 = right wall. Used to make logic side-agnostic.
        """
        if self.wall_follow_side == 'left':
            return self.regions['fleft'], self.regions['left'], 1.0
        return self.regions['fright'], self.regions['right'], -1.0

    def smooth_wall_command(self, msg, target_linear, target_angular):
        """EMA filter on wall velocity commands — prevents proportional-controller oscillations."""
        a = self.clamp(self.wall_command_alpha, 0.0, 1.0)
        self.last_wall_linear  = a * target_linear  + (1.0 - a) * self.last_wall_linear
        self.last_wall_angular = a * target_angular + (1.0 - a) * self.last_wall_angular
        msg.linear.x  = self.last_wall_linear
        msg.angular.z = self.clamp(self.last_wall_angular, -self.w_max, self.w_max)

    def set_avoidance_command(self, msg, closest_range, theta_closest):
        """Emergency avoidance: steer directly away from closest detected point."""
        theta_away = self.normalize_angle(
            theta_closest + (math.pi if theta_closest <= 0.0 else -math.pi))
        msg.linear.x = (0.0 if closest_range < self.front_stop_distance
                        else self.clamp(
                            self.avoidance_kv * (closest_range - self.front_stop_distance),
                            0.0, min(self.v_max, 0.04)))
        msg.angular.z = self.clamp(self.avoidance_kw * theta_away, -self.w_max, self.w_max)

    def set_wall_follow_command(self, msg, closest_front_range, closest_front_angle):
        """Full wall-following controller with acquisition, recovery, and corner handling."""
        front_side, side_region, side_sign = self.wall_follow_geometry()
        side_wall = min(side_region, front_side)
        away_turn   = -side_sign   # direction away from followed wall
        toward_turn =  side_sign   # direction toward followed wall

        # Combined front distance: sector + closest-front scan
        front_distance = min(
            self.regions['front'],
            closest_front_range if closest_front_range is not None else 10.0)

        # Wall-lost threshold with hysteresis deadband
        wall_lost_threshold = max(self.wall_lost_distance, self.wall_acquire_distance) + self.wall_follow_deadband

        # Priority 1: active recovery overrides everything
        if self.wall_recovery_phase != 'none':
            if self.handle_wall_recovery(msg, side_region, front_side, front_distance):
                return

        # Priority 2: active corner turn
        if self.wall_corner_phase != 'none':
            if self.handle_wall_corner_turn(msg, front_distance):
                return

        # Clear corner suppression once front is free
        if self.wall_corner_suppress_until_clear and front_distance >= self.front_stop_distance:
            self.wall_corner_suppress_until_clear = False
        if self.wall_corner_suppress_until_clear:
            self.smooth_wall_command(msg, 0.0, 0.0)
            return

        # Priority 3: front wall → start corner turn
        if (front_distance < self.front_stop_distance
                and self.wall_acquired
                and side_wall <= self.wall_lost_distance
                and not self.wall_corner_suppress_until_clear):
            if self.handle_wall_corner_turn(msg, front_distance):
                return

        if front_distance < self.front_stop_distance and not self.wall_corner_suppress_until_clear:
            # Creep forward slightly if wall is close and we're not dangerously near front
            creep = (min(self.wall_corner_forward_speed, self.wall_follow_speed * 0.25)
                     if (self.wall_acquired
                         and side_wall <= self.wall_lost_distance
                         and front_distance > self.wall_too_close)
                     else 0.0)
            # Ease corner turn if already far enough from wall
            corner_w = away_turn * self.wall_corner_angular_speed
            if side_region > self.wall_distance:
                corner_w *= 0.65
            self.smooth_wall_command(msg, creep, corner_w)
            return

        # Priority 4: not yet acquired → search
        if not self.wall_acquired:
            if side_wall <= self.wall_acquire_distance:
                self.wall_acquired = True
                self.reset_wall_recovery()
                self.finish_initial_wall_acquisition()
            else:
                # Slow turn toward wall side
                self.smooth_wall_command(msg, 0.02, toward_turn * self.wall_search_angular_speed)
                return

        # Priority 5: wall lost → recovery
        if side_wall > wall_lost_threshold:
            self.wall_acquired = False
            if self.handle_wall_recovery(msg, side_region, front_side, front_distance):
                return
            self.smooth_wall_command(msg, 0.0, 0.0)
            return

        # Normal following: proportional control on side distance
        self.reset_wall_recovery()
        if not self.wall_corner_suppress_until_clear:
            self.reset_wall_corner_turn()

        # Emergency: diagonal wall too close on followed side
        if front_side < self.wall_too_close:
            self.smooth_wall_command(msg, 0.02, away_turn * self.wall_corner_angular_speed)
            return

        # Proportional: side_error drives angular, front_side_error adds correction
        wall_error  = side_region - self.wall_distance
        if abs(wall_error) < self.wall_follow_deadband:
            wall_error = 0.0  # ignore tiny errors to prevent jitter
        front_error = max(0.0, self.wall_distance - front_side)

        target_angular = (side_sign * self.wall_follow_kp * wall_error
                          - side_sign * self.wall_front_kp * front_error)
        target_linear  = self.wall_follow_speed
        if front_side < self.wall_distance:
            target_linear *= 0.7   # slow if diagonal wall is tight
        if side_region < self.wall_too_close:
            target_linear *= 0.7   # slow if side wall is too close

        self.smooth_wall_command(msg, target_linear, target_angular)

    # ─── Main control loop (20 Hz) ────────────────────────────────────────────

    def control_loop(self):
        if not self.goal_received:
            return

        msg = Twist()
        now      = self.get_clock().now()
        odom_age = self.message_age(now, self.last_odom_time)
        scan_age = self.message_age(now, self.last_scan_time)

        # Goal check runs even if scan is stale (odom-only)
        if self.check_goal_priority(odom_age):
            return

        if not self.sensors_ready(odom_age, scan_age):
            self.cmd_pub.publish(msg)
            self.publish_diagnostics(msg, None, None, odom_age, scan_age)
            return

        dist_to_goal  = math.hypot(self.target_x - self.x, self.target_y - self.y)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta     = self.normalize_angle(angle_to_goal - self.theta)

        closest_range,       closest_angle       = self.get_closest_object_info()
        closest_front_range, closest_front_angle = self.get_closest_front_object_info()
        self.closest_range       = closest_range
        self.closest_angle       = closest_angle
        self.closest_front_range = closest_front_range
        self.closest_front_angle = closest_front_angle

        ok, reason = self.should_stop_for_goal(dist_to_goal)
        if ok:
            self.stop_at_goal(reason)
            return

        # ── State transitions (no velocity changes here) ───────────────────
        if self.state == 'GO_TO_GOAL':
            if self.should_enter_wall_following(dist_to_goal, err_theta):
                self.enter_wall_following(dist_to_goal)

        elif self.state == 'WALL_FOLLOWING':
            dist_m_line = self.distance_to_m_line()
            dist_to_hit = math.hypot(self.x - self.hit_x, self.y - self.hit_y)
            closest_clear = (closest_front_range is None
                             or closest_front_range > self.avoidance_start_distance)

            # Only start tracking left_hit_region after wall is acquired and initial acq is done
            if (self.wall_acquired
                    and not self.initial_wall_acquisition
                    and dist_to_hit > self.min_hit_separation):
                self.left_hit_region = True

            returned_to_hit = (
                not self.initial_wall_acquisition
                and self.wall_acquired
                and self.left_hit_region
                and dist_to_hit < self.hit_return_tolerance
                and dist_to_goal >= (self.hit_distance - self.m_line_goal_improvement))

            # M-line re-entry: closer to goal than at hit AND path clear
            if (closest_clear
                    and dist_m_line  < self.m_line_tolerance
                    and dist_to_goal < (self.hit_distance - self.m_line_goal_improvement)
                    and self.wall_acquired
                    and not self.initial_wall_acquisition
                    and self.left_hit_region
                    and self.is_path_to_goal_clear(err_theta)):
                self.get_logger().info(
                    f'[M-LINE] re-entry dist={dist_to_goal:.2f}m '
                    f'(hit={self.hit_distance:.2f}m) m_err={dist_m_line:.3f}m')
                self.change_state('GO_TO_GOAL')

            elif returned_to_hit:
                # Full loop without improvement → goal is blocked
                self.get_logger().warn(
                    f'[BLOCKED] returned to hit ({self.hit_x:.2f},{self.hit_y:.2f}) '
                    f'without M-line improvement. Goal may be unreachable.')
                self.change_state('STOP')
                self.cmd_pub.publish(Twist())
                self.goal_received = False
                return

        # ── Velocity commands ──────────────────────────────────────────────
        if self.state == 'GO_TO_GOAL':
            if self.should_enter_wall_following(dist_to_goal, err_theta):
                # Entered wall following above; apply wall command immediately
                self.set_wall_follow_command(msg, closest_front_range, closest_front_angle)
            elif abs(err_theta) > self.heading_tolerance:
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
                heading_factor = max(0.0, math.cos(err_theta))
                if heading_factor < 0.2:
                    msg.linear.x = 0.0          # nearly perpendicular: rotate in place
                else:
                    msg.linear.x = self.clamp(
                        self.k_rho * dist_to_goal * heading_factor,
                        self.min_forward_speed, self.v_max)
            else:
                msg.linear.x  = self.clamp(self.k_rho * dist_to_goal, 0.0, self.v_max)
                msg.angular.z = 0.0

            # Near-goal speed cap
            if dist_to_goal < self.near_goal_slow_distance:
                factor = self.clamp(dist_to_goal / self.near_goal_slow_distance, 0.25, 1.0)
                msg.linear.x = min(msg.linear.x, self.near_goal_v_max * factor)

            # Front slow-down band
            if (msg.linear.x > 0.0
                    and closest_front_range is not None
                    and closest_front_range < self.front_slow_distance):
                clearance = closest_front_range - self.front_stop_distance
                band = self.front_slow_distance - self.front_stop_distance
                msg.linear.x *= self.clamp(clearance / band, 0.0, 1.0)

        elif self.state == 'WALL_FOLLOWING':
            self.set_wall_follow_command(msg, closest_front_range, closest_front_angle)

        self.cmd_pub.publish(msg)
        self.publish_diagnostics(msg, dist_to_goal, err_theta, odom_age, scan_age)

    # ─── Sensor validation ────────────────────────────────────────────────────

    def sensors_ready(self, odom_age, scan_age):
        """True only when all required sensors have fresh data."""
        odom_stale = self.require_odom and (odom_age is None or odom_age > self.sensor_timeout)
        scan_stale = self.require_scan and (scan_age is None or scan_age > self.sensor_timeout)
        return not odom_stale and not scan_stale

    # ─── Diagnostics (throttled to 2 s) ───────────────────────────────────────

    def publish_diagnostics(self, cmd, dist, err_theta, odom_age=None, scan_age=None):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return
        self.last_diagnostic_time = now

        odom_age = odom_age or self.message_age(now, self.last_odom_time)
        scan_age = scan_age or self.message_age(now, self.last_scan_time)

        if odom_age is None or odom_age > self.sensor_timeout:
            self.get_logger().warn(f'[STALE] odom={self.format_age(odom_age)} — zero velocity')
        if scan_age is None or scan_age > self.sensor_timeout:
            self.get_logger().warn(f'[STALE] scan={self.format_age(scan_age)} — zero velocity')

        dist_text = 'no_odom' if dist      is None else f'{dist:.2f}'
        err_text  = 'no_odom' if err_theta is None else f'{err_theta:.2f}'
        path_text = 'no_odom' if err_theta is None else f'{self.goal_path_distance(err_theta):.2f}'
        _, side_dist, _ = self.wall_follow_geometry()

        self.get_logger().info(
            f'[BUG2] state={self.state} | '
            f'cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} | '
            f'dist={dist_text} err={err_text} path_front={path_text} | '
            f'odom={self.format_age(odom_age)} scan={self.format_age(scan_age)} | '
            f'closest={self.format_closest()} | '
            f'wall_side={self.wall_follow_side} acquired={self.wall_acquired} '
            f'wall_dist={side_dist:.2f} front={self.regions["front"]:.2f} | '
            f'recovery={self.wall_recovery_phase} corner={self.wall_corner_phase} | '
            f'R={self.regions["right"]:.2f} L={self.regions["left"]:.2f} '
            f'FR={self.regions["fright"]:.2f} FL={self.regions["fleft"]:.2f}'
        )

    # ─── Utilities ────────────────────────────────────────────────────────────

    def message_age(self, now, last_time):
        if last_time is None:
            return None
        return (now - last_time).nanoseconds * 1.0e-9

    def format_age(self, age):
        return 'never' if age is None else f'{age:.1f}s'

    def format_closest(self):
        if self.closest_range is None:
            return 'none'
        return f'{self.closest_range:.2f}@{math.degrees(self.closest_angle):.0f}deg'

    # ─── Callbacks ────────────────────────────────────────────────────────────

    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.start_x, self.start_y = self.x, self.y   # M-line origin = current pose
        self.hit_distance      = float('inf')
        self.left_hit_region   = False
        self.best_dist_to_goal = float('inf')
        self.goal_received     = True
        self.get_logger().info(
            f'[GOAL] target=({self.target_x},{self.target_y}) '
            f'from=({self.start_x:.2f},{self.start_y:.2f}) M-line set')
        self.publish_mline_marker()
        dist = math.hypot(self.target_x - self.x, self.target_y - self.y)
        if self.start_with_wall_acquisition:
            # Go find the wall first; start navigating after acquisition
            self.enter_wall_following(dist, initial_acquisition=True)
        else:
            self.change_state('GO_TO_GOAL')

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.angle_ranges = []
        for i, d in enumerate(msg.ranges):
            # Rotate scan so 0 rad = robot forward (corrects LiDAR mounting offset)
            a = self.normalize_angle(
                msg.angle_min + i * msg.angle_increment - math.radians(self.scan_front_angle))
            if math.isinf(d) or math.isnan(d) or d > msg.range_max:
                d = 10.0
            elif d < max(msg.range_min, 0.12):
                d = 0.01   # clamp noise/reflections instead of dropping
            self.angle_ranges.append((a, d))

        self.regions = {
            'front':  self.sector_min(0.0,                math.radians(22.5)),
            'fright': self.sector_min(math.radians(-45),  math.radians(22.5)),
            'right':  self.sector_min(math.radians(-90),  math.radians(22.5)),
            'bright': self.sector_min(math.radians(-135), math.radians(22.5)),
            'back':   self.sector_min(math.pi,            math.radians(22.5)),
            'bleft':  self.sector_min(math.radians(135),  math.radians(22.5)),
            'left':   self.sector_min(math.radians(90),   math.radians(22.5)),
            'fleft':  self.sector_min(math.radians(45),   math.radians(22.5)),
        }
        self.closest_range,       self.closest_angle       = self.get_closest_object_info()
        self.closest_front_range, self.closest_front_angle = self.get_closest_front_object_info()

    def shutdown_function(self, signum, frame):
        self.get_logger().info('[SHUTDOWN] publishing zero velocity before exit')
        self.cmd_pub.publish(Twist())
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.cmd_pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()