#!/usr/bin/env python3
# Bug0: reactive obstacle avoidance — NO M-line memory.
# States: WAITING → GO_TO_GOAL ↔ WALL_FOLLOWING → STOP
# Difference vs Bug2: returns to GO_TO_GOAL as soon as the path to goal is clear,
# regardless of where the robot is relative to the start-goal line.
# Faster for simple obstacles; may loop forever around concave obstacles.
import math
import signal
import sys
import rclpy
from rclpy import qos
from rclpy.node import Node
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class Bug0Node(Node):
    def __init__(self):
        super().__init__('bug0_node')
        latched = qos.QoSProfile(depth=1)
        latched.durability = qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub  = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry,   'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D,     'goal', self.goal_callback, latched)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos.qos_profile_sensor_data)

        signal.signal(signal.SIGINT, self.shutdown_function)

        # --- State machine ---
        self.state         = 'WAITING'
        self.goal_received = False

        # --- Odometry pose ---
        self.x = self.y = self.theta = 0.0

        # --- Goal ---
        self.target_x = self.target_y = 0.0

        # --- Sensor timing ---
        self.last_odom_time       = None
        self.last_scan_time       = None
        self.last_diagnostic_time = self.get_clock().now()

        # --- Scan derived data ---
        self.angle_ranges        = []
        self.closest_range       = None
        self.closest_angle       = 0.0
        self.closest_front_range = None
        self.closest_front_angle = 0.0
        self.regions = {
            'front': 10.0, 'fright': 10.0, 'right': 10.0, 'bright': 10.0,
            'back': 10.0,  'bleft':  10.0, 'left':  10.0, 'fleft':  10.0,
        }

        # --- Parameters ---
        self.declare_parameter('goal_tolerance',           0.05)
        self.declare_parameter('k_rho',                    0.6)   # proportional gain: distance → forward speed
        self.declare_parameter('k_alpha',                  1.5)   # proportional gain: heading error → angular speed
        self.declare_parameter('v_max',                    0.08)
        self.declare_parameter('w_max',                    0.40)
        self.declare_parameter('heading_tolerance',        0.15)  # rad; below this, stop rotating and drive forward
        self.declare_parameter('min_forward_speed',        0.02)
        self.declare_parameter('require_scan',             True)
        self.declare_parameter('require_odom',             True)
        self.declare_parameter('sensor_timeout',           1.0)
        self.declare_parameter('scan_front_angle',         0.0)   # deg; rotate scan so 0 rad = robot forward
        self.declare_parameter('front_stop_distance',      0.22)  # m; hard stop threshold
        self.declare_parameter('front_slow_distance',      0.30)  # m; start slowing down
        self.declare_parameter('avoidance_start_distance', 0.38)  # m; trigger wall following / avoidance
        self.declare_parameter('avoidance_kv',             0.5)
        self.declare_parameter('avoidance_kw',             0.7)
        self.declare_parameter('wall_threshold',           0.24)  # m; distance that counts as "wall detected" in rule-based follower
        self.declare_parameter('wall_follow_side',         'left')  # 'right' or 'left'
        self.declare_parameter('near_goal_slow_distance',  0.35)
        self.declare_parameter('near_goal_v_max',          0.025)

        self.goal_tolerance           = self.get_parameter('goal_tolerance').value
        self.k_rho                    = self.get_parameter('k_rho').value
        self.k_alpha                  = self.get_parameter('k_alpha').value
        self.v_max                    = self.get_parameter('v_max').value
        self.w_max                    = self.get_parameter('w_max').value
        self.heading_tolerance        = self.get_parameter('heading_tolerance').value
        self.min_forward_speed        = self.get_parameter('min_forward_speed').value
        self.require_scan             = self.get_parameter('require_scan').value
        self.require_odom             = self.get_parameter('require_odom').value
        self.sensor_timeout           = self.get_parameter('sensor_timeout').value
        self.scan_front_angle         = self.get_parameter('scan_front_angle').value
        self.front_stop_distance      = self.get_parameter('front_stop_distance').value
        self.front_slow_distance      = self.get_parameter('front_slow_distance').value
        self.avoidance_start_distance = self.get_parameter('avoidance_start_distance').value
        self.avoidance_kv             = self.get_parameter('avoidance_kv').value
        self.avoidance_kw             = self.get_parameter('avoidance_kw').value
        self.wall_threshold           = self.get_parameter('wall_threshold').value
        self.near_goal_slow_distance  = self.get_parameter('near_goal_slow_distance').value
        self.near_goal_v_max          = self.get_parameter('near_goal_v_max').value
        self.wall_follow_side         = self.get_parameter('wall_follow_side').value.lower()
        if self.wall_follow_side not in ('right', 'left'):
            self.get_logger().warn(
                f'Invalid wall_follow_side="{self.wall_follow_side}". Defaulting to "right".')
            self.wall_follow_side = 'right'

        self.create_timer(0.05, self.control_loop)  # 20 Hz control loop
        self.get_logger().info(
            f'Bug0 ready | wall_side={self.wall_follow_side} '
            f'v_max={self.v_max} w_max={self.w_max}')

    # ─── Helpers ──────────────────────────────────────────────────────────────

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
        """Minimum distance in the angular sector [center±half_width]."""
        values = [d for a, d in self.angle_ranges
                  if abs(self.normalize_angle(a - center_angle)) <= half_width]
        return min(values) if values else default

    def get_closest_object_info(self):
        """Returns (min_distance, angle) across all scan points."""
        if not self.angle_ranges:
            return None, 0.0
        return min(((d, a) for a, d in self.angle_ranges), key=lambda x: x[0])

    def get_closest_front_object_info(self):
        """Returns (min_distance, angle) within ±40° of front."""
        front = [(d, a) for a, d in self.angle_ranges if abs(a) <= math.radians(40)]
        if not front:
            return None, 0.0
        return min(front, key=lambda x: x[0])

    def wall_follow_geometry(self):
        """Returns (wall_front_region, wall_side_region, turn_sign).
        turn_sign: -1 = turn right (follow right wall), +1 = turn left (follow left wall).
        """
        if self.wall_follow_side == 'left':
            return self.regions['fleft'], self.regions['left'], 1.0
        return self.regions['fright'], self.regions['right'], -1.0

    def is_path_to_goal_clear(self, err_theta):
        """True if front and the direction toward goal are both obstacle-free."""
        return (self.sector_min(err_theta, math.radians(15)) > self.front_slow_distance
                and self.regions['front'] > self.front_slow_distance)

    # ─── Velocity commands ────────────────────────────────────────────────────

    def set_avoidance_command(self, msg, closest_range, theta_closest):
        """Emergency avoidance: steer away from the closest detected point."""
        theta_away = self.normalize_angle(
            theta_closest + (math.pi if theta_closest <= 0.0 else -math.pi))
        msg.linear.x = (0.0 if closest_range < self.front_stop_distance
                        else self.clamp(
                            self.avoidance_kv * (closest_range - self.front_stop_distance),
                            0.0, min(self.v_max, 0.04)))
        msg.angular.z = self.clamp(self.avoidance_kw * theta_away, -self.w_max, self.w_max)

    # ─── Main control loop (20 Hz) ────────────────────────────────────────────

    def control_loop(self):
        if not self.goal_received:
            return

        msg = Twist()
        now      = self.get_clock().now()
        odom_age = self.message_age(now, self.last_odom_time)
        scan_age = self.message_age(now, self.last_scan_time)

        if not self.sensors_ready(odom_age, scan_age):
            self.cmd_pub.publish(msg)           # publish zero velocity while waiting
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

        # Goal reached
        if dist_to_goal < self.goal_tolerance:
            self.change_state('STOP')
            self.get_logger().info(f'[GOAL REACHED] dist={dist_to_goal:.2f}m')
            self.cmd_pub.publish(Twist())
            self.goal_received = False
            return

        # ── State transition logic ─────────────────────────────────────────
        if self.state == 'GO_TO_GOAL':
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                self.get_logger().info(
                    f'[WALL] obstacle at {closest_front_range:.2f}m — entering WALL_FOLLOWING '
                    f'side={self.wall_follow_side}')
                self.change_state('WALL_FOLLOWING')

        elif self.state == 'WALL_FOLLOWING':
            # Return to GO_TO_GOAL as soon as the direct path to goal is unobstructed
            closest_clear = (closest_front_range is None
                             or closest_front_range > self.avoidance_start_distance)
            if (closest_clear
                    and self.is_path_to_goal_clear(err_theta)
                    and self.regions['front'] > self.front_slow_distance):
                self.get_logger().info(
                    f'[CLEAR] path to goal open at err={err_theta:.2f}rad — resuming GO_TO_GOAL')
                self.change_state('GO_TO_GOAL')

        # ── Velocity commands ──────────────────────────────────────────────
        if self.state == 'GO_TO_GOAL':
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                # Obstacle in avoidance zone: steer away
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)

            elif abs(err_theta) > self.heading_tolerance:
                # Heading error too large: rotate toward goal, limited forward motion
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
                heading_factor = max(0.0, math.cos(err_theta))
                if heading_factor < 0.2:
                    msg.linear.x = 0.0          # nearly perpendicular: rotate in place
                else:
                    msg.linear.x = self.clamp(
                        self.k_rho * dist_to_goal * heading_factor,
                        self.min_forward_speed, self.v_max)
            else:
                # Aligned: drive straight toward goal
                msg.linear.x  = self.clamp(self.k_rho * dist_to_goal, 0.0, self.v_max)
                msg.angular.z = 0.0

            # Near-goal speed cap
            if dist_to_goal < self.near_goal_slow_distance:
                factor = self.clamp(dist_to_goal / self.near_goal_slow_distance, 0.25, 1.0)
                msg.linear.x = min(msg.linear.x, self.near_goal_v_max * factor)

            # Front obstacle slow-down band
            if (msg.linear.x > 0.0
                    and closest_front_range is not None
                    and closest_front_range < self.front_slow_distance):
                clearance = closest_front_range - self.front_stop_distance
                band = self.front_slow_distance - self.front_stop_distance
                msg.linear.x *= self.clamp(clearance / band, 0.0, 1.0)

        elif self.state == 'WALL_FOLLOWING':
            wall_front, wall_side, turn_sign = self.wall_follow_geometry()

            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                # Obstacle directly ahead: use avoidance command
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)

            elif self.regions['front'] < self.front_stop_distance:
                # Wall directly ahead: stop and rotate away from followed side
                msg.linear.x  = 0.0
                msg.angular.z = -turn_sign * self.w_max

            elif wall_front < self.wall_threshold:
                # Diagonal wall ahead on followed side: align parallel (turn away slightly)
                msg.linear.x  = 0.025
                msg.angular.z = -turn_sign * 0.22

            elif wall_side < self.wall_threshold:
                if wall_side < 0.18:
                    # Too close to wall: back off
                    msg.linear.x  = 0.03
                    msg.angular.z = -turn_sign * 0.18
                else:
                    # Good distance: advance with slight correction toward wall
                    msg.linear.x  = 0.04
                    msg.angular.z = turn_sign * 0.03

            else:
                # Wall lost (corner / gap): turn hard toward followed side to reacquire
                msg.linear.x  = 0.02
                msg.angular.z = turn_sign * self.w_max

        self.cmd_pub.publish(msg)
        self.publish_diagnostics(msg, dist_to_goal, err_theta, odom_age, scan_age)

    # ─── Sensor validation ────────────────────────────────────────────────────

    def sensors_ready(self, odom_age, scan_age):
        """Returns True only if all required sensors have fresh data."""
        odom_stale = self.require_odom and (odom_age is None or odom_age > self.sensor_timeout)
        scan_stale = self.require_scan and (scan_age is None or scan_age > self.sensor_timeout)
        return not odom_stale and not scan_stale

    # ─── Diagnostics (throttled to every 2 s) ─────────────────────────────────

    def publish_diagnostics(self, cmd, dist, err_theta, odom_age=None, scan_age=None):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return
        self.last_diagnostic_time = now

        odom_age = odom_age or self.message_age(now, self.last_odom_time)
        scan_age = scan_age or self.message_age(now, self.last_scan_time)

        if odom_age is None or odom_age > self.sensor_timeout:
            self.get_logger().warn(f'[STALE] odom age={self.format_age(odom_age)} — holding zero velocity')
        if scan_age is None or scan_age > self.sensor_timeout:
            self.get_logger().warn(f'[STALE] scan age={self.format_age(scan_age)} — holding zero velocity')

        dist_text = 'no_odom' if dist      is None else f'{dist:.2f}'
        err_text  = 'no_odom' if err_theta is None else f'{err_theta:.2f}'
        _, wall_side, _ = self.wall_follow_geometry()

        self.get_logger().info(
            f'[BUG0] state={self.state} | '
            f'cmd v={cmd.linear.x:.2f} w={cmd.angular.z:.2f} | '
            f'dist={dist_text} err={err_text} | '
            f'odom={self.format_age(odom_age)} scan={self.format_age(scan_age)} | '
            f'closest={self.format_closest()} | '
            f'wall_side={self.wall_follow_side} '
            f'wall_dist={wall_side:.2f} front={self.regions["front"]:.2f} | '
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
        self.goal_received = True
        self.change_state('GO_TO_GOAL')
        self.get_logger().info(
            f'[GOAL] target=({self.target_x},{self.target_y}) '
            f'from=({self.x:.2f},{self.y:.2f})')

    def scan_callback(self, msg):
        self.last_scan_time = self.get_clock().now()
        self.angle_ranges = []
        for i, d in enumerate(msg.ranges):
            # Rotate scan so 0 rad = robot forward (scan_front_angle corrects LiDAR mounting offset)
            a = self.normalize_angle(
                msg.angle_min + i * msg.angle_increment - math.radians(self.scan_front_angle))
            if math.isinf(d) or math.isnan(d) or d > msg.range_max:
                d = 10.0
            elif d < max(msg.range_min, 0.12):
                d = 0.01          # clamp noise/reflections instead of dropping
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
    node = Bug0Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()