#!/usr/bin/env python3
# BUG2 FÍSICO: CONTROL REACTIVO CON LIDAR (SIN MAPA, SIN MEMORIA)
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


class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')
        latched = qos.QoSProfile(depth=1)
        latched.durability = qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, latched)
        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos.qos_profile_sensor_data
        )

        signal.signal(signal.SIGINT, self.shutdown_function)

        self.state = 'WAITING'
        self.goal_received = False

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.start_x = 0.0
        self.start_y = 0.0
        self.hit_x = 0.0
        self.hit_y = 0.0
        self.hit_distance = float('inf')
        self.left_hit_region = False
        self.best_dist_to_goal = float('inf')

        self.last_odom_time = None
        self.last_scan_time = None
        self.last_diagnostic_time = self.get_clock().now()

        self.angle_ranges = []
        self.closest_range = None
        self.closest_angle = 0.0
        self.closest_front_range = None
        self.closest_front_angle = 0.0

        self.regions = {
            'front': 10.0, 'fright': 10.0, 'right': 10.0, 'bright': 10.0,
            'back': 10.0, 'bleft': 10.0, 'left': 10.0, 'fleft': 10.0,
        }

        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('wall_follow_goal_tolerance', 0.18)
        self.declare_parameter('goal_pass_margin', 0.02)
        self.declare_parameter('goal_pass_lateral_tolerance', 0.22)
        self.declare_parameter('near_goal_slow_distance', 0.10)
        self.declare_parameter('near_goal_v_max', 0.10)
        self.declare_parameter('m_line_tolerance', 0.10)
        self.declare_parameter('min_hit_separation', 0.35)
        self.declare_parameter('hit_return_tolerance', 0.18)
        self.declare_parameter('m_line_goal_improvement', 0.12)
        self.declare_parameter('k_rho', 0.8)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.08)
        self.declare_parameter('w_max', 0.40)
        self.declare_parameter('heading_tolerance', 0.15)
        self.declare_parameter('min_forward_speed', 0.02)
        self.declare_parameter('front_stop_distance', 0.18)
        self.declare_parameter('front_slow_distance', 0.28)
        self.declare_parameter('avoidance_start_distance', 0.25)
        self.declare_parameter('wall_follow_start_distance', 0.24)
        self.declare_parameter('wall_distance', 0.16)
        self.declare_parameter('right_too_close', 0.15)
        self.declare_parameter('avoidance_kv', 0.5)
        self.declare_parameter('avoidance_kw', 0.7)
        self.declare_parameter('sensor_timeout', 2.0)
        self.declare_parameter('require_scan', True)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('scan_front_angle', 0.0)
        self.declare_parameter('k_wall', 2.5)
        self.declare_parameter('wall_follow_speed', 0.10)
        

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.wall_follow_goal_tolerance = self.get_parameter('wall_follow_goal_tolerance').value
        self.goal_pass_margin = self.get_parameter('goal_pass_margin').value
        self.goal_pass_lateral_tolerance = self.get_parameter('goal_pass_lateral_tolerance').value
        self.near_goal_slow_distance = self.get_parameter('near_goal_slow_distance').value
        self.near_goal_v_max = self.get_parameter('near_goal_v_max').value
        self.m_line_tolerance = self.get_parameter('m_line_tolerance').value
        self.min_hit_separation = self.get_parameter('min_hit_separation').value
        self.hit_return_tolerance = self.get_parameter('hit_return_tolerance').value
        self.m_line_goal_improvement = self.get_parameter('m_line_goal_improvement').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.min_forward_speed = self.get_parameter('min_forward_speed').value
        self.front_stop_distance = self.get_parameter('front_stop_distance').value
        self.front_slow_distance = self.get_parameter('front_slow_distance').value
        self.avoidance_start_distance = self.get_parameter('avoidance_start_distance').value
        self.wall_follow_start_distance = self.get_parameter('wall_follow_start_distance').value
        self.wall_distance = self.get_parameter('wall_distance').value
        self.right_too_close = self.get_parameter('right_too_close').value
        self.avoidance_kv = self.get_parameter('avoidance_kv').value
        self.avoidance_kw = self.get_parameter('avoidance_kw').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.require_scan = self.get_parameter('require_scan').value
        self.require_odom = self.get_parameter('require_odom').value
        self.scan_front_angle = self.get_parameter('scan_front_angle').value
        self.k_wall = self.get_parameter('k_wall').value
        self.wall_follow_speed = self.get_parameter('wall_follow_speed').value

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Nodo Bug2 fisico inicializado. Esperando meta en goal...')

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value): #wtf?
        return max(min(value, max_value), min_value)

    def change_state(self, new_state): #ok?
        if self.state != new_state:
            self.get_logger().info(f'Cambiando de estado: {self.state} -> {new_state}')
            self.state = new_state

    def sector_min(self, center_angle, half_width, default=10.0): #wtf is this?????
        values = [
            distance for angle, distance in self.angle_ranges
            if abs(self.normalize_angle(angle - center_angle)) <= half_width
        ]
        return min(values) if values else default

    def get_closest_object_info(self):
        if not self.angle_ranges:
            return None, 0.0
        return min(
            ((distance, angle) for angle, distance in self.angle_ranges),
            key=lambda item: item[0],
        )

    def get_closest_front_object_info(self):
        front_values = [
            (distance, angle) for angle, distance in self.angle_ranges
            if abs(angle) <= math.radians(40)
        ]
        if not front_values:
            return None, 0.0
        return min(front_values, key=lambda item: item[0])

    def distance_to_m_line(self):# distancia a linea proyectada?
        num = abs((self.target_y - self.start_y) * self.x -
                  (self.target_x - self.start_x) * self.y +
                  self.target_x * self.start_y -
                  self.target_y * self.start_x)
        den = math.sqrt((self.target_y - self.start_y) ** 2 +
                        (self.target_x - self.start_x) ** 2)
        if den == 0.0:
            return 0.0
        return num / den

    def goal_line_progress(self):
        goal_dx = self.target_x - self.start_x
        goal_dy = self.target_y - self.start_y
        goal_length = math.sqrt(goal_dx ** 2 + goal_dy ** 2)
        if goal_length == 0.0:
            return 0.0, 0.0, 0.0

        robot_dx = self.x - self.start_x
        robot_dy = self.y - self.start_y
        progress = (robot_dx * goal_dx + robot_dy * goal_dy) / goal_length
        lateral_error = self.distance_to_m_line()
        return progress, goal_length, lateral_error

    def should_stop_for_goal(self, dist_to_goal): # need several improvements!!!
        if dist_to_goal <= self.goal_tolerance:
            return True, f'dist={dist_to_goal:.2f} m'

        if self.state == 'WALL_FOLLOWING' and dist_to_goal <= self.wall_follow_goal_tolerance:
            return True, f'captura en WALL_FOLLOWING, dist={dist_to_goal:.2f} m'

        if (self.best_dist_to_goal <= self.wall_follow_goal_tolerance and
                dist_to_goal > self.best_dist_to_goal + self.goal_pass_margin):
            return True, (
                f'ya paso cerca de meta, dist={dist_to_goal:.2f} m, '
                f'mejor={self.best_dist_to_goal:.2f} m'
            )

        progress, goal_length, lateral_error = self.goal_line_progress()
        crossed_goal_plane = progress >= (goal_length - self.goal_tolerance)
        near_goal_corridor = lateral_error <= self.goal_pass_lateral_tolerance
        if crossed_goal_plane and near_goal_corridor:
            return True, (
                f'cruzo plano de meta, progreso={progress:.2f}/{goal_length:.2f} m, '
                f'error_lateral={lateral_error:.2f} m'
            )

        return False, ''

    def stop_at_goal(self, reason):   #Could be part of the fuction above
        self.change_state('STOP')
        self.get_logger().info(f'Meta alcanzada. Deteniendo Bug2: {reason}.')
        self.cmd_pub.publish(Twist())
        self.goal_received = False

    def is_path_to_goal_clear(self, err_theta): # Dude?? this is an if in the actual BOG2 i think
        return (
            self.sector_min(err_theta, math.radians(15)) > self.front_slow_distance and
            self.regions['front'] > self.front_slow_distance
        )

    def enter_wall_following(self, dist_to_goal): # this is a function to start tracking the surface of the wall, idk if we need it
        self.hit_distance = dist_to_goal
        self.hit_x = self.x
        self.hit_y = self.y
        self.left_hit_region = False
        self.get_logger().info(
            f'Punto de impacto registrado en ({self.hit_x:.2f}, {self.hit_y:.2f}) '
            f'a {self.hit_distance:.2f} m.'
        )
        self.change_state('WALL_FOLLOWING')

    def set_avoidance_command(self, msg, closest_range, theta_closest): # huuh??? this is to avoid objects or???...
        theta_avoidance = theta_closest - math.pi if theta_closest > 0.0 else theta_closest + math.pi
        theta_avoidance = self.normalize_angle(theta_avoidance)

        if closest_range < self.front_stop_distance:
            msg.linear.x = 0.0
        else:
            clearance = closest_range - self.front_stop_distance
            msg.linear.x = self.clamp(
                self.avoidance_kv * clearance,
                0.0,
                min(self.v_max, 0.04),
            )

        msg.angular.z = self.clamp(
            self.avoidance_kw * theta_avoidance, -self.w_max,self.w_max,
        )

    def control_loop(self): #this is the main function so it should be main instead ??
        if not self.goal_received:
            return

        msg = Twist()
        now = self.get_clock().now()
        odom_age = self.message_age(now, self.last_odom_time)
        scan_age = self.message_age(now, self.last_scan_time)

        if odom_age is not None and odom_age <= self.sensor_timeout:
            dist_to_goal = math.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
            self.best_dist_to_goal = min(self.best_dist_to_goal, dist_to_goal)
            should_stop, stop_reason = self.should_stop_for_goal(dist_to_goal)
            if should_stop:
                self.stop_at_goal(stop_reason)
                return

        if not self.sensors_ready(odom_age, scan_age):
            self.cmd_pub.publish(msg)
            self.publish_diagnostics(msg, None, None, odom_age, scan_age)
            return

        dist_to_goal = math.sqrt((self.target_x - self.x) ** 2 + (self.target_y - self.y) ** 2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)

        closest_range, closest_angle = self.get_closest_object_info()
        closest_front_range, closest_front_angle = self.get_closest_front_object_info()
        self.closest_range = closest_range
        self.closest_angle = closest_angle
        self.closest_front_range = closest_front_range
        self.closest_front_angle = closest_front_angle

        should_stop, stop_reason = self.should_stop_for_goal(dist_to_goal)
        if should_stop:
            self.stop_at_goal(stop_reason)
            return

        if self.state == 'GO_TO_GOAL':
            if closest_front_range is not None and closest_front_range < self.wall_follow_start_distance:
                self.enter_wall_following(dist_to_goal)

        elif self.state == 'WALL_FOLLOWING':
            dist_m_line = self.distance_to_m_line()
            dist_to_hit = math.sqrt((self.x - self.hit_x) ** 2 + (self.y - self.hit_y) ** 2)
            closest_clear = closest_front_range is None or closest_front_range > self.avoidance_start_distance
            if dist_to_hit > self.min_hit_separation:
                self.left_hit_region = True

            returned_to_hit = (
                self.left_hit_region and
                dist_to_hit < self.hit_return_tolerance and
                dist_to_goal >= (self.hit_distance - self.m_line_goal_improvement)
            )

            if (closest_clear and dist_m_line < self.m_line_tolerance and
                dist_to_goal < (self.hit_distance - self.m_line_goal_improvement) and
                self.left_hit_region and self.is_path_to_goal_clear(err_theta)):

                self.get_logger().info(f'Linea M interceptada a {dist_to_goal:.2f} m. Cambio a GO_TO_GOAL.')
                self.change_state('GO_TO_GOAL')

            elif returned_to_hit:
                self.change_state('STOP')
                self.get_logger().warn(
                    'Regrese al punto de impacto sin encontrar una Linea M mejor. '
                    'La meta puede estar bloqueada.'
                )
                self.cmd_pub.publish(Twist())
                self.goal_received = False
                return

        if self.state == 'GO_TO_GOAL':
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)

            elif abs(err_theta) > self.heading_tolerance:
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
                heading_factor = max(0.0, math.cos(err_theta))

                if heading_factor < 0.2:
                    msg.linear.x = 0.0

                else:
                    forward_speed = self.k_rho * dist_to_goal * heading_factor
                    msg.linear.x = self.clamp(forward_speed, self.min_forward_speed, self.v_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, 0.0, self.v_max)
                msg.angular.z = 0.0

            if dist_to_goal < self.near_goal_slow_distance:
                near_factor = self.clamp(dist_to_goal / self.near_goal_slow_distance, 0.25, 1.0)
                msg.linear.x = min(msg.linear.x, self.near_goal_v_max * near_factor)
            
            if msg.linear.x > 0.0 and closest_front_range is not None and closest_front_range < self.front_slow_distance:
                clearance = closest_front_range - self.front_stop_distance
                slow_band = self.front_slow_distance - self.front_stop_distance
                msg.linear.x *= self.clamp(clearance / slow_band, 0.0, 1.0)

        elif self.state == 'WALL_FOLLOWING':
            if self.regions['front'] < self.front_stop_distance:
                msg.linear.x = 0.0
                msg.angular.z = 0.6 * self.w_max

            elif closest_front_range is not None and closest_front_range < self.front_stop_distance:
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)

            else:
                if self.regions['right'] > self.wall_follow_start_distance * 2.0:
                    # No hay pared a la derecha: avanza buscándola con giro suave
                    msg.linear.x = self.wall_follow_speed
                    msg.angular.z = -0.5 * self.k_wall * (self.regions['right'] - self.wall_distance)
                    msg.angular.z = self.clamp(msg.angular.z, -0.3 * self.w_max, 0.3 * self.w_max)

                else:
                    # Pared a la derecha visible: control proporcional normal
                    error = self.regions['right'] - self.wall_distance
                    msg.angular.z = self.clamp(-self.k_wall * error, -self.w_max, self.w_max)
                    turn_penalty = max(0.0, 1.0 - abs(msg.angular.z) / self.w_max)
                    msg.linear.x = self.clamp(
                        self.wall_follow_speed * turn_penalty,
                        self.min_forward_speed,
                        self.v_max,)

        self.cmd_pub.publish(msg)
        self.publish_diagnostics(msg, dist_to_goal, err_theta, odom_age, scan_age)

    def sensors_ready(self, odom_age, scan_age): #BROOOOOOOOOOOOOOO NO SHIT WHY THIS IS HERE?
        odom_missing = self.require_odom and (odom_age is None or odom_age > self.sensor_timeout)
        scan_missing = self.require_scan and (scan_age is None or scan_age > self.sensor_timeout)
        return not odom_missing and not scan_missing

    def publish_diagnostics(self, cmd_msg, dist_to_goal, err_theta, odom_age=None, scan_age=None): #What is this??
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return

        self.last_diagnostic_time = now
        if odom_age is None:
            odom_age = self.message_age(now, self.last_odom_time)
        if scan_age is None:
            scan_age = self.message_age(now, self.last_scan_time)

        if odom_age is None or odom_age > self.sensor_timeout:
            self.get_logger().warn('No estoy recibiendo odom reciente; el robot se mantiene detenido.')
        if scan_age is None or scan_age > self.sensor_timeout:
            self.get_logger().warn('No estoy recibiendo scan reciente; el robot se mantiene detenido.')

        dist_text = 'sin_odom' if dist_to_goal is None else f'{dist_to_goal:.2f}'
        err_text = 'sin_odom' if err_theta is None else f'{err_theta:.2f}'
        self.get_logger().info(
            f'cmd_vel: v={cmd_msg.linear.x:.2f}, w={cmd_msg.angular.z:.2f}, '
            f'estado={self.state}, dist={dist_text}, err_theta={err_text}, '
            f'odom_age={self.format_age(odom_age)}, scan_age={self.format_age(scan_age)}, '
            f'closest={self.format_closest()}, regions={self.format_regions()}'
        )

    def message_age(self, now, last_time): #again??
        if last_time is None:
            return None
        return (now - last_time).nanoseconds * 1.0e-9

    def format_age(self, age): 
        if age is None:
            return 'nunca'
        return f'{age:.1f}s'

    def format_closest(self):
        if self.closest_range is None:
            return 'none'
        return f'{self.closest_range:.2f}@{math.degrees(self.closest_angle):.0f}deg'

    def format_regions(self):
        return (
            f"front={self.regions['front']:.2f}, fright={self.regions['fright']:.2f}, "
            f"right={self.regions['right']:.2f}, bright={self.regions['bright']:.2f}, "
            f"back={self.regions['back']:.2f}, bleft={self.regions['bleft']:.2f}, "
            f"left={self.regions['left']:.2f}, fleft={self.regions['fleft']:.2f}"
        )

    def odom_callback(self, msg):
        self.last_odom_time = self.get_clock().now()
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):
        self.target_x = msg.x
        self.target_y = msg.y
        self.start_x = self.x
        self.start_y = self.y
        self.hit_distance = float('inf')
        self.left_hit_region = False
        self.best_dist_to_goal = float('inf')
        self.goal_received = True
        self.change_state('GO_TO_GOAL')
        self.get_logger().info(f'Bug2 Meta: x={self.target_x}, y={self.target_y}. Linea M trazada.')

    def scan_callback(self, msg): #get data from scan??
        self.last_scan_time = self.get_clock().now()
        self.angle_ranges = []

        for index, distance in enumerate(msg.ranges):
            angle = msg.angle_min + index * msg.angle_increment - math.radians(self.scan_front_angle)
            angle = self.normalize_angle(angle)

            if math.isinf(distance) or math.isnan(distance) or distance > msg.range_max:
                clean_distance = 10.0
            elif distance < max(msg.range_min, 0.12):
                clean_distance = 0.01
            else:
                clean_distance = distance

            self.angle_ranges.append((angle, clean_distance))

        self.regions = {
            'front': self.sector_min(0.0, math.radians(22.5)),
            'fright': self.sector_min(math.radians(-45), math.radians(22.5)),
            'right': self.sector_min(math.radians(-90), math.radians(22.5)),
            'bright': self.sector_min(math.radians(-135), math.radians(22.5)),
            'back': self.sector_min(math.pi, math.radians(22.5)),
            'bleft': self.sector_min(math.radians(135), math.radians(22.5)),
            'left': self.sector_min(math.radians(90), math.radians(22.5)),
            'fleft': self.sector_min(math.radians(45), math.radians(22.5)),
        }
        self.closest_range, self.closest_angle = self.get_closest_object_info()
        self.closest_front_range, self.closest_front_angle = self.get_closest_front_object_info()

    def shutdown_function(self, signum, frame): #this need to be improved
        self.cmd_pub.publish(Twist())
        rclpy.shutdown()
        sys.exit(0)


def main(args=None): #nope need to be restructured
    rclpy.init(args=args)
    node = Bug2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()