#!/usr/bin/env python3 
import rclpy 
from rclpy import qos
from rclpy.node import Node 
from geometry_msgs.msg import Twist, Pose2D 
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry 
from std_msgs.msg import Float32
import math 
import signal 
import sys 

def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class Bug0Node(Node):  
    def __init__(self):  
        super().__init__('bug0_node') 
        latched = qos.QoSProfile(depth=1)
        latched.durability = qos.QoSDurabilityPolicy.TRANSIENT_LOCAL

        
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', latched)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, latched)
        self.odom_sensor_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback, qos.qos_profile_sensor_data
        )
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, latched)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, latched)
        self.scan_sensor_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback, qos.qos_profile_sensor_data
        )
        self.laser_distance_sub = self.create_subscription(
            Float32, 'LaserDistance', self.laser_distance_callback, 10
        )
        self.laser_distance_sensor_sub = self.create_subscription(
            Float32, 'LaserDistance', self.laser_distance_callback, qos.qos_profile_sensor_data
        )
        self.servo_angle_pub = self.create_publisher(Float32, 'ServoAngle', 10)
        self.servo_angle_sub = self.create_subscription(Float32, 'ServoAngle', self.servo_angle_callback, 10)
        self.servo_angle_sensor_sub = self.create_subscription(
            Float32, 'ServoAngle', self.servo_angle_callback, qos.qos_profile_sensor_data
        )

        signal.signal(signal.SIGINT, self.shutdown_function) 

        self.state = "WAITING" 
        self.goal_received = False 
        
        self.x = 0.0 
        self.y = 0.0 
        self.theta = 0.0 
        self.target_x = 0.0 
        self.target_y = 0.0 
        self.last_odom_time = None
        self.last_scan_time = None
        self.last_laser_distance_time = None
        self.last_servo_angle_time = None
        self.last_diagnostic_time = self.get_clock().now()
        self.servo_angle = 0.0
        self.servo_command_angle = 0.0
        self.servo_direction = 1.0
        self.servo_samples = []
        self.closest_front_range = None
        self.closest_front_angle = 0.0
        self.closest_range = None
        self.closest_angle = 0.0
        
        self.d_thresh = 0.24  
        
        # Control proporcional hacia la meta
        self.declare_parameter('goal_tolerance', 0.05)
        self.declare_parameter('k_rho', 0.6)
        self.declare_parameter('k_alpha', 1.5)
        self.declare_parameter('v_max', 0.06)
        self.declare_parameter('w_max', 0.30)
        self.declare_parameter('heading_tolerance', 0.15)
        self.declare_parameter('rotate_in_place', False)
        self.declare_parameter('min_forward_speed', 0.02)
        self.declare_parameter('require_scan', True)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('sensor_timeout', 1.0)
        self.declare_parameter('servo_angle_in_degrees', True)
        self.declare_parameter('servo_window', 1.5)
        self.declare_parameter('servo_front_angle', 0.0)
        self.declare_parameter('front_stop_distance', 0.22)
        self.declare_parameter('front_slow_distance', 0.38)
        self.declare_parameter('avoidance_start_distance', 0.38)
        self.declare_parameter('avoidance_kv', 0.5)
        self.declare_parameter('avoidance_kw', 0.7)
        self.declare_parameter('laser_distance_scale', 1.0)
        self.declare_parameter('scan_front_angle', 0.0)
        self.declare_parameter('obstacle_source', 'scan')
        self.declare_parameter('servo_sweep_enabled', False)
        self.declare_parameter('servo_min_angle', -60.0)
        self.declare_parameter('servo_max_angle', 60.0)
        self.declare_parameter('servo_step', 15.0)
        self.declare_parameter('servo_period', 0.12)

        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.k_rho = self.get_parameter('k_rho').value
        self.k_alpha = self.get_parameter('k_alpha').value
        self.v_max = self.get_parameter('v_max').value
        self.w_max = self.get_parameter('w_max').value
        self.heading_tolerance = self.get_parameter('heading_tolerance').value
        self.rotate_in_place = self.get_parameter('rotate_in_place').value
        self.min_forward_speed = self.get_parameter('min_forward_speed').value
        self.require_scan = self.get_parameter('require_scan').value
        self.require_odom = self.get_parameter('require_odom').value
        self.sensor_timeout = self.get_parameter('sensor_timeout').value
        self.servo_angle_in_degrees = self.get_parameter('servo_angle_in_degrees').value
        self.servo_window = self.get_parameter('servo_window').value
        self.servo_front_angle = self.get_parameter('servo_front_angle').value
        self.front_stop_distance = self.get_parameter('front_stop_distance').value
        self.front_slow_distance = self.get_parameter('front_slow_distance').value
        self.avoidance_start_distance = self.get_parameter('avoidance_start_distance').value
        self.avoidance_kv = self.get_parameter('avoidance_kv').value
        self.avoidance_kw = self.get_parameter('avoidance_kw').value
        self.laser_distance_scale = self.get_parameter('laser_distance_scale').value
        self.scan_front_angle = self.get_parameter('scan_front_angle').value
        self.obstacle_source = self.get_parameter('obstacle_source').value
        self.servo_sweep_enabled = self.get_parameter('servo_sweep_enabled').value
        self.servo_min_angle = self.get_parameter('servo_min_angle').value
        self.servo_max_angle = self.get_parameter('servo_max_angle').value
        self.servo_step = self.get_parameter('servo_step').value
        self.servo_period = self.get_parameter('servo_period').value

        self.angle_ranges = []

        self.regions = {
            'front': 10.0, 'fright': 10.0, 'right': 10.0, 'bright': 10.0,
            'back': 10.0, 'bleft': 10.0, 'left': 10.0, 'fleft': 10.0,
        }

        # Ejecuta el bucle de control a 20 Hz para reducir desfases durante las maniobras
        self.create_timer(0.05, self.control_loop)
        self.create_timer(self.servo_period, self.servo_sweep_callback)
        self.get_logger().info("Nodo Bug 0 inicializado. Esperando meta en /goal...") 

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def change_state(self, new_state):
        if self.state != new_state:
            self.get_logger().info(f'Cambiando de estado: {self.state} -> {new_state}')
            self.state = new_state

    def sector_min(self, center_angle, half_width, default=10.0):
        values = [
            distance for angle, distance in self.angle_ranges
            if abs(self.normalize_angle(angle - center_angle)) <= half_width
        ]
        return min(values) if values else default

    def is_path_to_goal_clear(self, err_theta):
        return self.sector_min(err_theta, math.radians(15)) > (self.d_thresh + 0.15)

    def get_closest_front_object_info(self):
        front_values = [
            (distance, angle) for angle, distance in self.angle_ranges
            if abs(angle) <= math.radians(70)
        ]
        if not front_values:
            return None, 0.0
        return min(front_values, key=lambda item: item[0])

    def get_closest_object_info(self):
        if not self.angle_ranges:
            return None, 0.0
        return min(
            ((distance, angle) for angle, distance in self.angle_ranges),
            key=lambda item: item[0],
        )

    def set_avoidance_command(self, msg, closest_range, theta_closest):
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
            self.avoidance_kw * theta_avoidance,
            -self.w_max,
            self.w_max,
        )

    def control_loop(self): 
        if not self.goal_received:
            return

        msg = Twist()
        now = self.get_clock().now()
        odom_age = self.message_age(now, self.last_odom_time)
        obstacle_age = self.obstacle_age(now)

        if not self.sensors_ready(odom_age, obstacle_age):
            self.cmd_pub.publish(msg)
            self.publish_diagnostics(msg, None, None, odom_age, obstacle_age)
            return
        
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        err_theta = self.normalize_angle(angle_to_goal - self.theta)
        closest_range, theta_closest = self.get_closest_object_info()
        closest_front_range, closest_front_angle = self.get_closest_front_object_info()

        self.closest_range = closest_range
        self.closest_angle = theta_closest
        self.closest_front_range = closest_front_range
        self.closest_front_angle = closest_front_angle

        if dist_to_goal < self.goal_tolerance:
            self.change_state("STOP")
            self.get_logger().info('Meta alcanzada con Bug 0.')
            self.cmd_pub.publish(Twist()) 
            self.goal_received = False 
            return

        # ---------------- TRANSICIONES DE ESTADO ---------------- #
        if self.state == "GO_TO_GOAL":
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                self.change_state("WALL_FOLLOWING")
                
        elif self.state == "WALL_FOLLOWING":
            # Condición de retorno a GO_TO_GOAL cuando la ruta hacia la meta se libera
            closest_clear = closest_front_range is None or closest_front_range > self.avoidance_start_distance
            if closest_clear and self.is_path_to_goal_clear(err_theta) and self.regions['front'] > self.front_slow_distance:
                self.get_logger().info('Ruta a la meta despejada. Cambiando a GO_TO_GOAL.')
                self.change_state("GO_TO_GOAL")

        # ---------------- ACCIONES DE ESTADO ---------------- #
        # Estructura de control independiente para evitar bloqueos de velocidad
        if self.state == "GO_TO_GOAL":
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)
            elif abs(err_theta) > self.heading_tolerance:
                msg.angular.z = self.clamp(self.k_alpha * err_theta, -self.w_max, self.w_max)
                if self.rotate_in_place:
                    msg.linear.x = 0.0
                else:
                    heading_factor = max(0.0, math.cos(err_theta))
                    if heading_factor < 0.2:
                        msg.linear.x = 0.0
                    else:
                        forward_speed = self.k_rho * dist_to_goal * heading_factor
                        msg.linear.x = self.clamp(forward_speed, self.min_forward_speed, self.v_max)
            else:
                msg.linear.x = self.clamp(self.k_rho * dist_to_goal, -self.v_max, self.v_max)
                msg.angular.z = 0.0  

            if msg.linear.x > 0.0 and closest_front_range is not None and closest_front_range < self.front_slow_distance:
                clearance = closest_front_range - self.front_stop_distance
                slow_band = self.front_slow_distance - self.front_stop_distance
                speed_factor = self.clamp(clearance / slow_band, 0.0, 1.0)
                msg.linear.x *= speed_factor
                
        elif self.state == "WALL_FOLLOWING":
            if closest_front_range is not None and closest_front_range < self.avoidance_start_distance:
                self.set_avoidance_command(msg, closest_front_range, closest_front_angle)

            # Algoritmo de seguimiento con ajustes basados en Bug 2
            # 1. Pared frontal: detiene y gira a la izquierda
            elif self.regions['front'] < self.front_stop_distance:
                msg.linear.x = 0.0
                msg.angular.z = self.w_max
            
            # 2. Pared diagonal: alinea la trayectoria en paralelo
            elif self.regions['fright'] < self.d_thresh:
                msg.linear.x = 0.025
                msg.angular.z = 0.22
            
            # 3. Pared lateral derecha detectada
            elif self.regions['right'] < self.d_thresh:
                # Corrección suave cuando la distancia es demasiado corta
                if self.regions['right'] < 0.18:
                    msg.linear.x = 0.03
                    msg.angular.z = 0.18
                # Distancia adecuada: mantiene el seguimiento de la pared
                else:
                    msg.linear.x = 0.04
                    msg.angular.z = -0.03  
            
            # 4. Esquina o pérdida de pared: gira a la derecha para recuperar el trayecto
            else:
                msg.linear.x = 0.02
                msg.angular.z = -self.w_max

        self.cmd_pub.publish(msg)
        self.publish_diagnostics(msg, dist_to_goal, err_theta, odom_age, obstacle_age)

    def sensors_ready(self, odom_age, scan_age):
        odom_missing = self.require_odom and (odom_age is None or odom_age > self.sensor_timeout)
        scan_missing = self.require_scan and (scan_age is None or scan_age > self.sensor_timeout)
        return not odom_missing and not scan_missing

    def publish_diagnostics(self, cmd_msg, dist_to_goal, err_theta, odom_age=None, scan_age=None):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < 2.0e9:
            return

        self.last_diagnostic_time = now
        if odom_age is None:
            odom_age = self.message_age(now, self.last_odom_time)
        if scan_age is None:
            scan_age = self.obstacle_age(now)

        if odom_age is None or odom_age > 1.0:
            self.get_logger().warn(
                'No estoy recibiendo /odom reciente; revisa odometria o lanza use_localisation:=true.'
            )
        if scan_age is None or scan_age > 1.0:
            self.get_logger().warn(
                'No estoy recibiendo sensor de obstaculos reciente; el robot se mantiene detenido.'
            )

        self.get_logger().info(
            f'estado={self.state}, x={self.x:.2f}, y={self.y:.2f}, theta={self.theta:.2f}'
        )

    def obstacle_age(self, now):
        scan_age = self.message_age(now, self.last_scan_time)
        laser_age = self.message_age(now, self.last_laser_distance_time)

        if self.obstacle_source == 'scan':
            return scan_age
        if self.obstacle_source == 'laser_distance':
            return laser_age

        ages = [age for age in (scan_age, laser_age) if age is not None]
        return min(ages) if ages else None

    def message_age(self, now, last_time):
        if last_time is None:
            return None
        return (now - last_time).nanoseconds * 1.0e-9

    def format_age(self, age):
        if age is None:
            return 'nunca'
        return f'{age:.1f}s'

    def format_regions(self):
        return (
            f"front={self.regions['front']:.2f}, fright={self.regions['fright']:.2f}, "
            f"right={self.regions['right']:.2f}, bright={self.regions['bright']:.2f}, "
            f"back={self.regions['back']:.2f}, bleft={self.regions['bleft']:.2f}, "
            f"left={self.regions['left']:.2f}, fleft={self.regions['fleft']:.2f}"
        )

    def format_closest(self):
        if self.closest_range is None:
            return 'none'
        return f'{self.closest_range:.2f}@{math.degrees(self.closest_angle):.0f}deg'

    def servo_sweep_callback(self):
        if not self.servo_sweep_enabled:
            return

        self.servo_command_angle += self.servo_direction * self.servo_step
        if self.servo_command_angle >= self.servo_max_angle:
            self.servo_command_angle = self.servo_max_angle
            self.servo_direction = -1.0
        elif self.servo_command_angle <= self.servo_min_angle:
            self.servo_command_angle = self.servo_min_angle
            self.servo_direction = 1.0

        msg = Float32()
        msg.data = self.servo_command_angle if self.servo_angle_in_degrees else math.radians(self.servo_command_angle)
        self.servo_angle_pub.publish(msg)

        angle = math.radians(self.servo_command_angle)
        self.servo_angle = self.normalize_angle(angle - math.radians(self.servo_front_angle))
        self.last_servo_angle_time = self.get_clock().now()

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
        self.change_state("GO_TO_GOAL")
        self.get_logger().info(f"Bug 0 Metarecibida: x={self.target_x}, y={self.target_y}")

    def scan_callback(self, msg): 
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
            'front':  self.sector_min(0.0, math.radians(22.5)),
            'fright': self.sector_min(math.radians(-45), math.radians(22.5)),
            'right':  self.sector_min(math.radians(-90), math.radians(22.5)),
            'bright': self.sector_min(math.radians(-135), math.radians(22.5)),
            'back':   self.sector_min(math.pi, math.radians(22.5)),
            'bleft':  self.sector_min(math.radians(135), math.radians(22.5)),
            'left':   self.sector_min(math.radians(90), math.radians(22.5)),
            'fleft':  self.sector_min(math.radians(45), math.radians(22.5)),
        }
        self.closest_front_range, self.closest_front_angle = self.get_closest_front_object_info()
        self.closest_range, self.closest_angle = self.get_closest_object_info()

    def servo_angle_callback(self, msg):
        angle = msg.data
        if self.servo_angle_in_degrees:
            angle = math.radians(angle)
        self.servo_angle = self.normalize_angle(angle - math.radians(self.servo_front_angle))
        self.last_servo_angle_time = self.get_clock().now()

    def laser_distance_callback(self, msg):
        if self.obstacle_source == 'scan':
            return

        now = self.get_clock().now()
        self.last_laser_distance_time = now

        distance = msg.data * self.laser_distance_scale
        if math.isinf(distance) or math.isnan(distance) or distance > 10.0:
            clean_distance = 10.0
        elif distance < 0.02:
            clean_distance = 0.01
        else:
            clean_distance = distance

        now_seconds = now.nanoseconds * 1.0e-9
        servo_age = self.message_age(now, self.last_servo_angle_time)
        angle = self.servo_angle if servo_age is not None and servo_age <= self.sensor_timeout else 0.0

        self.servo_samples.append((now_seconds, angle, clean_distance))
        self.servo_samples = [
            sample for sample in self.servo_samples
            if now_seconds - sample[0] <= self.servo_window
        ]
        self.angle_ranges = [(angle, distance) for _, angle, distance in self.servo_samples]

        self.regions = {
            'front':  self.sector_min(0.0, math.radians(22.5)),
            'fright': self.sector_min(math.radians(-45), math.radians(22.5)),
            'right':  self.sector_min(math.radians(-90), math.radians(22.5)),
            'bright': self.sector_min(math.radians(-135), math.radians(22.5)),
            'back':   self.sector_min(math.pi, math.radians(22.5)),
            'bleft':  self.sector_min(math.radians(135), math.radians(22.5)),
            'left':   self.sector_min(math.radians(90), math.radians(22.5)),
            'fleft':  self.sector_min(math.radians(45), math.radians(22.5)),
        }

    def shutdown_function(self, signum, frame): 
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