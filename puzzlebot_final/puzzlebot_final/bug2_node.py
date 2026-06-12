#!/usr/bin/env python3
import math
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Pose2D, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class PureBug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')
        
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.goal_sub = self.create_subscription(Pose2D, 'goal', self.goal_callback, latched)
        
        qos_sensor = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_sensor)

        # --- Variables de Estado ---
        self.state = 'WAITING'
        self.x = self.y = self.theta = 0.0
        self.goal_x = self.goal_y = 0.0
        self.start_x = self.start_y = 0.0
        self.goal_received = False
        
        # --- Variables Matemáticas Bug 2 ---
        self.A = self.B = self.C = self.denom_line = 0.0
        self.d_hit = float('inf') 
        
        # --- Datos del LiDAR ---
        self.regions = {'front': 10.0, 'left': 10.0, 'right': 10.0, 'fleft': 10.0, 'fright': 10.0}

        # --- Parámetros Sintonizables ---
        self.d_stop = 0.25         # Distancia para detenerse frente a obstáculo (m)
        self.d_wall = 0.25         # Distancia deseada para seguir la pared (m)
        self.min_progress = 0.10   # minProgress para soltar pared (m)
        self.m_line_tol = 0.10     # Tolerancia para d_line (m)
        self.goal_tol = 0.15       # Tolerancia de llegada a meta (m)
        
        self.v_max = 0.10          # Reducido un poco para dar más tiempo de reacción físico
        self.w_max = 0.50
        self.kp_wall = 1.5         
        self.kp_heading = 1.5      

        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info('Pure Bug2 Inicializado. Protección de colisión y paro seguro activos.')

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    # --- Callbacks ---
    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def goal_callback(self, msg):
        self.goal_x = msg.x
        self.goal_y = msg.y
        self.start_x = self.x
        self.start_y = self.y
        
        self.A = self.goal_y - self.start_y
        self.B = -(self.goal_x - self.start_x)
        self.C = (self.goal_x * self.start_y) - (self.goal_y * self.start_x)
        self.denom_line = math.hypot(self.A, self.B)
        
        self.goal_received = True
        self.state = 'GO_TO_GOAL'
        self.get_logger().info(f'Meta Recibida: ({self.goal_x}, {self.goal_y}).')

    def scan_callback(self, msg):
        ranges = []
        for r in msg.ranges:
            if math.isinf(r) or math.isnan(r) or r > msg.range_max:
                ranges.append(10.0) # Fuera de rango o sin eco
            elif r < max(msg.range_min, 0.12):
                ranges.append(0.01) # [CORRECCIÓN]: Peligro inminente, punto ciego del LiDAR
            else:
                ranges.append(r)
        
        num_rays = len(ranges)
        if num_rays == 0: return
        
        def get_min_in_sector(start_angle, end_angle):
            start_idx = int((start_angle / 360.0) * num_rays)
            end_idx = int((end_angle / 360.0) * num_rays)
            if start_idx < end_idx:
                sector = ranges[start_idx:end_idx]
            else:
                sector = ranges[start_idx:] + ranges[:end_idx]
            return min(sector) if sector else 10.0

        self.regions = {
            'front':  min(get_min_in_sector(345, 360), get_min_in_sector(0, 15)),
            'fleft':  get_min_in_sector(15, 70),
            'left':   get_min_in_sector(70, 110),
            'fright': get_min_in_sector(290, 345),
            'right':  get_min_in_sector(250, 290)
        }

    # --- Matemáticas Principales ---
    def distance_to_goal(self):
        return math.hypot(self.goal_x - self.x, self.goal_y - self.y)
        
    def distance_to_m_line(self):
        if self.denom_line == 0: return 0.0
        return abs(self.A * self.x + self.B * self.y + self.C) / self.denom_line

    # --- Bucle de Control Central ---
    def control_loop(self):
        if not self.goal_received:
            return

        msg = Twist()
        d_gtg = self.distance_to_goal()
        
        if d_gtg < self.goal_tol:
            self.state = 'STOP'
            self.cmd_pub.publish(Twist())
            self.goal_received = False
            self.get_logger().info('¡Meta Alcanzada!')
            return

        if self.state == 'GO_TO_GOAL':
            if self.regions['front'] < self.d_stop or self.regions['fleft'] < self.d_stop or self.regions['fright'] < self.d_stop:
                self.d_hit = d_gtg 
                
                if self.regions['fleft'] < self.regions['fright']:
                    self.state = 'WALL_FOLLOWING_CW'
                else:
                    self.state = 'WALL_FOLLOWING_CCW'
                
                self.get_logger().info(f'Impacto. d_hit={self.d_hit:.2f}. Estado: {self.state}')
                
            else:
                angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
                err_theta = self.normalize_angle(angle_to_goal - self.theta)
                
                msg.angular.z = max(min(self.kp_heading * err_theta, self.w_max), -self.w_max)
                if abs(err_theta) < 0.2:
                    msg.linear.x = self.v_max
                else:
                    msg.linear.x = 0.0 

        elif self.state in ['WALL_FOLLOWING_CW', 'WALL_FOLLOWING_CCW']:
            d_line = self.distance_to_m_line()
            progress_condition = d_gtg < (self.d_hit - self.min_progress)
            path_clear = self.regions['front'] > 0.4
            
            if d_line < self.m_line_tol and progress_condition and path_clear:
                self.get_logger().info(f'M-Line Interceptada. Regresando a GTG.')
                self.state = 'GO_TO_GOAL'
            else:
                if self.state == 'WALL_FOLLOWING_CW': 
                    if self.regions['front'] < self.d_stop:
                        msg.linear.x = 0.0
                        msg.angular.z = -self.w_max 
                    else:
                        error = self.regions['left'] - self.d_wall
                        msg.linear.x = self.v_max * 0.6  # [CORRECCIÓN] Avanza más lento en las curvas
                        msg.angular.z = max(min(self.kp_wall * error, self.w_max), -self.w_max)
                        
                elif self.state == 'WALL_FOLLOWING_CCW': 
                    if self.regions['front'] < self.d_stop:
                        msg.linear.x = 0.0
                        msg.angular.z = self.w_max 
                    else:
                        error = self.d_wall - self.regions['right'] 
                        msg.linear.x = self.v_max * 0.6  # [CORRECCIÓN] Avanza más lento en las curvas
                        msg.angular.z = max(min(self.kp_wall * error, self.w_max), -self.w_max)

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PureBug2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Interrupción de teclado detectada. Frenando emergencia...')
    finally:
        # [CORRECCIÓN] Bucle de paro seguro para Micro-ROS
        stop_msg = Twist()
        for _ in range(5):
            node.cmd_pub.publish(stop_msg)
            time.sleep(0.05)  # Da tiempo a que DDS envíe el paquete antes de matar el proceso
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()