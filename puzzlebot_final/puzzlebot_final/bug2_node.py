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
        self.scan_offset = math.radians(0.0)   # offset de montaje del lidar — calibrar abajo
        
        # --- Datos del LiDAR ---
        self.regions = {'front': 10.0, 'left': 10.0, 'right': 10.0, 'fleft': 10.0, 'fright': 10.0}

        # --- Parámetros Sintonizables ---
        self.d_stop = 0.25         # Freno inminente frontal
        self.d_wall = 0.25         # Distancia deseada a la pared
        self.min_progress = 0.10   # minProgress (Diapositiva 47)
        self.m_line_tol = 0.10     # Tolerancia de intercepción M-Line (Diapositiva 48)
        self.goal_tol = 0.15       
        
        self.v_max = 0.10          
        self.w_max = 0.50
        self.kp_wall = 1.5         
        self.kp_heading = 1.5      

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info('Pure Bug2: Lógica de esquinas y Paro de Emergencia activos.')

    def normalize_angle(self, angle):
        while angle > math.pi: angle -= 2.0 * math.pi
        while angle < -math.pi: angle += 2.0 * math.pi
        return angle

    def clamp(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)

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
        self.get_logger().info(f'Meta Recibida: ({self.goal_x:.2f}, {self.goal_y:.2f}).')

def scan_callback(self, msg):
    n = len(msg.ranges)
    if n == 0:
        return

    sectors = {'front': float('inf'), 'fleft': float('inf'), 'left': float('inf'),
               'fright': float('inf'), 'right': float('inf')}

    for i, r in enumerate(msg.ranges):
        # Limpieza de lecturas
        if math.isinf(r) or math.isnan(r) or r > msg.range_max:
            d = 10.0
        elif r < max(msg.range_min, 0.12):
            d = 0.01
        else:
            d = r

        # Ángulo REAL del rayo respecto al frente del robot (0 = frente, + = izquierda)
        ang = self.normalize_angle(msg.angle_min + i * msg.angle_increment + self.scan_offset)
        deg = math.degrees(ang)

        if   -15 <= deg <=  15: key = 'front'
        elif  15 <  deg <=  70: key = 'fleft'
        elif  70 <  deg <= 110: key = 'left'
        elif -70 <= deg <  -15: key = 'fright'
        elif -110 <= deg < -70: key = 'right'
        else:
            continue

        if d < sectors[key]:
            sectors[key] = d

    self.regions = {k: (v if v != float('inf') else 10.0) for k, v in sectors.items()}

    # --- DEBUG de calibración (quítalo cuando ya esté afinado) ---
    self.get_logger().info(
        f"F={self.regions['front']:.2f} FL={self.regions['fleft']:.2f} "
        f"L={self.regions['left']:.2f} FR={self.regions['fright']:.2f} "
        f"R={self.regions['right']:.2f}", throttle_duration_sec=0.5)

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
                
                self.get_logger().info(f'Impacto. d_hit={self.d_hit:.2f}. Muro asignado: {self.state}')
                
            else:
                angle_to_goal = math.atan2(self.goal_y - self.y, self.goal_x - self.x)
                err_theta = self.normalize_angle(angle_to_goal - self.theta)
                
                msg.angular.z = self.clamp(self.kp_heading * err_theta, -self.w_max, self.w_max)
                if abs(err_theta) < 0.2:
                    msg.linear.x = self.v_max
                else:
                    msg.linear.x = 0.0 

        elif self.state in ['WALL_FOLLOWING_CW', 'WALL_FOLLOWING_CCW']:
            d_line = self.distance_to_m_line()
            progress_condition = d_gtg < (self.d_hit - self.min_progress)
            path_clear = self.regions['front'] > 0.4
            
            # Condición de Salida Estricta (Diapositivas)
            if d_line < self.m_line_tol and progress_condition and path_clear:
                self.get_logger().info(f'M-Line Interceptada (Progreso válido). Regresando a GTG.')
                self.state = 'GO_TO_GOAL'
            else:
                # --- CONTROLADORES DE MURO ROBUSTOS ---
                if self.state == 'WALL_FOLLOWING_CW':  # Pared Izquierda
                    if self.regions['front'] < self.d_stop or self.regions['fleft'] < self.d_stop * 0.8:
                        # 1. Esquina Interna: Bloqueo frontal
                        msg.linear.x = 0.0
                        msg.angular.z = -self.w_max 
                    elif self.regions['left'] > self.d_wall + 0.15:
                        # 2. Esquina Externa: Perdió la pared, gira para buscarla
                        msg.linear.x = self.v_max * 0.3
                        msg.angular.z = self.w_max * 0.8
                    else:
                        # 3. Muro Recto: Proporcional
                        error = self.regions['left'] - self.d_wall
                        msg.linear.x = self.v_max * 0.6 
                        msg.angular.z = self.clamp(self.kp_wall * error, -self.w_max, self.w_max)
                        
                elif self.state == 'WALL_FOLLOWING_CCW': # Pared Derecha
                    if self.regions['front'] < self.d_stop or self.regions['fright'] < self.d_stop * 0.8:
                        # 1. Esquina Interna: Bloqueo frontal
                        msg.linear.x = 0.0
                        msg.angular.z = self.w_max 
                    elif self.regions['right'] > self.d_wall + 0.15:
                        # 2. Esquina Externa: Perdió la pared, gira para buscarla
                        msg.linear.x = self.v_max * 0.3
                        msg.angular.z = -self.w_max * 0.8
                    else:
                        # 3. Muro Recto: Proporcional
                        error = self.d_wall - self.regions['right'] 
                        msg.linear.x = self.v_max * 0.6 
                        msg.angular.z = self.clamp(self.kp_wall * error, -self.w_max, self.w_max)

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PureBug2Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('\n[EMERGENCIA] Ctrl+C detectado. Deteniendo motores...')
    finally:
        # [CORRECCIÓN CRÍTICA]: Drenar la cola DDS antes de matar el proceso
        stop_msg = Twist()
        for _ in range(10):
            node.cmd_pub.publish(stop_msg)
            rclpy.spin_once(node, timeout_sec=0.02)
        
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()