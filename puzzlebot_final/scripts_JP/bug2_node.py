#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

def euler_from_quaternion(x, y, z, w):
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class Bug2Node(Node):
    def __init__(self):
        super().__init__('bug2_node')
        
        # Posición actual
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Coordenadas de inicio (Para la línea m)
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_recorded = False
        
        # Meta a alcanzar en nuevomaze.world
        self.target_x = 8.0
        self.target_y = 0.0
        
        # Variables de estado para Bug 2
        self.hit_distance = float('inf') # Distancia a la meta en el momento del choque
        self.state = 0 # 0 = Ir a la meta, 1 = Seguir pared
        self.d_thresh = 0.4
        
        self.regions = {
            'right': 10.0, 'fright': 10.0, 'front': 10.0, 'fleft': 10.0, 'left': 10.0,
        }
        
        # Suscriptores y publicadores
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        if not self.start_recorded:
            self.start_x = self.x
            self.start_y = self.y
            self.start_recorded = True

    def scan_callback(self, msg):
        ranges = msg.ranges
        ranges = [r if not math.isinf(r) and not math.isnan(r) else 10.0 for r in ranges]
        
        self.regions = {
            'front':  min(min(ranges[0:15] + ranges[-15:]), 10.0),
            'fleft':  min(min(ranges[16:75]), 10.0),
            'left':   min(min(ranges[76:105]), 10.0),
            'right':  min(min(ranges[-105:-76]), 10.0),
            'fright': min(min(ranges[-75:-16]), 10.0),
        }

    def distance_to_m_line(self):
        """Calcula la distancia perpendicular desde el robot a la línea m."""
        num = abs((self.target_y - self.start_y) * self.x - 
                  (self.target_x - self.start_x) * self.y + 
                  self.target_x * self.start_y - 
                  self.target_y * self.start_x)
        den = math.sqrt((self.target_y - self.start_y)**2 + (self.target_x - self.start_x)**2)
        if den == 0:
            return 0.0
        return num / den

    def change_state(self, state):
        if state != self.state:
            self.get_logger().info(f'Cambiando a estado: {state}')
            self.state = state

    def control_loop(self):
        if not self.start_recorded:
            return

        msg = Twist()
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        
        err_theta = angle_to_goal - self.theta
        err_theta = math.atan2(math.sin(err_theta), math.cos(err_theta))

        if dist_to_goal < 0.1:
            self.change_state(2)
            self.get_logger().info('¡Meta alcanzada!')
            self.cmd_pub.publish(Twist())
            return

        # LÓGICA DE ESTADOS BUG 2
        if self.state == 0:
            # Transición: Encontrar un obstáculo
            if self.regions['front'] < self.d_thresh:
                self.hit_distance = dist_to_goal
                self.change_state(1)
                
        elif self.state == 1:
            # Transición: Interceptar la línea m más cerca de la meta
            dist_m_line = self.distance_to_m_line()
            
            # Margen de 0.1m para la línea, y asegurar que estamos más cerca que el Hit Point
            if dist_m_line < 0.1 and dist_to_goal < (self.hit_distance - 0.2):
                self.change_state(0)

        # COMPORTAMIENTO
        if self.state == 0:
            # Ir hacia la meta
            if abs(err_theta) > 0.2:
                msg.linear.x = 0.0
                msg.angular.z = 0.3 if err_theta > 0 else -0.3
            else:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                
        elif self.state == 1:
            # Wall Following (Reglas originales de Papuland)
            if self.regions['front'] < self.d_thresh:
                msg.linear.x = 0.0
                msg.angular.z = 0.5
            elif self.regions['fright'] < self.d_thresh:
                msg.linear.x = 0.1
                msg.angular.z = 0.2
            elif self.regions['right'] < self.d_thresh:
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.1
                msg.angular.z = -0.3

        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bug2Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()