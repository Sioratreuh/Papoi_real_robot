#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import numpy as np

def euler_from_quaternion(x, y, z, w):
    """Convierte cuaterniones a ángulos de Euler para obtener Theta."""
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

class Bug0Node(Node):
    def __init__(self):
        super().__init__('bug0_node')
        
        # Estado de Pose del Robot
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # Meta a alcanzar (Target Position)
        self.target_x = 8.0
        self.target_y = 0.0
        
        # LiDAR - Regiones divididas para entender el entorno
        self.regions = {
            'right': 10.0,
            'fright': 10.0,
            'front': 10.0,
            'fleft': 10.0,
            'left': 10.0,
        }
        
        # Máquina de estados: 0 = Ir a la meta, 1 = Seguir pared
        self.state = 0 
        
        # Parámetros de control
        self.d_thresh = 0.4  # A qué distancia considera que hay un obstáculo
        
        # Suscriptores y publicadores
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Bucle de control principal (10 Hz)
        self.create_timer(0.1, self.control_loop)

    def odom_callback(self, msg):
        """Actualiza la posición del robot basándose en la Odometría."""
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.theta = euler_from_quaternion(q.x, q.y, q.z, q.w)

    def scan_callback(self, msg):
        """Procesa los datos del LiDAR y los divide en 5 regiones clave."""
        ranges = msg.ranges
        # Filtramos valores infinitos o erróneos
        ranges = [r if not math.isinf(r) and not math.isnan(r) else 10.0 for r in ranges]
        
        # Asumiendo un LiDAR de 360 grados (0 al frente).
        # Se segmenta el arreglo para saber dónde está libre y dónde bloqueado.
        length = len(ranges)
        self.regions = {
            'front':  min(min(ranges[0:15] + ranges[-15:]), 10.0),
            'fleft':  min(min(ranges[16:75]), 10.0),
            'left':   min(min(ranges[76:105]), 10.0),
            'right':  min(min(ranges[-105:-76]), 10.0),
            'fright': min(min(ranges[-75:-16]), 10.0),
        }

    def change_state(self, state):
        if state is not self.state:
            self.get_logger().info(f'Cambiando a estado: {state}')
            self.state = state

    def control_loop(self):
        """El cerebro de Bug 0."""
        msg = Twist()
        
        # Distancia y ángulo a la meta
        dist_to_goal = math.sqrt((self.target_x - self.x)**2 + (self.target_y - self.y)**2)
        angle_to_goal = math.atan2(self.target_y - self.y, self.target_x - self.x)
        
        # Error de ángulo (diferencia entre a donde veo y a donde quiero ir)
        err_theta = angle_to_goal - self.theta
        # Normalización del ángulo entre -pi y pi
        err_theta = math.atan2(math.sin(err_theta), math.cos(err_theta))

        # Si ya llegamos, detenerse
        if dist_to_goal < 0.1:
            self.change_state(2) # Estado 2 = Detenido
            self.get_logger().info('¡Meta alcanzada!')
            self.cmd_pub.publish(Twist())
            return

        # ---------------------------------------------------------
        # LÓGICA BUG 0
        # ---------------------------------------------------------
        
        # TRANSICIONES DE ESTADO
        if self.state == 0: # Ir a la meta
            # Si veo algo en frente muy cerca, cambio a rodear pared
            if self.regions['front'] < self.d_thresh:
                self.change_state(1)
                
        elif self.state == 1: # Rodear pared
            # Condición de salida estricta de Bug 0: 
            # Si el frente a la meta está despejado, vuelvo a ir a la meta.
            # (Aquí evaluaremos si el ángulo hacia la meta está libre en el sensor)
            if self.regions['front'] > self.d_thresh and abs(err_theta) < 0.2:
                self.change_state(0)

        # ACCIONES DE ESTADO
        if self.state == 0:
            # Comportamiento: Go To Goal (Control Proporcional Básico)
            if abs(err_theta) > 0.2:
                # Rotar hacia la meta
                msg.linear.x = 0.0
                msg.angular.z = 0.3 if err_theta > 0 else -0.3
            else:
                # Avanzar
                msg.linear.x = 0.2
                msg.angular.z = 0.0
                
        elif self.state == 1:
            # Comportamiento: Wall Following (Esquivando hacia la izquierda)
            # Reglas simples:
            # - Si hay pared al frente, gira a la izquierda.
            # - Si hay pared a la derecha, avanza.
            # - Si no hay pared a la derecha, gira a la derecha para no perderla.
            
            if self.regions['front'] < self.d_thresh:
                # Obstáculo enfrente: Girar sobre el eje
                msg.linear.x = 0.0
                msg.angular.z = 0.5
            elif self.regions['fright'] < self.d_thresh:
                # Pared en diagonal derecha: Alejarse un poco
                msg.linear.x = 0.1
                msg.angular.z = 0.2
            elif self.regions['right'] < self.d_thresh:
                # Pared paralela a la derecha: Avanzar recto
                msg.linear.x = 0.2
                msg.angular.z = 0.0
            else:
                # Perdimos la pared: Girar buscando la pared a la derecha
                msg.linear.x = 0.1
                msg.angular.z = -0.3

        # Publicar velocidades
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()