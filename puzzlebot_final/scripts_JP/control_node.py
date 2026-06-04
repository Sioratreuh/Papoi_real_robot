#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')
        
        # Suscriptores
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(Odometry, 'set_point', self.setpoint_callback, 10)
        
        # Publicadores
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.flag_pub = self.create_publisher(Bool, 'next_point', 10)
        
        # Estado actual
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        
        # Objetivo
        self.target_x = None
        self.target_y = None
        
        # Parámetros del controlador (Setpoints)
        self.kw = 2.0  # Ganancia angular
        self.kv = 0.5  # Ganancia lineal
        self.distance_tolerance = 0.05  # 5 cm de tolerancia para dar por alcanzado el punto
        
        # Bucle de control a 20 Hz
        self.create_timer(0.05, self.control_loop)

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        
        # Convertir cuaternión a Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.current_theta = math.atan2(siny_cosp, cosy_cosp)

    def setpoint_callback(self, msg):
        self.target_x = msg.pose.pose.position.x
        self.target_y = msg.pose.pose.position.y

    def control_loop(self):
        if self.target_x is None or self.target_y is None:
            return
            
        # Calcular errores
        dx = self.target_x - self.current_x
        dy = self.target_y - self.current_y
        distance = math.sqrt(dx**2 + dy**2)
        
        msg_vel = Twist()
        
        # Verificar si ya llegamos al objetivo
        if distance < self.distance_tolerance:
            self.cmd_pub.publish(msg_vel) # Detener el robot (v=0, w=0)
            self.flag_pub.publish(Bool(data=True)) # Avisar que llegamos
            return
            
        target_angle = math.atan2(dy, dx)
        angle_error = target_angle - self.current_theta
        
        # Normalizar el ángulo para que siempre busque el camino más corto (-pi a pi)
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))
        
        # Ley de control
        # Limitamos la velocidad lineal máxima a 0.2 m/s por seguridad
        msg_vel.linear.x = min(0.2, self.kv * distance) 
        msg_vel.angular.z = self.kw * angle_error
        
        self.cmd_pub.publish(msg_vel)
        self.flag_pub.publish(Bool(data=False)) # Aún estamos en camino

def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()