#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
import math

class WaypointManager(Node):
    def __init__(self):
        super().__init__('waypoint_manager')
        latched = QoSProfile(depth=1)
        latched.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.publisher_ = self.create_publisher(Pose2D, 'goal', latched)
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        
        # RUTA DEL LABERINTO (Un solo destino final)
        # Formato: (X, Y)
        self.waypoints = [
            (2.70, -0.60),  # Meta única: Salida superior izquierda (Cerca de ArUco H)
        ]
        
        self.current_index = 0
        self.goal_tolerance = 0.20 # Tolerancia de 20cm
        self.goal_published = False

    def odom_callback(self, msg):
        if self.current_index >= len(self.waypoints):
            return

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        target_x, target_y = self.waypoints[self.current_index]

        dist = math.sqrt((target_x - x)**2 + (target_y - y)**2)

        if dist < self.goal_tolerance:
            self.get_logger().info('¡Meta final alcanzada con éxito!')
            self.current_index += 1
            self.goal_published = False

        if self.current_index < len(self.waypoints) and not self.goal_published:
            goal_msg = Pose2D()
            goal_msg.x = self.waypoints[self.current_index][0]
            goal_msg.y = self.waypoints[self.current_index][1]
            self.publisher_.publish(goal_msg)
            self.get_logger().info(f'>>> Imán activado hacia la meta única: X={goal_msg.x}, Y={goal_msg.y}')
            self.goal_published = True

def main(args=None):
    rclpy.init(args=args)
    node = WaypointManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()