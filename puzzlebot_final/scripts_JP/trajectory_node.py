#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import Odometry

class TrajectoryNode(Node):
    def __init__(self):
        super().__init__('trajectory_node')
        
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        
        x0 = float(self.get_parameter('x0').value)
        y0 = float(self.get_parameter('y0').value)
        
        # SELECCIÓN DE TRAYECTORIA (Comentar/Descomentar)
        
        # OPCIÓN A: LÍNEA RECTA (1 Metro)
        self.points = [
            (x0 + 1.0, y0)
        ]

        # OPCIÓN B: TRAYECTORIA PARABÓLICA (y = x^2)
        #self.points = [
        #    (x0 + 0.25, y0 + 0.0625),
        #    (x0 + 0.50, y0 + 0.25),
        #    (x0 + 0.75, y0 + 0.5625),
        #    (x0 + 1.00, y0 + 1.00)
        #]
        
        self.current_index = 0
        self.last_flag = False 
        
        self.pub = self.create_publisher(Odometry, 'set_point', 10)
        self.create_subscription(Bool, 'next_point', self.flag_callback, 10)
        
        self.timer = self.create_timer(1.0, self.publish_point)

    def flag_callback(self, msg):
        current_flag = msg.data
        if current_flag and not self.last_flag: 
            self.current_index += 1
            if self.current_index >= len(self.points):
                self.get_logger().info('Experimento de línea recta (1 metro) completado.')
                self.current_index = len(self.points) - 1
            else:
                self.get_logger().info(f'Transitando al vértice {self.current_index + 1}')
        
        self.last_flag = current_flag

    def publish_point(self):
        if self.current_index < len(self.points):
            x, y = self.points[self.current_index]
            odom_msg = Odometry()
            odom_msg.pose.pose.position.x = x
            odom_msg.pose.pose.position.y = y
            self.pub.publish(odom_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()