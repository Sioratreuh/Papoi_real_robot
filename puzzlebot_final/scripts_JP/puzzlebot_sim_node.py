#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float32

class PuzzlebotSim(Node):
    def __init__(self):
        super().__init__('puzzlebot_sim_node')
        self.r = 0.05
        self.l = 0.19
        
        # Declarar y leer parámetros de posición inicial
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = 0.0
        self.v = 0.0
        self.w = 0.0
        
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, 'pose_sim', 10)
        self.wr_pub = self.create_publisher(Float32, 'wr', 10)
        self.wl_pub = self.create_publisher(Float32, 'wl', 10)
        
        self.dt = 0.01
        self.create_timer(self.dt, self.update_kinematics)

    def cmd_vel_callback(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update_kinematics(self):
        self.x += self.v * math.cos(self.theta) * self.dt
        self.y += self.v * math.sin(self.theta) * self.dt
        self.theta += self.w * self.dt
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "world"
        pose_msg.pose.position.x = self.x
        pose_msg.pose.position.y = self.y
        self.pose_pub.publish(pose_msg)
        
        wr = (self.v + (self.w * self.l / 2.0)) / self.r
        wl = (self.v - (self.w * self.l / 2.0)) / self.r
        self.wr_pub.publish(Float32(data=wr))
        self.wl_pub.publish(Float32(data=wl))

def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotSim()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()