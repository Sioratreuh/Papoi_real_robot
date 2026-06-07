#!/usr/bin/env python3
# Publica /joint_states (animacion de ruedas en RViz) a partir de la odometria.
# NO difunde TF: el dueno de odom->base_footprint es el EKF.
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState


class PuzzlebotJointStatePublisher(Node):
    """Publish joint states from odometry messages"""
    def __init__(self):
        super().__init__('joint_state_publisher')

        # Parametros fisicos (solo afectan la velocidad visual de giro de la rueda)
        self.r = 0.045   # Wheel radius (m)
        self.l = 0.17   # Wheel separation (m)

        self.angle_r = 0.0
        self.angle_l = 0.0
        self.v = 0.0
        self.w = 0.0

        # Relativo (no '/odom'): asi el launch puede remapearlo
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        self.dt = 0.01
        self.create_timer(self.dt, self.publish)
        self.get_logger().info('Joint State Publisher node started')

    def odom_callback(self, msg):
        # Solo extrae velocidades; el TF lo emite el EKF.
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

    def publish(self):
        wr = (self.v + self.w * self.l / 2.0) / self.r
        wl = (self.v - self.w * self.l / 2.0) / self.r

        self.angle_r += wr * self.dt
        self.angle_l += wl * self.dt

        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = ['wheel_l_joint', 'wheel_r_joint']
        js.position = [self.angle_l, self.angle_r]
        self.joint_pub.publish(js)


def main(args=None):
    rclpy.init(args=args)
    node = PuzzlebotJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()