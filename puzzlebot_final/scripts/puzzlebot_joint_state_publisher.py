#!/usr/bin/env python3
# ROS2 node to publish joint states from odometry
import rclpy
import math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class PuzzlebotJointStatePublisher(Node):
    """Publish joint states and TF from odometry messages"""
    def __init__(self):
        super().__init__('puzzlebot_joint_state_publisher')

        # Robot physical parameters (must match puzzlebot_localization.py)
        self.r = 0.05    # Wheel radius (m)
        self.l = 0.19    # Wheel separation (m)

        # Accumulated wheel angles
        self.angle_r = 0.0
        self.angle_l = 0.0

        # Current velocities
        self.v = 0.0
        self.w = 0.0

        # Subscribe to odometry
        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Publishers
        self.joint_pub      = self.create_publisher(JointState, '/joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Update at 100 Hz
        self.dt = 0.01
        self.create_timer(self.dt, self.publish)

        self.get_logger().info('Puzzlebot Joint State Publisher node started')

    def odom_callback(self, msg):
        # Extract velocities from odometry message
        self.v = msg.twist.twist.linear.x
        self.w = msg.twist.twist.angular.z

        # Broadcast odom -> base_footprint transform
        t = TransformStamped()
        t.header.stamp    = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id  = 'base_footprint'
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)

    def publish(self):
        # Compute wheel angular velocities from robot velocities
        wr = (self.v + self.w * self.l / 2.0) / self.r
        wl = (self.v - self.w * self.l / 2.0) / self.r

        # Integrate wheel angles
        self.angle_r += wr * self.dt
        self.angle_l += wl * self.dt

        # Publish joint states
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name         = ['wheel_l_joint', 'wheel_r_joint']
        js.position     = [self.angle_l, self.angle_r]
        self.joint_pub.publish(js)

def main(args=None):
    # Initialize and run ROS2 node
    rclpy.init(args=args)
    node = PuzzlebotJointStatePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()