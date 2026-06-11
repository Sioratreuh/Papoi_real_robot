#!/usr/bin/env python3
# ROS2 localization node for Puzzlebot using encoder integration
import rclpy
import math
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


def quaternion_from_euler(ai, aj, ak):
    # Convert Euler angles (roll, pitch, yaw) to quaternion
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
    q = np.empty((4,))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss
    return q

class PuzzlebotLocalization(Node):
    """Differential-drive robot localization via encoder integration"""
    def __init__(self):
        super().__init__('puzzlebot_localization')

        # Robot physical parameters
        self.r = 0.045    # Wheel radius (m)
        self.l = 0.19   # Wheel separation (m)

        # Current pose
        # Agregar al __init__, antes del timer:
        self.declare_parameter('odom_offset_x', 0.25)  # Offset inicial X para odometría (ajustar según posición física del robot)
        self.declare_parameter('odom_offset_y', -0.295) # Offset inicial Y para odometría (ajustar según posición física del robot)
        self.declare_parameter('odom_offset_theta', 0.0)
        self.x = self.get_parameter('odom_offset_x').value
        self.y = self.get_parameter('odom_offset_y').value
        self.theta = self.get_parameter('odom_offset_theta').value
        # Wheel angular velocities (rad/s)
        self.wr = 0.0
        self.wl = 0.0

        # Subscribe to encoder velocities
        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        self.create_subscription(Float32, 'VelocityEncR', self.wr_callback, qos_sensor)
        self.create_subscription(Float32, 'VelocityEncL', self.wl_callback, qos_sensor)

        # Publish odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # Update at 100 Hz
        self.dt = 0.01
        self.create_timer(self.dt, self.update_odometry)

        self.get_logger().info('Puzzlebot Localization node started')
        self.declare_parameter('publish_without_fresh_encoders', True)
        self.publish_without_fresh = self.get_parameter('publish_without_fresh_encoders').value

    def wr_callback(self, msg):
        # Right wheel velocity callback
        self.wr = msg.data

    def wl_callback(self, msg):
        # Left wheel velocity callback
        self.wl = msg.data

    def update_odometry(self):
        if not self.publish_without_fresh:
            if self.wr == 0.0 and self.wl == 0.0:
                return
        # Compute linear and angular velocity from wheel velocities
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.l

        # Integrate pose using forward kinematics
        self.x     += v * math.cos(self.theta) * self.dt
        self.y     += v * math.sin(self.theta) * self.dt
        self.theta += w * self.dt

        current_time = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0.0, 0.0, self.theta)

        # Publish odometry message
        odom = Odometry()
        odom.header.stamp    = current_time
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_footprint'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x  = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

def main(args=None):
    # Initialize and run ROS2 node
    rclpy.init(args=args)
    node = PuzzlebotLocalization()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()