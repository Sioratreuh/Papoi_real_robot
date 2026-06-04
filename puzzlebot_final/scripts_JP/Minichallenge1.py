import rclpy, math, tf_transformations
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster


class DynamicTFClass(Node):
    def __init__(self):
        super().__init__('dynamic_frame_publisher')
        self.get_logger().info('Dynamic TF broadcaster node started')

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t0 = self.get_clock().now()
        self.P = 5.0
        self.A = 1.0
        self.tf_msg = TransformStamped()
        self.p = StaticTransformBroadcaster(self)
        

        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def timer_callback(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1e9
        self.tf_msg.header.stamp = self.get_clock().now().to_msg()
        self.tf_msg.header.frame_id = 'map'
        self.tf_msg.child_frame_id = 'base_footprint'

        w = 2 * math.pi / self.P

        self.tf_msg.transform.translation.x = self.A * math.cos(w * t)
        self.tf_msg.transform.translation.y = self.A * math.sin(w * t)
        self.tf_msg.transform.translation.z = 0.0

        q = tf_transformations.quaternion_from_euler(0.0, 0.0, math.pi/2 + w * t)
        self.tf_msg.transform.rotation.x = q[0]
        self.tf_msg.transform.rotation.y = q[1]
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]

        self.p.sendTransform(self.tf_msg)

        joint_msg = JointState()
        joint_msg.header.stamp = self.get_clock().now().to_msg()
        joint_msg.name = ['wheel_l_joint', 'wheel_r_joint']
        joint_msg.position = [w * t, w * t]
        joint_msg.velocity = [w, w]

        self.joint_pub.publish(joint_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DynamicTFClass()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\nNode terminated by user')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
