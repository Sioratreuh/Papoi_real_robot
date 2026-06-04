#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0; aj /= 2.0; ak /= 2.0
    ci = math.cos(ai); si = math.sin(ai)
    cj = math.cos(aj); sj = math.sin(aj)
    ck = math.cos(ak); sk = math.sin(ak)
    cc = ci*ck; cs = ci*sk; sc = si*ck; ss = si*sk
    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs; q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc; q[3] = cj*cc + sj*ss
    return q

class LocalisationNode(Node):
    def __init__(self):
        super().__init__('localisation_node')
        
        self.declare_parameter('x0', 0.0)
        self.declare_parameter('y0', 0.0)
        
        self.r = 0.05
        self.l = 0.19
        self.x = float(self.get_parameter('x0').value)
        self.y = float(self.get_parameter('y0').value)
        self.theta = 0.0
        self.wr = 0.0
        self.wl = 0.0

        # MINICHALLENGE 4: Inicialización de Covarianza
        self.P = np.zeros((3, 3)) 
        self.var_x = 0.0005  
        self.var_y = 0.0001  
        self.C = 0.010      
        self.B = 0.0         

        # Nos suscribimos directo a Gazebo para obtener velocidades reales
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # PUBLICAMOS EN UN CANAL SEPARADO PARA NO PELEAR CON GAZEBO
        self.odom_pub = self.create_publisher(Odometry, '/odom_est', 10)
        
        self.dt = 0.01
        self.create_timer(self.dt, self.update_odometry)

    def joint_callback(self, msg):
        try:
            # Extraemos la velocidad (rad/s) de cada llanta
            idx_l = msg.name.index('wheel_l_joint')
            idx_r = msg.name.index('wheel_r_joint')
            self.wl = msg.velocity[idx_l]
            self.wr = msg.velocity[idx_r]
        except (ValueError, IndexError):
            pass

    def update_covariance(self, v, dt):
        J_h = np.array([
            [1.0, 0.0, -v * dt * math.sin(self.theta)],
            [0.0, 1.0,  v * dt * math.cos(self.theta)],
            [0.0, 0.0,  1.0]
        ])

        Q_base = np.array([
            [self.var_x, self.B, self.B],
            [self.B, self.var_y, self.B],
            [self.B, self.B, self.C]
        ])

        distancia_paso = abs(v) * dt
        Q = Q_base * distancia_paso

        self.P = J_h @ self.P @ J_h.T + Q

    def update_odometry(self):
        v = self.r * (self.wr + self.wl) / 2.0
        w = self.r * (self.wr - self.wl) / self.l
        
        self.update_covariance(v, self.dt)
        
        self.x += v * math.cos(self.theta) * self.dt
        self.y += v * math.sin(self.theta) * self.dt
        self.theta += w * self.dt
        
        current_time = self.get_clock().now().to_msg()
        q = quaternion_from_euler(0.0, 0.0, self.theta)
        
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # Llenado de matriz 6x6
        odom.pose.covariance = [0.0] * 36
        odom.pose.covariance[0]  = self.P[0, 0]  
        odom.pose.covariance[7]  = self.P[1, 1]  
        odom.pose.covariance[35] = self.P[2, 2]  
        odom.pose.covariance[1]  = self.P[0, 1]  
        odom.pose.covariance[6]  = self.P[1, 0]  
        odom.pose.covariance[5]  = self.P[0, 2]  
        odom.pose.covariance[30] = self.P[2, 0]  
        odom.pose.covariance[11] = self.P[1, 2]  
        odom.pose.covariance[31] = self.P[2, 1]  

        self.odom_pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = LocalisationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()