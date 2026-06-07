#!/usr/bin/env python3
"""
EKF FÍSICO - Capa de Fusión Sensorial (Opción 1: Cadena de Nodos)

Basado en código de referencia minichallenge6_new_test.

ARQUITECTURA:
  [localisation_node] → /odom (odometría cruda, dead-reckoning)
                        ↓
                    [ekf_node] ← /marker_publisher/markers (ArUco)
                        ↓
                    /odom_ekf (odometría fusionada)
                        ↓
                    [bug2_node, waypoint_manager]

RESPONSABILIDADES:
- Recibe /odom: predicción del EKF (ya integrada por localisation_node)
- Recibe /marker_publisher/markers: correcciones visuales
- Publica /odom_ekf: predicción + corrección ArUco
- Maneja TF (odom → base_footprint)

CAMBIO CRÍTICO: NO re-integra encoders. Solo consume odom como predicción.
"""

import math
import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy import qos
from rclpy.node import Node

from .aruco_detection_monitor import (
    CAMERA_TO_BASE_ROTATION_MATRIX,
    CAMERA_TO_BASE_TRANSLATION,
    KNOWN_MARKERS,
)


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(theta):
    return 0.0, 0.0, math.sin(theta / 2.0), math.cos(theta / 2.0)


def multiply_quaternions(first, second):
    x1, y1, z1, w1 = first
    x2, y2, z2, w2 = second
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def normalize_quaternion(quaternion):
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        return 0.0, 0.0, 0.0, 1.0
    return x / norm, y / norm, z / norm, w / norm


def quaternion_from_matrix(matrix):
    m00, m01, m02 = matrix[0]
    m10, m11, m12 = matrix[1]
    m20, m21, m22 = matrix[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (
            (m21 - m12) / s,
            (m02 - m20) / s,
            (m10 - m01) / s,
            0.25 * s,
        )

    if m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        return (
            0.25 * s,
            (m01 + m10) / s,
            (m02 + m20) / s,
            (m21 - m12) / s,
        )

    if m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        return (
            (m01 + m10) / s,
            0.25 * s,
            (m12 + m21) / s,
            (m02 - m20) / s,
        )

    s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
    return (
        (m02 + m20) / s,
        (m12 + m21) / s,
        0.25 * s,
        (m10 - m01) / s,
    )


class EKFNode(Node):
    def __init__(self):
        super().__init__('ekf_node')

        self.declare_parameter('aruco_detection_type', 'aruco_msgs')
        self.declare_parameter('aruco_pose_source_frame', 'camera')
        self.declare_parameter('camera_offset_x', CAMERA_TO_BASE_TRANSLATION[0])
        self.declare_parameter('camera_offset_y', CAMERA_TO_BASE_TRANSLATION[1])
        self.declare_parameter('camera_offset_z', CAMERA_TO_BASE_TRANSLATION[2])
        self.declare_parameter('max_marker_distance', 2.0)
        self.declare_parameter('max_aruco_innovation', 1.5)
        self.declare_parameter('max_aruco_raw_disagreement', 0.0)
        self.declare_parameter('aruco_measurement_std_x', 0.08)
        self.declare_parameter('aruco_measurement_std_y', 0.08)
        self.declare_parameter('process_noise_x', 0.003)
        self.declare_parameter('process_noise_y', 0.003)
        self.declare_parameter('process_noise_theta', 0.01)
        self.declare_parameter('initial_covariance_x', 0.05)
        self.declare_parameter('initial_covariance_y', 0.05)
        self.declare_parameter('initial_covariance_theta', 0.10)
        self.declare_parameter('diagnostic_period', 1.0)
        self.declare_parameter('max_prediction_dt', 0.25)
        self.declare_parameter('use_aruco_correction', True)

        self.aruco_detection_type = self.get_parameter('aruco_detection_type').value
        self.aruco_pose_source_frame = self.get_parameter('aruco_pose_source_frame').value.lower()
        self.camera_to_base_translation = (
            self.get_parameter('camera_offset_x').value,
            self.get_parameter('camera_offset_y').value,
            self.get_parameter('camera_offset_z').value,
        )
        self.camera_to_base_rotation = quaternion_from_matrix(CAMERA_TO_BASE_ROTATION_MATRIX)
        self.max_marker_distance = self.get_parameter('max_marker_distance').value
        self.max_aruco_innovation = self.get_parameter('max_aruco_innovation').value
        self.max_aruco_raw_disagreement = self.get_parameter('max_aruco_raw_disagreement').value
        self.diagnostic_period = self.get_parameter('diagnostic_period').value
        self.max_prediction_dt = self.get_parameter('max_prediction_dt').value
        self.use_aruco_correction = self.get_parameter('use_aruco_correction').value

        self.state = np.zeros(3)
        self.sigma = np.diag([
            self.get_parameter('initial_covariance_x').value,
            self.get_parameter('initial_covariance_y').value,
            self.get_parameter('initial_covariance_theta').value,
        ])
        self.q_base = np.diag([
            self.get_parameter('process_noise_x').value,
            self.get_parameter('process_noise_y').value,
            self.get_parameter('process_noise_theta').value,
        ])
        self.r_aruco = np.diag([
            self.get_parameter('aruco_measurement_std_x').value ** 2,
            self.get_parameter('aruco_measurement_std_y').value ** 2,
        ])

        self.initialized = False
        self.last_odom_stamp = None
        self.last_odom_msg = None
        self.last_raw_pose = None
        self.last_diagnostic_time = self.get_clock().now()
        self.last_aruco_status = 'sin_correccion'

        marker_msg_type = self.marker_message_type()
        # Se suscribe a /odom que publica localisation_node
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.marker_sub = self.create_subscription(
            marker_msg_type,
            '/marker_publisher/markers',
            self.markers_callback,
            qos.qos_profile_sensor_data,
        )
        self.odom_pub = self.create_publisher(Odometry, 'odom_ekf', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            'EKF Node inicializado: entrada /odom (localisation_node), salida /odom_ekf, '
            f'aruco_pose_source_frame={self.aruco_pose_source_frame}'
        )

    def marker_message_type(self):
        if self.aruco_detection_type == 'aruco_msgs':
            try:
                from aruco_msgs.msg import MarkerArray
                return MarkerArray
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_msgs. Verifica que el workspace esté sourceado.'
                ) from exc

        if self.aruco_detection_type == 'visualization_marker_array':
            from visualization_msgs.msg import MarkerArray
            return MarkerArray

        if self.aruco_detection_type == 'aruco_opencv':
            try:
                from aruco_opencv_msgs.msg import ArucoDetection
                return ArucoDetection
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_opencv_msgs. Usa aruco_detection_type:=aruco_msgs '
                    'si el tópico es /marker_publisher/markers.'
                ) from exc

        raise RuntimeError(
            'aruco_detection_type debe ser aruco_msgs, aruco_opencv o visualization_marker_array.'
        )

    def stamp_to_seconds(self, stamp):
        return stamp.sec + stamp.nanosec * 1.0e-9

    def odom_callback(self, msg):
        stamp_sec = self.stamp_to_seconds(msg.header.stamp)
        if stamp_sec == 0.0:
            stamp_sec = self.get_clock().now().nanoseconds * 1.0e-9

        raw_theta = yaw_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        self.last_raw_pose = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            raw_theta,
        )
        self.last_odom_msg = msg

        if not self.initialized:
            self.state[:] = self.last_raw_pose
            self.state[2] = normalize_angle(self.state[2])
            self.initialized = True
            self.last_odom_stamp = stamp_sec
            self.publish_odometry(msg.header.stamp)
            self.publish_diagnostics()
            return

        dt = stamp_sec - self.last_odom_stamp
        self.last_odom_stamp = stamp_sec
        if dt <= 0.0:
            self.publish_odometry(msg.header.stamp)
            self.publish_diagnostics()
            return

        dt = min(dt, self.max_prediction_dt)
        v = msg.twist.twist.linear.x
        w = msg.twist.twist.angular.z
        self.predict(v, w, dt)
        self.publish_odometry(msg.header.stamp)
        self.publish_diagnostics()

    def predict(self, v, w, dt):
        theta = self.state[2]
        self.state[0] += dt * v * math.cos(theta)
        self.state[1] += dt * v * math.sin(theta)
        self.state[2] = normalize_angle(self.state[2] + dt * w)

        f_jacobian = np.array([
            [1.0, 0.0, -dt * v * math.sin(theta)],
            [0.0, 1.0,  dt * v * math.cos(theta)],
            [0.0, 0.0,  1.0],
        ])
        self.sigma = f_jacobian @ self.sigma @ f_jacobian.T + self.q_base * max(dt, 1.0e-3)

    def markers_callback(self, msg):
        if not self.initialized or not self.use_aruco_correction:
            return

        candidates = self.known_marker_candidates(msg)
        if not candidates:
            self.last_aruco_status = 'sin_marcador_valido'
            return

        marker = min(candidates, key=lambda item: item['distance'])
        self.correct_with_marker(marker)

        if self.last_odom_msg is not None:
            self.publish_odometry(self.last_odom_msg.header.stamp)
        self.publish_diagnostics()

    def known_marker_candidates(self, msg):
        markers = self.extract_markers(msg)
        candidates = []

        for marker_id, pose in markers:
            if marker_id not in KNOWN_MARKERS or pose is None:
                continue

            marker_in_robot = self.marker_pose_in_robot(pose)
            rel_x, rel_y, _ = marker_in_robot['position']
            distance = math.sqrt(rel_x * rel_x + rel_y * rel_y)
            if distance > self.max_marker_distance:
                continue

            candidates.append({
                'id': marker_id,
                'pose': pose,
                'marker_in_robot': marker_in_robot,
                'distance': distance,
                'landmark': KNOWN_MARKERS[marker_id],
            })

        return candidates

    def extract_markers(self, msg):
        if self.aruco_detection_type == 'aruco_opencv':
            return [
                (int(marker.marker_id), marker.pose)
                for marker in msg.markers
            ]

        if self.aruco_detection_type == 'aruco_msgs':
            return [
                (int(marker.id), marker.pose.pose)
                for marker in msg.markers
            ]

        return [
            (int(marker.id), marker.pose)
            for marker in msg.markers
        ]

    def marker_pose_in_robot(self, pose):
        if self.aruco_pose_source_frame == 'base':
            orientation = pose.orientation
            return {
                'position': (pose.position.x, pose.position.y, pose.position.z),
                'orientation': normalize_quaternion((
                    orientation.x,
                    orientation.y,
                    orientation.z,
                    orientation.w,
                )),
            }

        return self.pose_camera_to_robot(pose)

    def pose_camera_to_robot(self, pose):
        position = pose.position
        orientation = pose.orientation
        tx, ty, tz = self.camera_to_base_translation

        robot_position = (
            position.z + tx,
            -position.x + ty,
            -position.y + tz,
        )
        robot_orientation = normalize_quaternion(
            multiply_quaternions(
                self.camera_to_base_rotation,
                (orientation.x, orientation.y, orientation.z, orientation.w),
            )
        )

        return {
            'position': robot_position,
            'orientation': robot_orientation,
        }

    def correct_with_marker(self, marker):
        landmark_x, landmark_y = marker['landmark']
        rel_x, rel_y, _ = marker['marker_in_robot']['position']
        theta = self.state[2]
        cos_theta = math.cos(theta)
        sin_theta = math.sin(theta)

        marker_dx_map = cos_theta * rel_x - sin_theta * rel_y
        marker_dy_map = sin_theta * rel_x + cos_theta * rel_y
        measured_robot = np.array([
            landmark_x - marker_dx_map,
            landmark_y - marker_dy_map,
        ])
        raw_delta_text = 'raw_delta=sin_odom'
        marker_error_text = 'marker_error=sin_odom'

        if self.last_raw_pose is not None:
            raw_x, raw_y, raw_theta = self.last_raw_pose
            raw_delta = measured_robot - np.array([raw_x, raw_y])
            raw_delta_norm = float(np.linalg.norm(raw_delta))

            raw_cos = math.cos(raw_theta)
            raw_sin = math.sin(raw_theta)
            marker_from_raw = np.array([
                raw_x + raw_cos * rel_x - raw_sin * rel_y,
                raw_y + raw_sin * rel_x + raw_cos * rel_y,
            ])
            marker_error = np.array([landmark_x, landmark_y]) - marker_from_raw
            raw_delta_text = (
                f'raw_delta=(dx={raw_delta[0]:.3f}, dy={raw_delta[1]:.3f}, '
                f'norm={raw_delta_norm:.3f})'
            )
            marker_error_text = (
                f'marker_desde_raw=(x={marker_from_raw[0]:.3f}, y={marker_from_raw[1]:.3f}), '
                f'marker_error=(dx={marker_error[0]:.3f}, dy={marker_error[1]:.3f})'
            )

            if (
                    self.max_aruco_raw_disagreement > 0.0 and
                    raw_delta_norm > self.max_aruco_raw_disagreement):
                self.last_aruco_status = (
                    f'id={marker["id"]}, rechazada_raw_disagreement, '
                    f'dist={marker["distance"]:.3f}, '
                    f'aruco_robot=(x={rel_x:.3f}, y={rel_y:.3f}), '
                    f'medicion=(x={measured_robot[0]:.3f}, y={measured_robot[1]:.3f}), '
                    f'{raw_delta_text}, {marker_error_text}'
                )
                return

        h = np.array([
            self.state[0],
            self.state[1],
        ])
        innovation = measured_robot - h
        innovation_norm = float(np.linalg.norm(innovation))
        if innovation_norm > self.max_aruco_innovation:
            self.last_aruco_status = (
                f'id={marker["id"]}, rechazada_innovacion='
                f'(x={innovation[0]:.3f}, y={innovation[1]:.3f})'
            )
            return

        h_jacobian = np.array([
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
        ])
        s_matrix = h_jacobian @ self.sigma @ h_jacobian.T + self.r_aruco
        kalman_gain = self.sigma @ h_jacobian.T @ np.linalg.inv(s_matrix)
        kalman_gain[2, :] = 0.0

        correction = kalman_gain @ innovation
        self.state += correction
        self.state[2] = normalize_angle(self.state[2])

        identity = np.eye(3)
        joseph = identity - kalman_gain @ h_jacobian
        self.sigma = joseph @ self.sigma @ joseph.T + kalman_gain @ self.r_aruco @ kalman_gain.T

        self.last_aruco_status = (
            f'id={marker["id"]}, dist={marker["distance"]:.3f}, '
            f'aruco_robot=(x={rel_x:.3f}, y={rel_y:.3f}), '
            f'medicion=(x={measured_robot[0]:.3f}, y={measured_robot[1]:.3f}), '
            f'{raw_delta_text}, {marker_error_text}, '
            f'innovacion=(x={innovation[0]:.3f}, y={innovation[1]:.3f}), '
            f'correccion=(dx={correction[0]:.3f}, dy={correction[1]:.3f})'
        )

    def publish_odometry(self, stamp):
        if self.last_odom_msg is None:
            return

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.last_odom_msg.header.frame_id or 'odom'
        odom_msg.child_frame_id = self.last_odom_msg.child_frame_id or 'base_footprint'

        odom_msg.pose.pose.position.x = float(self.state[0])
        odom_msg.pose.pose.position.y = float(self.state[1])
        odom_msg.pose.pose.position.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(self.state[2])
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = odom_msg.header.frame_id
        transform.child_frame_id = odom_msg.child_frame_id
        transform.transform.translation.x = float(self.state[0])
        transform.transform.translation.y = float(self.state[1])
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(transform)

        odom_msg.twist = self.last_odom_msg.twist
        odom_msg.pose.covariance = self.pose_covariance_36()
        self.odom_pub.publish(odom_msg)

    def pose_covariance_36(self):
        covariance = [0.0] * 36
        covariance[0] = float(self.sigma[0, 0])
        covariance[1] = float(self.sigma[0, 1])
        covariance[5] = float(self.sigma[0, 2])
        covariance[6] = float(self.sigma[1, 0])
        covariance[7] = float(self.sigma[1, 1])
        covariance[11] = float(self.sigma[1, 2])
        covariance[30] = float(self.sigma[2, 0])
        covariance[31] = float(self.sigma[2, 1])
        covariance[35] = float(self.sigma[2, 2])
        return covariance

    def publish_diagnostics(self):
        now = self.get_clock().now()
        if (now - self.last_diagnostic_time).nanoseconds < self.diagnostic_period * 1.0e9:
            return

        self.last_diagnostic_time = now
        if self.last_raw_pose is None:
            raw_text = 'raw=sin_odom'
        else:
            raw_x, raw_y, raw_theta = self.last_raw_pose
            raw_text = f'raw=(x={raw_x:.3f}, y={raw_y:.3f}, theta={raw_theta:.3f})'

        self.get_logger().info(
            f'{raw_text}, '
            f'ekf=(x={self.state[0]:.3f}, y={self.state[1]:.3f}, theta={self.state[2]:.3f}), '
            f'aruco={self.last_aruco_status}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EKFNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
