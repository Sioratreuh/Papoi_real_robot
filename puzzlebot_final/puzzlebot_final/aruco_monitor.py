#!/usr/bin/env python3
import math
import rclpy
from rclpy import qos
from rclpy.node import Node


KNOWN_MARKERS = {
    70:  (1.84, -0.295),
    705: (0.93, -1.23),
    706: (2.42, -1.27),
    708: (1.19, -1.25),
    703: (1.21, -2.09),
    702: (0.0, -1.82),
    75:  (2.72, -2.40),
    701: (2.77,  0.0),
    710: (1.86, -0.28),
    711: (3.02, -2.70),
    712: (0.00, -0.92),
    713: (0.37, -3.25),
    714: (0.00, -0.26),
}


class ArucoDetectionMonitor(Node):
    def __init__(self):
        super().__init__('aruco_detection_monitor')

        self.declare_parameter('detection_topic', '/aruco_detections')
        self.declare_parameter('detection_type', 'aruco_opencv')
        self.declare_parameter('diagnostic_period', 1.0)
        self.declare_parameter('known_marker_ids', [])

        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_type = self.get_parameter('detection_type').value
        self.diagnostic_period = self.get_parameter('diagnostic_period').value
        self.known_marker_ids = set(self.get_parameter('known_marker_ids').value)

        self.last_detections = []
        self.last_detection_time = None
        self.received_message_count = 0
        self.last_raw_marker_count = 0
        msg_type = self.message_type()
        self.create_subscription(msg_type, self.detection_topic, self.detection_callback, 10)
        self.create_subscription(
            msg_type,
            self.detection_topic,
            self.detection_callback,
            qos.qos_profile_sensor_data,
        )
        self.create_timer(self.diagnostic_period, self.publish_diagnostics)

        self.get_logger().info(
            f'Monitor ArUco escuchando {self.detection_topic} como {self.detection_type}. '
            'Solo reporta detecciones; no corrige odometria.'
        )

    def message_type(self):
        if self.detection_type == 'aruco_opencv':
            try:
                from aruco_opencv_msgs.msg import ArucoDetection
                return ArucoDetection
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_opencv_msgs. Sourcea el workspace/instalacion '
                    'donde existe aruco_opencv, o usa detection_type:=visualization_marker_array.'
                ) from exc

        if self.detection_type == 'visualization_marker_array':
            from visualization_msgs.msg import MarkerArray
            return MarkerArray

        if self.detection_type == 'aruco_msgs':
            try:
                from aruco_msgs.msg import MarkerArray
                return MarkerArray
            except ImportError as exc:
                raise RuntimeError(
                    'No pude importar aruco_msgs. Verifica el tipo real con: '
                    'ros2 topic type /marker_publisher/markers'
                ) from exc

        if self.detection_type == 'markers_list':
            from std_msgs.msg import Int32MultiArray
            return Int32MultiArray

        if self.detection_type == 'markers_list_u32':
            from std_msgs.msg import UInt32MultiArray
            return UInt32MultiArray

        raise RuntimeError(
            'detection_type debe ser aruco_opencv, aruco_msgs, visualization_marker_array, '
            'markers_list o markers_list_u32.'
        )

    def detection_callback(self, msg):
        self.last_detection_time = self.get_clock().now()
        self.received_message_count += 1

        if self.detection_type == 'aruco_opencv':
            self.last_raw_marker_count = len(msg.markers)
            self.last_detections = [
                self.marker_record(marker.marker_id, marker.pose)
                for marker in msg.markers
            ]
            return

        if self.detection_type == 'aruco_msgs':
            self.last_raw_marker_count = len(msg.markers)
            self.last_detections = [
                self.marker_record(marker.id, marker.pose.pose)
                for marker in msg.markers
            ]
            return

        if self.detection_type in ('markers_list', 'markers_list_u32'):
            self.last_raw_marker_count = len(msg.data)
            self.last_detections = [
                self.marker_record(marker_id, None)
                for marker_id in msg.data
            ]
            return

        self.last_raw_marker_count = len(msg.markers)
        self.last_detections = [
            self.marker_record(marker.id, marker.pose)
            for marker in msg.markers
        ]

    def marker_record(self, marker_id, pose):
        distance = None
        if pose is not None:
            distance = math.sqrt(
                pose.position.x ** 2 +
                pose.position.y ** 2 +
                pose.position.z ** 2
            )

        return {
            'id': int(marker_id),
            'pose': pose,
            'distance': distance,
            'known': not self.known_marker_ids or int(marker_id) in self.known_marker_ids,
            'map_position': KNOWN_MARKERS.get(int(marker_id)),
        }

    def publish_diagnostics(self):
        if self.last_detection_time is None:
            self.get_logger().info(
                f'Aun no recibo mensajes en {self.detection_topic}. '
                'El topico existe, pero no ha llegado ningun callback al monitor.'
            )
            return

        if self.last_raw_marker_count == 0:
            self.get_logger().info(
                f'Recibo mensajes en {self.detection_topic}, pero vienen sin markers. '
                f'mensajes={self.received_message_count}'
            )
            return

        detections = [item for item in self.last_detections if item['known']]
        if not detections:
            self.get_logger().info('Recibi detecciones, pero ninguna coincide con known_marker_ids.')
            return

        if all(item['distance'] is None for item in detections):
            all_markers_text = '; '.join(
                self.format_marker(item, selected=False)
                for item in detections
            )
            self.get_logger().info(
                f'arucos_detectados_ids_y_coordenadas: {all_markers_text}'
            )
            return

        closest = min(
            detections,
            key=lambda item: item['distance'] if item['distance'] is not None else float('inf')
        )
        all_markers_text = '; '.join(
            self.format_marker(item, selected=(item is closest))
            for item in detections
        )

        self.get_logger().info(
            f'aruco_detectado_mas_cercano: {self.format_marker(closest, selected=True)}'
        )
        self.get_logger().info(
            f'arucos_detectados: {all_markers_text}'
        )

    def format_marker(self, marker, selected=False):
        pose = marker['pose']
        map_text = 'map=(desconocido)'
        if marker['map_position'] is not None:
            marker_x, marker_y = marker['map_position']
            map_text = f'map=({marker_x:.2f}, {marker_y:.2f})'

        selected_text = ' seleccionado' if selected else ''
        if pose is None:
            return f'id={marker["id"]}{selected_text}, {map_text}'

        distance_text = 'dist=sin_pose'
        if marker['distance'] is not None:
            distance_text = f'dist={marker["distance"]:.2f}m'

        return (
            f'id={marker["id"]}{selected_text}, {map_text}, '
            f'{distance_text}, '
            f'cam=(x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetectionMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()