import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'puzzlebot_sim'

    use_localisation = LaunchConfiguration('use_localisation')
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    odom_topic = LaunchConfiguration('odom_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    wr_topic = LaunchConfiguration('wr_topic')
    wl_topic = LaunchConfiguration('wl_topic')
    require_scan = LaunchConfiguration('require_scan')
    require_odom = LaunchConfiguration('require_odom')
    front_stop_distance = LaunchConfiguration('front_stop_distance')
    avoidance_start_distance = LaunchConfiguration('avoidance_start_distance')
    wall_follow_start_distance = LaunchConfiguration('wall_follow_start_distance')
    goal_tolerance = LaunchConfiguration('goal_tolerance')
    wall_follow_goal_tolerance = LaunchConfiguration('wall_follow_goal_tolerance')
    goal_pass_margin = LaunchConfiguration('goal_pass_margin')
    goal_pass_lateral_tolerance = LaunchConfiguration('goal_pass_lateral_tolerance')
    near_goal_slow_distance = LaunchConfiguration('near_goal_slow_distance')
    near_goal_v_max = LaunchConfiguration('near_goal_v_max')
    scan_front_angle = LaunchConfiguration('scan_front_angle')
    use_aruco_tracker = LaunchConfiguration('use_aruco_tracker')
    use_aruco_monitor = LaunchConfiguration('use_aruco_monitor')
    aruco_cam_base_topic = LaunchConfiguration('aruco_cam_base_topic')
    aruco_marker_size = LaunchConfiguration('aruco_marker_size')
    aruco_detection_topic = LaunchConfiguration('aruco_detection_topic')
    aruco_detection_type = LaunchConfiguration('aruco_detection_type')

    common_parameters = [{'use_sim_time': False}]
    
    # --- NUEVO: CARGAR EL URDF DEL ROBOT ---
    pkg_share = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'FinalChallenge.rviz')
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_file, 'r') as infp:
        robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_desc,
            'use_sim_time': False
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': False}],
    )
    # ---------------------------------------

    localisation = Node(
        package=package_name,
        executable='ekf_physical_node',
        name='ekf_physical_node',
        output='screen',
        parameters=common_parameters,
        condition=IfCondition(use_localisation),
        remappings=[
            ('odom', odom_topic),
            ('VelocityEncR', wr_topic),
            ('VelocityEncL', wl_topic),
        ],
    )

    raw_odom_node = Node(
        package=package_name,
        executable='localisation_node',
        name='raw_localisation_node',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    bug2_node = Node(
        package=package_name,
        executable='bug2_FC_node',
        name='bug2_FC_node',
        output='screen',
        parameters=[
            {'use_sim_time': False},
            {'require_scan': ParameterValue(require_scan, value_type=bool)},
            {'require_odom': ParameterValue(require_odom, value_type=bool)},
            {'front_stop_distance': ParameterValue(front_stop_distance, value_type=float)},
            {'avoidance_start_distance': ParameterValue(avoidance_start_distance, value_type=float)},
            {'wall_follow_start_distance': ParameterValue(wall_follow_start_distance, value_type=float)},
            {'goal_tolerance': ParameterValue(goal_tolerance, value_type=float)},
            {'wall_follow_goal_tolerance': ParameterValue(wall_follow_goal_tolerance, value_type=float)},
            {'goal_pass_margin': ParameterValue(goal_pass_margin, value_type=float)},
            {'goal_pass_lateral_tolerance': ParameterValue(goal_pass_lateral_tolerance, value_type=float)},
            {'near_goal_slow_distance': ParameterValue(near_goal_slow_distance, value_type=float)},
            {'near_goal_v_max': ParameterValue(near_goal_v_max, value_type=float)},
            {'scan_front_angle': ParameterValue(scan_front_angle, value_type=float)},
        ],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('odom', odom_topic),
            ('scan', scan_topic),
            ('goal', goal_topic),
        ],
    )

    aruco_tracker = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_tracker_autostart',
        output='screen',
        condition=IfCondition(use_aruco_tracker),
        parameters=[
            {'cam_base_topic': aruco_cam_base_topic},
            {'marker_size': ParameterValue(aruco_marker_size, value_type=float)},
        ],
    )

    aruco_monitor = Node(
        package=package_name,
        executable='arucostatus',
        name='arucostatus',
        output='screen',
        condition=IfCondition(use_aruco_monitor),
        parameters=[
            {'detection_topic': '/aruco_detections'},
            {'detection_type': 'aruco_opencv'},
        ],
    )

    waypoint_node = Node(
        package=package_name,
        executable='waypoint_manager',
        name='waypoint_manager',
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}]
    )

    laser_tf_fix = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='laser_tf_fix',
    arguments=['--x', '0', '--y', '0', '--z', '0',
               '--yaw', '0', '--pitch', '0', '--roll', '0',
               '--frame-id', 'laser_link', '--child-frame-id', 'laser'],
)

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        DeclareLaunchArgument('use_localisation', default_value='true', description='Genera odom desde encoders con EKF.'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', description='Topico de velocidad.'),
        DeclareLaunchArgument('odom_topic', default_value='odom', description='Topico de odometria.'),
        DeclareLaunchArgument('scan_topic', default_value='scan', description='Topico del RPLidar.'),
        DeclareLaunchArgument('goal_topic', default_value='goal', description='Topico de meta Pose2D.'),
        DeclareLaunchArgument('wr_topic', default_value='VelocityEncR', description='Encoder derecho.'),
        DeclareLaunchArgument('wl_topic', default_value='VelocityEncL', description='Encoder izquierdo.'),
        DeclareLaunchArgument('require_scan', default_value='true', description='Se detiene si no hay RPLidar.'),
        DeclareLaunchArgument('require_odom', default_value='true', description='Se detiene si no hay odom.'),
        DeclareLaunchArgument('front_stop_distance', default_value='0.22', description='Detener avance y seguir pared.'),
        DeclareLaunchArgument('avoidance_start_distance', default_value='0.38', description='Empezar a esquivar suavemente.'),
        DeclareLaunchArgument('wall_follow_start_distance', default_value='0.28', description='Cambiar a WALL_FOLLOWING.'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.05', description='Radio para alcanzar meta.'),
        DeclareLaunchArgument('wall_follow_goal_tolerance', default_value='0.18', description='Radio de captura de meta siguiendo pared.'),
        DeclareLaunchArgument('goal_pass_margin', default_value='0.02', description='Margen para detenerse al pasar meta.'),
        DeclareLaunchArgument('goal_pass_lateral_tolerance', default_value='0.22', description='Tolerancia lateral maxima.'),
        DeclareLaunchArgument('near_goal_slow_distance', default_value='0.35', description='Distancia para reducir velocidad.'),
        DeclareLaunchArgument('near_goal_v_max', default_value='0.025', description='Velocidad maxima cerca de meta.'),
        DeclareLaunchArgument('scan_front_angle', default_value='0.0', description='Frente del LaserScan.'),
        DeclareLaunchArgument('use_aruco_tracker', default_value='true', description='Arranca aruco_opencv.'),
        DeclareLaunchArgument('use_aruco_monitor', default_value='true', description='Arranca monitor ArUco.'),
        DeclareLaunchArgument('aruco_cam_base_topic', default_value='/image_raw', description='Topico base de imagen.'),
        DeclareLaunchArgument('aruco_marker_size', default_value='0.06', description='Tamano del ArUco (6cm).'),
        DeclareLaunchArgument('aruco_detection_topic', default_value='/marker_publisher/markers', description='Topico de detecciones.'),
        DeclareLaunchArgument('aruco_detection_type', default_value='visualization_marker_array', description='Tipo de deteccion.'),
        
        robot_state_publisher_node,
        joint_state_publisher_node,
        localisation,
        raw_odom_node,
        aruco_tracker,
        aruco_monitor,
        bug2_node,
        waypoint_node,
        rviz_node,
        laser_tf_fix,
    ])