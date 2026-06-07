"""
Final Challenge Launch - Puzzlebot Navigation Stack (Opción 1: Cadena de Nodos)

ARQUITECTURA:
/VelocityEncR/L → localisation_node (/odom crudo)
                 → ekf_node (corrección ArUco)
                 → /odom_ekf (ÚNICA FUENTE DE VERDAD)
                 → bug2_node + waypoint_manager

NOTA: robot_state_publisher va en puzzlebot_aruco.launch.xml (bringup), no aquí.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    package_name = 'puzzlebot_final'  # ✅ Correcto: paquete real
    
    # ==================== CONFIGURACIONES ====================
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    wr_topic = LaunchConfiguration('wr_topic')
    wl_topic = LaunchConfiguration('wl_topic')
    scan_topic = LaunchConfiguration('scan_topic')
    goal_topic = LaunchConfiguration('goal_topic')
    
    # Parámetros Bug2
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
    v_max = LaunchConfiguration('v_max')        
    w_max = LaunchConfiguration('w_max')
    bug_type = LaunchConfiguration('bug_type')
    wall_follow_side = LaunchConfiguration('wall_follow_side')
    odom_offset_x     = LaunchConfiguration('odom_offset_x')       # ← AGREGAR
    odom_offset_y     = LaunchConfiguration('odom_offset_y')       # ← AGREGAR
    odom_offset_theta = LaunchConfiguration('odom_offset_theta')  
    
    # ArUco
    use_aruco_monitor = LaunchConfiguration('use_aruco_monitor')
    
    # ==================== NODOS CAPA B - NAVEGACIÓN ====================
    
    # NODO 1: Localización cruda (dead-reckoning puro)
    localisation_node = Node(
        package=package_name,
        executable='localisation_node',  # ✅ entry_point correcto
        name='localisation_node',
        output='screen',
        parameters=[
    {'use_sim_time': False},
    {'odom_offset_x': ParameterValue(odom_offset_x, value_type=float)},
    {'odom_offset_y': ParameterValue(odom_offset_y, value_type=float)},
    {'odom_offset_theta': ParameterValue(odom_offset_theta, value_type=float)},
],
        remappings=[
            ('VelocityEncR', wr_topic),
            ('VelocityEncL', wl_topic),
        ],
    )
    
    # NODO 2: EKF (corrección con ArUco)
    ekf_node = Node(
        package=package_name,
        executable='ekf_node',  # ✅ entry_point correcto
        name='ekf_node',
        output='screen',
        parameters=[{'use_sim_time': False}],
        # NO remappings necesarios: suscribe a /odom (de localisation_node), publica /odom_ekf
    )
    
    # NODO 3: Generador de metas
    waypoint_node = Node(
        package=package_name,
        executable='waypoint_manager',  # ✅ entry_point correcto
        name='waypoint_manager',
        output='screen',
        parameters=[{'use_sim_time': False}],
        remappings=[
            ('odom', 'odom_ekf'),  # ← lee odom fusionado
            ('goal', goal_topic),
        ],
    )
    
    # NODO 4: Control principal Bug

    bug0_node = Node(
        package=package_name,
        executable='bug0_node',
        name='bug0_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", bug_type, "' == 'bug0'"])),
        parameters=[
            {'use_sim_time': False},
            {'front_stop_distance':      ParameterValue(front_stop_distance,      value_type=float)},
            {'avoidance_start_distance': ParameterValue(avoidance_start_distance, value_type=float)},
            {'goal_tolerance':           ParameterValue(goal_tolerance,           value_type=float)},
            {'near_goal_slow_distance':  ParameterValue(near_goal_slow_distance,  value_type=float)},
            {'near_goal_v_max':          ParameterValue(near_goal_v_max,          value_type=float)},
            {'scan_front_angle':         ParameterValue(scan_front_angle,         value_type=float)},
            {'v_max':                    ParameterValue(v_max,                    value_type=float)},
            {'w_max':                    ParameterValue(w_max,                    value_type=float)},
            {'wall_follow_side':         ParameterValue(wall_follow_side,         value_type=str)},
            {'sensor_timeout':           ParameterValue(sensor_timeout,           value_type=float)},
            {'require_scan':             ParameterValue(require_scan,             value_type=bool)},
            {'require_odom':             ParameterValue(require_odom,             value_type=bool)},
        ],
        remappings=[
            ('odom',  'odom_ekf'),
            ('scan',  scan_topic),
            ('goal',  goal_topic),
            ('cmd_vel', cmd_vel_topic),
        ],
    )

    bug2_node = Node(
        package=package_name,
        executable='bug2_node',  # ✅ entry_point correcto
        name='bug2_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", bug_type, "' == 'bug2'"])),
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
            {'v_max': ParameterValue(v_max, value_type=float)},
            {'w_max': ParameterValue(w_max, value_type=float)},
            {'wall_follow_side': ParameterValue(wall_follow_side, value_type=str)},
        ],
        remappings=[
            ('cmd_vel', cmd_vel_topic),
            ('odom', 'odom_ekf'),  # ← lee odom fusionado
            ('scan', scan_topic),
            ('goal', goal_topic),
        ],
    )
    
    # NODO 5: Monitor ArUco (diagnóstico, opcional)
    aruco_monitor = Node(
        package=package_name,
        executable='aruco_monitor',  # ✅ entry_point correcto
        name='aruco_monitor',
        output='screen',
        parameters=[
            {'detection_topic': '/marker_publisher/markers'},
            {'detection_type': 'aruco_msgs'},
        ],
    )
    
    # NODO 6: RViz (opcional, visualización)
    pkg_share = get_package_share_directory(package_name)
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'FinalChallenge.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # ===== FALLBACK: descomentar SOLO si el bringup no levanta RSP/JSP =====
    desc_share = get_package_share_directory('puzzlebot_description')
    urdf_xacro = os.path.join(desc_share, 'urdf', 'puzzlebot.urdf.xacro')
    robot_description = ParameterValue(Command(['xacro ', urdf_xacro]), value_type=str)
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': False}],
    )
    
    joint_state_publisher_node = Node(
        package='puzzlebot_final',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': False}],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '0'),
        # ==================== ARGUMENTOS DEL LAUNCH ====================
        DeclareLaunchArgument('cmd_vel_topic', default_value='cmd_vel', 
                            description='Tópico de velocidad.'),
        DeclareLaunchArgument('wr_topic', default_value='VelocityEncR', 
                            description='Encoder derecho.'),
        DeclareLaunchArgument('wl_topic', default_value='VelocityEncL', 
                            description='Encoder izquierdo.'),
        DeclareLaunchArgument('scan_topic', default_value='scan', 
                            description='Tópico del RPLidar.'),
        DeclareLaunchArgument('goal_topic', default_value='goal', 
                            description='Tópico de meta Pose2D.'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                            description='Lanzar RViz2'),
        
        # Bug2 parámetros
        DeclareLaunchArgument('require_scan', default_value='true', 
                            description='Se detiene si no hay RPLidar.'),
        DeclareLaunchArgument('require_odom', default_value='true', 
                            description='Se detiene si no hay odom.'),
        DeclareLaunchArgument('front_stop_distance', default_value='0.25', 
                            description='Detener avance y seguir pared.'),
        DeclareLaunchArgument('avoidance_start_distance', default_value='0.38', 
                            description='Empezar a esquivar suavemente.'),
        DeclareLaunchArgument('wall_follow_start_distance', default_value='0.28', 
                            description='Cambiar a WALL_FOLLOWING.'),
        DeclareLaunchArgument('goal_tolerance', default_value='0.05', 
                            description='Radio para alcanzar meta.'),
        DeclareLaunchArgument('wall_follow_goal_tolerance', default_value='0.08', 
                            description='Radio de captura de meta siguiendo pared.'),
        DeclareLaunchArgument('goal_pass_margin', default_value='0.02', 
                            description='Margen para detenerse al pasar meta.'),
        DeclareLaunchArgument('goal_pass_lateral_tolerance', default_value='0.22', 
                            description='Tolerancia lateral máxima.'),
        DeclareLaunchArgument('near_goal_slow_distance', default_value='0.35', 
                            description='Distancia para reducir velocidad.'),
        DeclareLaunchArgument('near_goal_v_max', default_value='0.025', 
                            description='Velocidad máxima cerca de meta.'),
        DeclareLaunchArgument('scan_front_angle', default_value='0.0', 
                            description='Frente del LaserScan.'),
        DeclareLaunchArgument('v_max', default_value='0.08',         
                            description='Velocidad lineal maxima (m/s).'),
        DeclareLaunchArgument('w_max', default_value='0.40',         
                            description='Velocidad angular maxima (rad/s).'),
        DeclareLaunchArgument('use_aruco_monitor', default_value='true', 
                            description='Arranca monitor ArUco.'),
        DeclareLaunchArgument('odom_offset_x', default_value='0.0', 
                            description='Offset inicial X para odometría.'),
        DeclareLaunchArgument('odom_offset_y', default_value='0.0', 
                            description='Offset inicial Y para odometría.'),
        DeclareLaunchArgument('odom_offset_theta', default_value='0.0', 
                            description='Offset inicial Theta para odometría.'),
        DeclareLaunchArgument('bug_type', default_value='bug2', 
                            description='Tipo de bug (bug2 o bug0).'),
        DeclareLaunchArgument('wall_follow_side', default_value='left', 
                            description='Lado para seguir la pared (left o right).'),

        # ==================== NODOS ====================
        localisation_node,
        ekf_node,
        waypoint_node,
        bug2_node,
        bug0_node,
        aruco_monitor,
        rviz_node,
        # --- Fallback si el bringup no publica el modelo (descomentar ambos) ---
        robot_state_publisher_node,
        joint_state_publisher_node,
    ])
