from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = 'puzzlebot_sim'
    pkg_share = get_package_share_directory(pkg_name)

    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    rviz_file = os.path.join(pkg_share, 'rviz', 'PuzzlebotConfig.rviz')

    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # 1. Publicador del modelo 3D
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    # 2. Simulador de cinemática (Parte 1)
    sim_node = Node(
        package='puzzlebot_sim',
        executable='sim_node',
        name='puzzlebot_sim_node',
        output='screen'
    )

    # 3. Localización y Odometría (Parte 2)
    loc_node = Node(
        package='puzzlebot_sim',
        executable='loc_node',
        name='localisation_node',
        output='screen'
    )

    # 4. Controlador de posición (Parte 3)
    ctrl_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='puzzlebot_sim',
                executable='ctrl_node',
                name='control_node',
                output='screen'
            )
        ]
    )

    # 5. Generador de trayectoria (Parte 3)
    traj_node = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='puzzlebot_sim',
                executable='traj_node',
                name='trajectory_node',
                output='screen'
            )
        ]
    )

    # 6. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        sim_node,
        loc_node,
        #ctrl_node,
        #traj_node,
        rviz_node
    ])