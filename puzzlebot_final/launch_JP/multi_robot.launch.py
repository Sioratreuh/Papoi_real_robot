from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_sim')
    spawn_launch_file = os.path.join(pkg_share, 'launch', 'spawn_robot.launch.py')
    rviz_file = os.path.join(pkg_share, 'rviz', 'PuzzlebotConfig.rviz')

    # Instancia del Robot 1
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot1',
            'x0': '1.0',  # <-- Modificado
            'y0': '1.0'   # <-- Modificado
        }.items()
    )

    # Instancia del Robot 2
    robot2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot2',
            'x0': '-1.0', # <-- Modificado
            'y0': '-1.0'  # <-- Modificado
        }.items()
    )

    # Nodos estáticos para enlazar el marco global "world" a los "odom" individuales
    tf_world_to_odom1 = Node(package='tf2_ros', executable='static_transform_publisher',
                             arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1/odom'])
    
    tf_world_to_odom2 = Node(package='tf2_ros', executable='static_transform_publisher',
                             arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot2/odom'])

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        tf_world_to_odom1,
        tf_world_to_odom2,
        robot1,
        robot2,
        rviz_node
    ])