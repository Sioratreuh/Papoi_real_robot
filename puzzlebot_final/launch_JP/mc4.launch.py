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

    # Instanciamos ÚNICAMENTE al Robot 1 en el origen
    robot1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch_file),
        launch_arguments={
            'robot_name': 'robot1',
            'x0': '0.0',
            'y0': '0.0'
        }.items()
    )

    # Transformación estática para enlazar el robot al mundo global
    tf_world_to_odom = Node(
        package='tf2_ros', 
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1/odom']
    )

    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_file]
    )

    return LaunchDescription([
        tf_world_to_odom,
        robot1,
        rviz_node
    ])