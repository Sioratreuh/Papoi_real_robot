from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch.actions import GroupAction
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('puzzlebot_sim')
    urdf_file = os.path.join(pkg_share, 'urdf', 'puzzlebot.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    robot_name = LaunchConfiguration('robot_name')
    x0 = LaunchConfiguration('x0')
    y0 = LaunchConfiguration('y0')

    robot_group = GroupAction([
        PushRosNamespace(robot_name),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{
                'robot_description': robot_desc, 
                'use_sim_time': True
            }],
            # ESTE TÚNEL CONECTA LAS LLANTAS DE GAZEBO CON RVIZ
            remappings=[('joint_states', '/joint_states')] 
        ),
        
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen',
            arguments=[
                '-topic', 'robot_description', 
                '-entity', robot_name, 
                '-x', x0, 
                '-y', y0, 
                '-z', '0.15'
            ]
        ),

        Node(
            package='puzzlebot_sim',
            executable='loc_node',
            name='localisation_node',
            parameters=[{'x0': x0, 'y0': y0}]
        )
    ])

    return LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot1'),
        DeclareLaunchArgument('x0', default_value='0.0'),
        DeclareLaunchArgument('y0', default_value='0.0'),
        robot_group
    ])