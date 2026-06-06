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



    # Leer URDF correctamente
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )



    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=rviz_file,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]

    )



    # Retrasar RViz para evitar errores de carga

    delayed_rviz = TimerAction(
        period=2.0,
        actions=[rviz_node]
    )



    ld = LaunchDescription([
        rviz_arg,
        robot_state_publisher_node,
        delayed_rviz
    ])

    return ld