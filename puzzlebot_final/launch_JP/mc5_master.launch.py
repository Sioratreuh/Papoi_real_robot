import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node

def generate_launch_description():
    pkg_puzzlebot_sim = get_package_share_directory('puzzlebot_sim')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # ARGUMENTO DE TERMINAL (Por defecto usa bug0)
    algo_arg = DeclareLaunchArgument(
        'algo', 
        default_value='bug0', 
        description='Algoritmo a utilizar: bug0 o bug2'
    )
    algo = LaunchConfiguration('algo')

    # 1. Gazebo con la pista de pruebas
    world_file = os.path.join(pkg_puzzlebot_sim, 'worlds', 'bug0_test.world')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': world_file}.items()
    )

    # 2. Inyector del robot
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_puzzlebot_sim, 'launch', 'spawn_robot.launch.py')
        ),
        launch_arguments={'x0': '0.0', 'y0': '0.0'}.items()
    )

    # 3. NODO BUG 0 (Solo se activa si algo == 'bug0')
    bug0_node = Node(
        package='puzzlebot_sim',
        executable='bug0_node',
        name='bug0_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", algo, "' == 'bug0'"]))
    )

    # 4. NODO BUG 2 (Solo se activa si algo == 'bug2')
    bug2_node = Node(
        package='puzzlebot_sim',
        executable='bug2_node',
        name='bug2_node',
        output='screen',
        condition=IfCondition(PythonExpression(["'", algo, "' == 'bug2'"]))
    )

    # 5. NODO DE RVIZ2 (Sincronizado con el tiempo de Gazebo)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}] # ESTA LÍNEA DESCONGELA EL MOVIMIENTO EN RVIZ
    )

    return LaunchDescription([
        algo_arg,
        gazebo,
        spawn_robot,
        bug0_node,
        bug2_node,
        rviz_node
    ])