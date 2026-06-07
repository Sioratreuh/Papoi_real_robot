from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'puzzlebot_final'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch_LAP', '*.py'))),
        (os.path.join('share', package_name, 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools', 'rclpy'],
    zip_safe=True,
    maintainer='peps',
    maintainer_email='ppangelhr@gmail.com',
    description='Launch files for running Nav2 on the physical Puzzlebot.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'localisation_node = puzzlebot_final.localisation_node:main',
            'ekf_node          = puzzlebot_final.ekf_node:main',
            'bug2_node         = puzzlebot_final.bug2_node:main',
            'waypoint_manager  = puzzlebot_final.waypoint_manager:main',
            'aruco_monitor     = puzzlebot_final.aruco_monitor:main',
            'joint_state_publisher = puzzlebot_final.joint_state_publisher:main',
        ],
    },
)
