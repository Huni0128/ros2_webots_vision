from launch import LaunchDescription
from webots_ros2_driver.webots_launcher import WebotsLauncher
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path   = get_package_share_directory('vision_sim')
    world_path = os.path.join(pkg_path, 'worlds', 'vision_world.wbt')
    ini_path   = os.path.join(pkg_path, 'resource', 'panda.ini')
    urdf_path  = os.path.join(pkg_path, 'resource', 'dummy.urdf')  # 실제 파일 경로

    return LaunchDescription([
        WebotsLauncher(
            world=world_path,
            ros2_supervisor=True
        ),
        Node(
            package='webots_ros2_driver',
            executable='driver',
            name='panda_driver',
            parameters=[
                {'robot_description': urdf_path},  # 문자열 대신 파일 경로
                {'robot': 'panda'},
                {'ini_filename': ini_path}
            ],
            output='screen'
        )
    ])
