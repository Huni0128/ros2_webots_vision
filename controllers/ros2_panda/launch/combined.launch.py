#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1) URDF → TF 변환을 위한 robot_state_publisher 노드 설정
    description_pkg = get_package_share_directory('panda_description')
    urdf_xacro = os.path.join(description_pkg, 'urdf', 'panda.urdf.xacro')
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_xacro])
        }],
        remappings=[
            ('/joint_states', '/panda_joint_states')
        ]
    )

    # 2) Webots 시뮬레이터 실행 설정
    launch_dir = os.path.dirname(os.path.realpath(__file__))
    project_root = os.path.abspath(os.path.join(launch_dir, '..', '..', '..'))
    world_rel = os.path.join('worlds', 'ros2_webots_vision.wbt')
    webots_launch = ExecuteProcess(
        cmd=['webots', '--stdout', '--stderr', world_rel],
        cwd=project_root,
        output='screen'
    )

    # 3) LaunchDescription 반환
    return LaunchDescription([
        rsp_node,
        webots_launch
    ])
