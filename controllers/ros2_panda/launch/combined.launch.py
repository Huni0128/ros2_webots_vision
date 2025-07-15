#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # robot_state_publisher 노드 (XACRO → URDF)
    robot_description_content = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('panda_description'),
            'urdf',
            'panda.urdf.xacro'
        ])
    ])

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
        remappings=[('/joint_states', '/panda_joint_states')]
    )

    # Webots 실행 (월드 파일 경로 지정)
    world_path = PathJoinSubstitution([
        FindPackageShare('ros2_panda'),
        '..', '..', '..', '..',
        'worlds',
        'ros2_webots_vision.wbt'
    ])

    webots_launch = ExecuteProcess(
        cmd=['webots', '--stdout', '--stderr', world_path],
        output='screen'
    )

    return LaunchDescription([
        rsp_node,
        webots_launch
    ])
