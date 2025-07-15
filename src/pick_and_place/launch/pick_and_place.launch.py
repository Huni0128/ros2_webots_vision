#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 파라미터 기본값 설정
    scan_limit_deg  = LaunchConfiguration('scan_limit_deg',  default='90.0')
    scan_step_deg   = LaunchConfiguration('scan_step_deg',   default='2.0')
    scan_joint_idx  = LaunchConfiguration('scan_joint_index',default='2')

    return LaunchDescription([
        # command_sub 노드: 명령 수신 → 상태 퍼블리시
        Node(
            package='pick_and_place',
            executable='command_sub',
            name='command_sub',
            output='screen'
        ),

        # scan_move 노드: 상태 수신 → 관절 제어 수행
        Node(
            package='pick_and_place',
            executable='scan_move',
            name='scan_move',
            output='screen',
            parameters=[{
                'scan.limit_deg':    scan_limit_deg,
                'scan.step_deg':     scan_step_deg,
                'scan.joint_index':  scan_joint_idx
            }]
        ),
    ])
