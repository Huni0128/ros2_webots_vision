#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1) 런치 아규먼트 선언
    declare_scan_limit = DeclareLaunchArgument(
        'scan_limit_deg',
        default_value='90.0',
        description='Scan limit in degrees'
    )
    declare_scan_step = DeclareLaunchArgument(
        'scan_step_deg',
        default_value='2.0',
        description='Scan step in degrees'
    )
    declare_scan_joint = DeclareLaunchArgument(
        'scan_joint_index',
        default_value='2',
        description='Index of joint to scan'
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',
        default_value='base_link',
        description='Reference frame for TF lookup'
    )
    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',
        default_value='camera_link',
        description='Camera frame id'
    )
    declare_object_frame = DeclareLaunchArgument(
        'object_frame',
        default_value='object_frame',
        description='Object frame id published by vision'
    )

    # 2) LaunchConfiguration 바인딩
    scan_limit_deg = LaunchConfiguration('scan_limit_deg')
    scan_step_deg  = LaunchConfiguration('scan_step_deg')
    scan_joint_idx = LaunchConfiguration('scan_joint_index')
    base_frame     = LaunchConfiguration('base_frame')
    camera_frame   = LaunchConfiguration('camera_frame')
    object_frame   = LaunchConfiguration('object_frame')

    # 3) 노드 정의
    command_sub_node = Node(
        package='pick_and_place',
        executable='command_sub',
        name='command_sub',
        output='screen'
    )

    scan_move_node = Node(
        package='pick_and_place',
        executable='scan_move',
        name='scan_move',
        output='screen',
        parameters=[{
            'scan.limit_deg':    scan_limit_deg,
            'scan.step_deg':     scan_step_deg,
            'scan.joint_index':  scan_joint_idx,
            'tf.base_frame':     base_frame,
            'tf.camera_frame':   camera_frame,
            'tf.object_frame':   object_frame,
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='test_object_broadcaster',
        output='screen',
        arguments=[
            '0.5', '0.0', '0.0',   # x, y, z
            '0', '0', '0',         # roll, pitch, yaw
            camera_frame,
            object_frame
        ]
    )

    center_control_node = Node(
        package='pick_and_place',
        executable='center_control',
        name='center_control',
        output='screen',
        parameters=[{
            'control.pan_joint_index':  2,
            'control.tilt_joint_index': 1,
            'control.kp_x':             0.5,
            'control.kp_y':             0.5,
            'control.threshold':        0.01
        }]
    )

    # 4) LaunchDescription에 모든 선언 및 노드 추가
    return LaunchDescription([
        declare_scan_limit,
        declare_scan_step,
        declare_scan_joint,
        declare_base_frame,
        declare_camera_frame,
        declare_object_frame,

        command_sub_node,
        scan_move_node,
        static_tf_node,
        center_control_node,
    ])
