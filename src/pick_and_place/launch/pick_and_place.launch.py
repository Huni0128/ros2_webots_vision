#!/usr/bin/env python3
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 1) 런치 선언
    declare_scan_limit = DeclareLaunchArgument(
        'scan_limit_deg',             # 스캔 제한 각도 (deg)
        default_value='90.0',
        description='Scan limit in degrees'
    )
    declare_scan_step = DeclareLaunchArgument(
        'scan_step_deg',              # 스캔 스텝 각도 (deg)
        default_value='2.0',
        description='Scan step in degrees'
    )
    declare_scan_joint = DeclareLaunchArgument(
        'scan_joint_index',           # 스캔할 관절 인덱스
        default_value='2',
        description='Index of joint to scan'
    )
    declare_base_frame = DeclareLaunchArgument(
        'base_frame',                 # TF 기반 프레임
        default_value='base_link',
        description='Reference frame for TF lookup'
    )
    declare_camera_frame = DeclareLaunchArgument(
        'camera_frame',               # 카메라 프레임 ID
        default_value='camera_link',
        description='Camera frame id'
    )
    declare_object_frame = DeclareLaunchArgument(
        'object_frame',               # 객체 프레임 ID
        default_value='object_frame',
        description='Object frame id published by vision'
    )

    # 2) LaunchConfiguration 바인딩 (값 참조)
    scan_limit_deg  = LaunchConfiguration('scan_limit_deg')
    scan_step_deg   = LaunchConfiguration('scan_step_deg')
    scan_joint_idx  = LaunchConfiguration('scan_joint_index')
    base_frame      = LaunchConfiguration('base_frame')
    camera_frame    = LaunchConfiguration('camera_frame')
    object_frame    = LaunchConfiguration('object_frame')

    # 3) 노드 정의
    command_sub_node = Node(
        package='pick_and_place',     # 패키지 이름
        executable='command_sub',     # 실행 파일
        name='command_sub',           # 노드 이름
        output='screen'               # 콘솔 출력 설정
    )

    scan_move_node = Node(
        package='pick_and_place',     # 스캔 이동 노드 패키지
        executable='scan_move',       # 실행 파일
        name='scan_move',             # 노드 이름
        output='screen',
        parameters=[{                # 노드 매개변수 설정
            'scan.limit_deg':    scan_limit_deg,
            'scan.step_deg':     scan_step_deg,
            'scan.joint_index':  scan_joint_idx,
            'tf.base_frame':     base_frame,
            'tf.camera_frame':   camera_frame,
            'tf.object_frame':   object_frame,
        }]
    )

    static_tf_node = Node(
        package='tf2_ros',            # TF2 브로드캐스터 패키지
        executable='static_transform_publisher',
        name='test_object_broadcaster',  # 노드 이름
        arguments=[                   # 고정 변환 파라미터
            '0.5', '0.0', '0.0',      # x, y, z 위치
            '0', '0', '0',            # roll, pitch, yaw
            camera_frame,             # 부모 프레임
            object_frame              # 자식 프레임
        ]
    )

    # 4) LaunchDescription에 노드 추가
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
    ])
