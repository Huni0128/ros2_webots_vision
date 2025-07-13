#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='object_detector',
            executable='object_detector',
            name='box_detector',
            output='screen',
        ),
        Node(
            package='object_depth_estimator',
            executable='object_depth_estimator',
            name='depth_estimator',
            output='screen',
        ),
    ])
