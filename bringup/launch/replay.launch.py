#!/usr/bin/env python3
"""
Teaching Replay Launch File

교시 데이터를 재생하는 런치 파일
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 런치 인자 선언
    csv_file_arg = DeclareLaunchArgument(
        'csv_file',
        default_value='',
        description='Path to teaching CSV file'
    )
    
    replay_speed_arg = DeclareLaunchArgument(
        'replay_speed',
        default_value='1.0',
        description='Replay speed multiplier (1.0 = original speed, 0.5 = half speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop replay continuously'
    )
    
    # Replay Node
    replay_node = Node(
        package='my_robot_ros2_control',
        executable='teaching_replay_node',
        name='teaching_replay_node',
        output='screen',
        parameters=[{
            'csv_file': LaunchConfiguration('csv_file'),
            'replay_speed': LaunchConfiguration('replay_speed'),
            'loop': LaunchConfiguration('loop'),
            'controller_name': 'my_robot_arm_controller'
        }]
    )
    
    return LaunchDescription([
        csv_file_arg,
        replay_speed_arg,
        loop_arg,
        replay_node,
    ])