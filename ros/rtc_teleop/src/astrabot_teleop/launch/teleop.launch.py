#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = LaunchConfiguration('config_file')
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'config_file',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('astrabot_teleop'), 'config', 'teleop.yaml']
                ),
                description='astrabot_teleop ROS 参数文件',
            ),
            Node(
                package='astrabot_teleop',
                executable='astrabot_teleop_node',
                name='astrabot_teleop',
                output='screen',
                parameters=[config_file],
            ),
        ]
    )
