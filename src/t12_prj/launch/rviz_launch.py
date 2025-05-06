#!/usr/bin/env python3
"""
This launch file starts RViz2 with a specified configuration file.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '<path_to_your_rviz_config_file>'],  # Replace with your RViz config file path
        ),
    ])

