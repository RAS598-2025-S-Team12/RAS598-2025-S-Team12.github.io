#!/usr/bin/env python3
"""
This launch file integrates the main nodes:
- Turtlebot navigation node (ttb_nav)
- Turtlebot state publisher (ttb_state_pub)
- UR5 controller node (ur5_controller)
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Launch the Turtlebot navigation node
        Node(
            package='t12_prj',
            executable='ttb_nav',
            name='ttb_nav',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        # Launch the Turtlebot state publisher node
        Node(
            package='t12_prj',
            executable='ttb_state_pub',
            name='ttb_state_pub',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
        # Launch the UR5 controller node
        Node(
            package='t12_prj',
            executable='ur5_controller',
            name='ur5_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),
    ])

