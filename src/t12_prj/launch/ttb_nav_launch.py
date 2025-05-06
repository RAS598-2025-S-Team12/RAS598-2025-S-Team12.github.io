#!/usr/bin/env python3
"""
This launch file starts the Turtlebot navigation node.
It loads navigation and robot configuration parameters from YAML files.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory to locate config files
    pkg_share = get_package_share_directory('t12_prj')
    move_base_params = os.path.join(pkg_share, 'config', 'move_base_params.yaml')
    robot_config = os.path.join(pkg_share, 'config', 'robot_config.yaml')
    
    return LaunchDescription([
        Node(
            package='t12_prj',
            executable='ttb_nav',
            name='ttb_nav',
            output='screen',
            parameters=[move_base_params, robot_config, {'use_sim_time': True}],
        )
    ])

