#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    
    print('Launching DRL Nodes for bot_3 ..')

    return LaunchDescription([
        Node(
            package='turtlebot',
            executable='dqn_gazebo_3.py',
            name='dqn_gazebo_3',
            output='screen'),

        Node(
            package='turtlebot',
            executable='dqn_environment_3.py',
            name='dqn_environment_3',
            output='screen'),

        Node(
            package='turtlebot',
            executable='dqn_agent_3.py',
            name='dqn_agent_3',
            output='screen'),
    ])