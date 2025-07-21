#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get directories
    pkg_dir = get_package_share_directory('wall_following_project')
    turtlebot3_gazebo_dir = get_package_share_directory('turtlebot3_gazebo')
    
    # World file (using TurtleBot3 world)
    world = os.path.join(turtlebot3_gazebo_dir, 'worlds', 'turtlebot3_world.world')
    
    return LaunchDescription([
        # Launch Gazebo with TurtleBot3 world
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                turtlebot3_gazebo_dir, '/launch/turtlebot3_world.launch.py'
            ]),
            launch_arguments={'world': world}.items(),
        ),
        
        # Launch wall follower
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                pkg_dir, '/launch/wall_following.launch.py'
            ]),
        ),
    ])