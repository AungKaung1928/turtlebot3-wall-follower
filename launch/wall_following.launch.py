#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_dir = get_package_share_directory('wall_following_project')
    
    # Default parameters file
    default_params_file = os.path.join(pkg_dir, 'config', 'wall_following_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock'
        ),
        
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Full path to parameter file'
        ),
        
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level (debug, info, warn, error)'
        ),
        
        # Wall follower node
        Node(
            package='wall_following_project',
            executable='wall_follower',
            name='wall_follower_controller',
            parameters=[
                LaunchConfiguration('params_file'),
                {'use_sim_time': LaunchConfiguration('use_sim_time')}
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            emulate_tty=True,
            respawn=False,  # Set to True for production auto-restart
        ),
    ])