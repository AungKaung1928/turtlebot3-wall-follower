#!/usr/bin/env python3
"""TurtleBot3 burger + wall follower in Gazebo Harmonic (gz-sim 8) via ros_gz.

Replaces the Gazebo Classic turtlebot3_gazebo path. The sim side is pure gz:
DiffDrive + gpu_lidar system plugins in description/turtlebot3_burger.urdf.xacro,
bridged to ROS by ros_gz_bridge with config/gz_bridge.yaml.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('wall_following_project')
    ros_gz_sim_dir = get_package_share_directory('ros_gz_sim')

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    yaw = LaunchConfiguration('yaw')
    params_file = LaunchConfiguration('params_file')

    declared = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument(
            'world',
            default_value=os.path.join(pkg_dir, 'worlds', 'wall_follow_world.sdf'),
            description='SDF world file'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='true = gz server only, no GUI (WSL / CI)'),
        DeclareLaunchArgument('x_pose', default_value='-2.0'),
        DeclareLaunchArgument('y_pose', default_value='-2.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_dir, 'config', 'wall_following_params.yaml'),
            description='Wall follower parameter file'),
    ]

    # model://turtlebot3_world -> this package's models/; package://turtlebot3_description/meshes
    # resolves through the parent of that package's share dir.
    tb3_desc_parent = os.path.dirname(get_package_share_directory('turtlebot3_description'))
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.pathsep.join([os.path.join(pkg_dir, 'models'), tb3_desc_parent,
                               os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))

    gz_args = PythonExpression([
        "'-r -v 3 ' + ('-s ' if '", headless, "' == 'true' else '') + '", world, "'"])
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_dir, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args, 'on_exit_shutdown': 'true'}.items(),
    )

    xacro_file = os.path.join(pkg_dir, 'description', 'turtlebot3_burger.urdf.xacro')
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': Command(['xacro ', xacro_file]),
        }],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_burger',
        output='screen',
        arguments=['-topic', '/robot_description', '-name', 'burger',
                   '-x', x_pose, '-y', y_pose, '-z', '0.01', '-Y', yaw],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{
            'config_file': os.path.join(pkg_dir, 'config', 'gz_bridge.yaml'),
            'use_sim_time': use_sim_time,
        }],
    )

    # params_file is passed explicitly: launch arguments are global, and any included
    # launch file that also declares 'params_file' would otherwise override ours.
    wall_follower = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_dir, 'launch', 'wall_following.launch.py')),
        launch_arguments={'params_file': params_file,
                          'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription(declared + [
        resource_path,
        gz_sim,
        robot_state_publisher,
        bridge,
        TimerAction(period=5.0, actions=[spawn]),
        TimerAction(period=8.0, actions=[wall_follower]),
    ])
