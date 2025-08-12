#!/usr/bin/env python3

"""
Complete rover navigation launch file for hardware
Launches rover with navigation, SLAM, sensor fusion, and VIPS
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    rover_description_dir = get_package_share_directory('rover_description')
    rover_nav_dir = get_package_share_directory('rover_navigation')
    
    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true')

    # Include rover hardware launch (robot + controllers + VIPS + EKF)
    rover_hardware_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_description'),
                'launch',
                'hardware.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # Include navigation launch
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('rover_navigation'),
                'launch',
                'navigation_hw.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )

    # RViz2 with navigation config
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare('rover_navigation'),
        'config',
        'nav2_default_view.rviz'
    ])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/goal_pose', '/goal_pose'),
            ('/initialpose', '/initialpose')
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    
    # Add nodes and launch files
    ld.add_action(rover_hardware_launch)
    ld.add_action(navigation_launch)
    ld.add_action(rviz_node)

    return ld
