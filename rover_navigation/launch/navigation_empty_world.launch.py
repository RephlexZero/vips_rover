#!/usr/bin/env python3

"""
Navigation stack for an empty world: no SLAM, no lidar, static map-less planning.
Assumes open space and relies on odometry only.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    rover_nav_dir = get_package_share_directory('rover_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')

    nav2_params_file = os.path.join(rover_nav_dir, 'config', 'nav2_params_empty_world.yaml')

    stdout_linebuf_envvar = SetEnvironmentVariable('RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation time if true')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='log level')

    # Only remap odom; keep /cmd_vel default so the converter sees Nav2 output
    nav2_remappings = [
        ('/odom', '/ackermann_steering_controller/odometry'),
    ]

    # Static map->odom identity TF (start BEFORE Nav2)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_static_tf',
        output='screen',
        arguments=['0','0','0','0','0','0','map','odom']
    )

    bringup_cmd_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=nav2_remappings),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        # Required for BT recovery actions (spin, backup, etc.)
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            arguments=['--ros-args', '--log-level', log_level],
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['controller_server',
                                        'smoother_server',
                                        'behavior_server',
                                        'planner_server',
                                        'bt_navigator']}]),
    ])

    # Cmd_vel to Ackermann converter: subscribe to /cmd_vel, publish /ackermann_cmd
    cmd_vel_converter = Node(
        package='rover_navigation',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel'),
            ('/ackermann_cmd', '/ackermann_steering_controller/reference_unstamped')
        ])

    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(static_tf)            # ensure TF exists before Nav2
    ld.add_action(bringup_cmd_group)
    ld.add_action(cmd_vel_converter)
    return ld
