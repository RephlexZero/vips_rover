#!/usr/bin/env python3

"""
Launch file for rover navigation in simulation mode with SLAM
Provides autonomous navigation with click-to-navigate functionality
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directories
    rover_nav_dir = get_package_share_directory('rover_navigation')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Config files
    nav2_params_file = os.path.join(rover_nav_dir, 'config', 'nav2_params_sim.yaml')
    # SLAM disabled for lidar-less setup
    
    # Remappings for ackermann drive
    nav2_remappings = [
        ('/odom', '/ackermann_steering_controller/odometry')
    ]

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level')

    # Static map->odom identity TF (start BEFORE Nav2)
    static_map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_tf',
        output='screen',
        arguments=['0','0','0','0','0','0','map','odom']
    )

    # Nav2 bringup group
    bringup_cmd_group = GroupAction([
        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=nav2_remappings),

        Node(
            package='nav2_smoother',
            executable='smoother_server',
            name='smoother_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_waypoint_follower',
            executable='waypoint_follower',
            name='waypoint_follower',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level]),

        Node(
            package='nav2_velocity_smoother',
            executable='velocity_smoother',
            name='velocity_smoother',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=[('/cmd_vel', '/cmd_vel'), ('/cmd_vel_smoothed', '/cmd_vel_smoothed')] + nav2_remappings),

        # Collision monitor disabled for simulation (no sensors)
        # Node(
        #     package='nav2_collision_monitor',
        #     executable='collision_monitor',
        #     name='collision_monitor',
        #     output='screen',
        #     respawn=use_respawn,
        #     respawn_delay=2.0,
        #     parameters=[nav2_params_file, {'use_sim_time': use_sim_time}],
        #     arguments=['--ros-args', '--log-level', log_level]),

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
                                        'planner_server',
                                        'behavior_server',
                                        'bt_navigator',
                                        'waypoint_follower',
                                        'velocity_smoother']}]),
    ])

    # SLAM Toolbox
    # SLAM removed (no lidar) – map frame is maintained by a static TF

    # Cmd_vel to Ackermann converter
    cmd_vel_converter = Node(
        package='rover_navigation',
        executable='cmd_vel_to_ackermann.py',
        name='cmd_vel_to_ackermann',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[
            ('/cmd_vel', '/cmd_vel_smoothed'),
            ('/ackermann_cmd', '/ackermann_steering_controller/reference_unstamped')
        ])

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(static_map_odom_tf)      # ensure TF exists before Nav2
    ld.add_action(bringup_cmd_group)
    # no slam_toolbox_node
    ld.add_action(cmd_vel_converter)

    return ld
