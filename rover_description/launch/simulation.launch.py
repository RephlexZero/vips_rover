#!/usr/bin/env python3

"""
Spawns the rover in Gazebo (gz-sim) with ros2_control.
- One-way /clock bridge to avoid time jump loops
- robot_description passed as a string (Jazzy-safe)
- Wait for ros2_control hardware interfaces before spawning controllers
- Spawns controllers in order and activates them
- RViz is optional (disabled by default to avoid duplicates)
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    SetEnvironmentVariable,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_rover_description = get_package_share_directory("rover_description")

    # Args
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim time"
    )
    declare_urdf_model = DeclareLaunchArgument(
        "urdf_model",
        default_value=os.path.join(pkg_rover_description, "urdf", "rover.urdf.xacro"),
        description="Path to rover xacro",
    )
    declare_start_rviz = DeclareLaunchArgument(
        "start_rviz", default_value="false", description="Start RViz from this launch"
    )

    # Build robot_description via xacro; force it to be a string so Jazzy doesn't parse as YAML
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("urdf_model"),
                " hardware_plugin:=gz_ros2_control/GazeboSimSystem",
                " controllers_file:=rover_controllers_sim.yaml",
                " use_sim_time:=true",
            ]
        ),
        value_type=str,
    )

    # Controller Manager configuration file with parameters
    controller_spawner_params = os.path.join(
        pkg_rover_description, "config", "controllers_spawn.yaml"
    )

    # Robot State Publisher
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            },
        ],
    )

    # Start Gazebo simulation (headless by default)
    gz_sim = Node(
        package="ros_gz_sim",
        executable="gz_sim",
        output="screen",
        arguments=["-r", "empty.sdf"],
    )

    # Spawn model into Gazebo
    create_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "rover", "-topic", "robot_description"],
    )

    # One-way clock bridge (GZ -> ROS) to avoid time jump loops
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # Optional RViz (off by default; Nav2 launch typically starts RViz)
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=IfCondition(LaunchConfiguration("start_rviz")),
    )

    # Wait until ros2_control hardware interfaces exist before spawning controllers
    wait_for_ros2_control = Node(
        package="rover_description",
        executable="wait_for_ros2_control.py",
        name="wait_for_ros2_control",
        output="screen",
        arguments=[
            "--controller-manager",
            "/controller_manager",
            "--joints",
            "rear_left_wheel_joint,rear_right_wheel_joint,front_left_steer_joint,front_right_steer_joint",
            "--timeout",
            "30.0",
        ],
    )

    # Controller Manager with explicit parameters from config file
    controller_spawner_params = os.path.join(
        pkg_rover_description, "config", "controllers_spawn.yaml"
    )

    # Spawners: configure and activate in order
    jsb_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_joint_state_broadcaster",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--activate",
        ],
        parameters=[controller_spawner_params, {"use_sim_time": True}],
    )
    ackermann_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_ackermann_steering_controller",
        output="screen",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
            "--activate",
        ],
        parameters=[controller_spawner_params, {"use_sim_time": True}],
    )

    # Spawn controllers only after wait node exits cleanly
    spawn_after_wait = RegisterEventHandler(
        OnProcessExit(
            target_action=wait_for_ros2_control,
            on_exit=[jsb_spawner, ackermann_spawner],
        )
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_use_sim_time,
            declare_urdf_model,
            declare_start_rviz,
            clock_bridge,
            gz_sim,
            robot_state_publisher_node,
            create_entity,
            wait_for_ros2_control,
            spawn_after_wait,
            rviz_node,
        ]
    )
