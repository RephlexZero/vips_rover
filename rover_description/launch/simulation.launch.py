#!/usr/bin/env python3

"""
Pure simulation: spawns the robot in Gazebo (gz-sim) with ros2_control and
delays controller configuration to avoid races with gz_ros2_control.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_rover_description = get_package_share_directory("rover_description")

    # Launch args
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim time"
    )
    declare_urdf_model = DeclareLaunchArgument(
        "urdf_model",
        default_value=os.path.join(pkg_rover_description, "urdf", "rover.urdf.xacro"),
        description="URDF xacro",
    )

    # Build robot_description (SIM parameters)
    robot_description = Command(
        [
            "xacro ",
            LaunchConfiguration("urdf_model"),
            " hardware_plugin:=gz_ros2_control/GazeboSimSystem",
            " controllers_file:=rover_controllers_sim.yaml",
            " use_sim_time:=true",
        ]
    )

    # Robot State Publisher (sim time)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # Spawn the model into Gazebo
    create_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "rover", "-topic", "robot_description"],
    )

    # Bridge clock
    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
    )

    # Controller spawners with delays and extended timeouts
    jsb_spawner = TimerAction(
        period=3.0,  # wait for controller_manager + interfaces
        actions=[
            Node(
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
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    ackermann_spawner = TimerAction(
        period=4.0,  # after JSB
        actions=[
            Node(
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
                ],
                parameters=[{"use_sim_time": True}],
            )
        ],
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_use_sim_time,
            declare_urdf_model,
            parameter_bridge,
            robot_state_publisher_node,
            create_entity,
            jsb_spawner,
            ackermann_spawner,
        ]
    )
