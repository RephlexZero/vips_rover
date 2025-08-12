#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_rover_description = get_package_share_directory("rover_description")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time", default_value="true", description="Use Gazebo sim time"
    )
    declare_urdf_model = DeclareLaunchArgument(
        "urdf_model",
        default_value=os.path.join(pkg_rover_description, "urdf", "rover.urdf.xacro"),
        description="URDF xacro",
    )

    # Build robot_description using xacro. Use a list of tokens (no trailing spaces)
    xacro_cmd = Command([
        'xacro ',
        LaunchConfiguration('urdf_model'),
        ' hardware_plugin:=gz_ros2_control/GazeboSimSystem',
        ' controllers_file:=rover_controllers_sim.yaml',
        ' use_sim_time:=true'
    ])
    # Force the parameter type to string so launch doesn't try to parse as YAML
    robot_description_param = ParameterValue(xacro_cmd, value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "robot_description": robot_description_param,
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }
        ],
    )

    create_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=["-name", "rover", "-topic", "robot_description"],
    )

    parameter_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="parameter_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock@gz.msgs.Clock"],
    )

    jsb_spawner = TimerAction(
        period=3.0,
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
        period=4.0,
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
