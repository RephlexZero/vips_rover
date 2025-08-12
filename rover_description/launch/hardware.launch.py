"""
Pure hardware launch file - NO SIMULATION COMPONENTS

This launch file is specifically designed for real hardware and includes:
- Hardware CAN interface
- VIPS driver for indoor positioning
- Real-time hardware control interfaces
- Sensor fusion with robot_localization

Use this for:
- Real rover hardware testing
- Field operations
- Hardware-in-the-loop validation
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """
    Real rover hardware system launch
    Assumes CAN interface is already configured (via setup script)
    """

    pkg_rover_description = get_package_share_directory("rover_description")
    pkg_rover_hardware = get_package_share_directory("rover_hardware_interface")

    # Launch arguments
    urdf_model_arg = DeclareLaunchArgument(
        name="urdf_model",
        default_value=os.path.join(pkg_rover_description, "urdf/rover.urdf.xacro"),
        description="Absolute path to robot urdf file",
    )

    vips_serial_port_arg = DeclareLaunchArgument(
        name="vips_serial_port",
        default_value="/dev/ttyUSB0",
        description="Serial port for VIPS communication",
    )

    can_interface_arg = DeclareLaunchArgument(
        name="can_interface", default_value="can0", description="CAN interface name"
    )

    # Robot description with HARDWARE parameters
    robot_description = ParameterValue(
        Command(
            [
                "xacro ",
                LaunchConfiguration("urdf_model"),
                " hardware_plugin:=rover_hardware_interface/RoverSystemHardware",
                " controllers_file:=rover_controllers_hw.yaml",
                " use_sim_time:=false",
            ]
        ),
        value_type=str,
    )

    # 1. Robot State Publisher (NO sim_time)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,  # CRITICAL: Hardware uses wall clock
            }
        ],
        output="screen",
    )

    # 2. VIPS Driver Node
    vips_driver_node = Node(
        package="vips_driver",
        executable="vips_node",
        name="vips_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "serial_port": LaunchConfiguration("vips_serial_port"),
                "use_sim_time": False,  # CRITICAL: Hardware uses wall clock
            }
        ],
    )

    # 3. Hardware Interface & Controller Manager
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {
                "robot_description": robot_description,
                "use_sim_time": False,  # CRITICAL: Hardware uses wall clock
            },
            os.path.join(pkg_rover_description, "config", "rover_controllers_hw.yaml"),
        ],
        output="screen",
    )

    # 4. Joint State Broadcaster (delayed start)
    joint_state_broadcaster_spawner = TimerAction(
        period=3.0,  # Wait for controller manager to be ready
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "joint_state_broadcaster",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ],
    )

    # 5. Ackermann Steering Controller (delayed start)
    ackermann_steering_controller_spawner = TimerAction(
        period=4.0,  # Wait for joint state broadcaster to be ready
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "ackermann_steering_controller",
                    "--controller-manager",
                    "/controller_manager",
                ],
                output="screen",
                parameters=[{"use_sim_time": False}],
            )
        ],
    )

    # 6. Robot Localization (EKF for sensor fusion)
    robot_localization_node = TimerAction(
        period=5.0,  # Wait for controllers to be ready
        actions=[
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[
                    os.path.join(
                        pkg_rover_description, "config", "robot_localization.yaml"
                    ),
                    {"use_sim_time": False},
                ],
                remappings=[
                    ("odometry/filtered", "odometry/filtered"),
                    ("/set_pose", "/initialpose"),
                ],
            )
        ],
    )

    return LaunchDescription(
        [
            # Launch arguments
            urdf_model_arg,
            vips_serial_port_arg,
            can_interface_arg,
            # Hardware system components
            robot_state_publisher_node,
            vips_driver_node,
            control_node,
            # Controllers (delayed start)
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
            # Sensor fusion (delayed start)
            robot_localization_node,
        ]
    )
