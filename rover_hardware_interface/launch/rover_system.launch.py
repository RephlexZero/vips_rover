import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("rover_hardware_interface")

    # Declare the launch argument for the URDF file
    default_urdf_model_path = os.path.join(pkg_share, "urdf/rover.urdf.xacro")
    urdf_model_arg = DeclareLaunchArgument(
        name="urdf_model",
        default_value=default_urdf_model_path,
        description="Absolute path to robot urdf file",
    )

    # Load the robot description from the URDF file
    robot_description = Command(["xacro ", LaunchConfiguration("urdf_model")])

    # Node: Robot State Publisher
    # This node publishes the robot's TF tree from the URDF description
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description}],
    )

    # Node: Controller Manager (ros2_control)
    # This is the main node that loads and manages the hardware interface and controllers
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {"robot_description": robot_description},
            os.path.join(pkg_share, "config", "rover_controllers.yaml"),
        ],
        output="screen",
    )

    # Node: Joint State Broadcaster Spawner
    # This node loads the joint_state_broadcaster controller into the controller manager
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Node: Ackermann Steering Controller Spawner
    # This node loads the ackermann_steering_controller into the controller manager
    ackermann_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "ackermann_steering_controller",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    return LaunchDescription(
        [
            urdf_model_arg,
            robot_state_publisher_node,
            control_node,
            joint_state_broadcaster_spawner,
            ackermann_steering_controller_spawner,
        ]
    )
