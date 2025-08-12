#!/usr/bin/env python3
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareInterfaces


class WaitForRos2Control(Node):
    def __init__(self, controller_manager: str, joints: list[str], timeout: float):
        super().__init__("wait_for_ros2_control")
        self.cm_service_name = (
            controller_manager.rstrip("/") + "/list_hardware_interfaces"
        )
        self.req = ListHardwareInterfaces.Request()
        self.cli = self.create_client(ListHardwareInterfaces, self.cm_service_name)
        self.joints = set(joints)
        self.timeout = timeout

    def wait_for_service(self) -> bool:
        self.get_logger().info(f"Waiting for service {self.cm_service_name}...")
        return self.cli.wait_for_service(timeout_sec=self.timeout)

    def joints_available(self) -> bool:
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        if not future.done():
            return False

        result = future.result()
        if result is None:
            return False

        # state_interfaces contain full interface names like "joint_name/interface_name"
        # We need to extract just the joint names
        joint_names = set()
        for iface in result.state_interfaces:
            # iface.name is like "rear_left_wheel_joint/position"
            # Split on '/' and take the first part (joint name)
            joint_name = iface.name.split("/")[0]
            joint_names.add(joint_name)

        missing = self.joints - joint_names
        if missing:
            self.get_logger().info(f"Waiting for joints: {sorted(missing)}")
            return False

        self.get_logger().info("All required joints are available.")
        return True


def main(argv=None):
    if argv is None:
        argv = sys.argv[1:]

    parser = argparse.ArgumentParser()
    parser.add_argument("--controller-manager", default="/controller_manager")
    parser.add_argument("--joints", default="")
    parser.add_argument("--timeout", type=float, default=30.0)

    # Accept ROS 2 launch remap args without failing
    args, ros_args = parser.parse_known_args(argv)

    joints = [j.strip() for j in args.joints.split(",") if j.strip()]
    rclpy.init(args=ros_args)
    node = WaitForRos2Control(args.controller_manager, joints, args.timeout)

    try:
        start = time.time()
        if not node.wait_for_service():
            node.get_logger().error(f"Timeout waiting for {node.cm_service_name}")
            sys.exit(1)

        ok = False
        while (time.time() - start) < args.timeout:
            if node.joints_available():
                ok = True
                break
            rclpy.spin_once(node, timeout_sec=0.2)
            time.sleep(0.3)

        if not ok:
            node.get_logger().error(
                "Timed out waiting for ros2_control to expose required joints."
            )
            sys.exit(1)

    finally:
        node.destroy_node()
        rclpy.shutdown()

    sys.exit(0)


if __name__ == "__main__":
    main()
