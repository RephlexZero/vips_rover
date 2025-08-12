#!/usr/bin/env python3
import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListHardwareInterfaces

REQUIRED_INTERFACES = {
    # joint_name: required command interface
    'rear_left_wheel_joint': 'velocity',
    'rear_right_wheel_joint': 'velocity',
    'front_left_steer_joint': 'position',
    'front_right_steer_joint': 'position',
}

class WaitForRos2Control(Node):
    def __init__(self, controller_manager, expected_joints, timeout_s):
        super().__init__('wait_for_ros2_control')
        self.cli = self.create_client(ListHardwareInterfaces, f'{controller_manager}/list_hardware_interfaces')
        self.expected_joints = expected_joints
        self.timeout_s = timeout_s

    def wait(self):
        if not self.cli.wait_for_service(timeout_sec=self.timeout_s):
            self.get_logger().error('controller_manager service not available')
            return False
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < self.timeout_s:
            req = ListHardwareInterfaces.Request()
            future = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
            if future.result() is None:
                continue
            cmd_ifaces = future.result().command_interfaces
            # Build mapping joint -> set of command interfaces
            joint_cmd_map = {}
            for ci in cmd_ifaces:
                joint_cmd_map.setdefault(ci.name, set()).add(ci.interface_name)
            # Interface names come as "joint_name" field in ci.name
            # e.g., "rear_left_wheel_joint"
            present = True
            for joint in self.expected_joints:
                required = REQUIRED_INTERFACES.get(joint, None)
                if required is None:
                    continue
                if joint not in joint_cmd_map or required not in joint_cmd_map[joint]:
                    present = False
                    break
            if present:
                self.get_logger().info('ros2_control hardware interfaces are ready')
                return True
            time.sleep(0.5)
        self.get_logger().error('Timed out waiting for ros2_control hardware interfaces')
        return False

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller-manager', default='/controller_manager')
    parser.add_argument('--joints', default='')
    parser.add_argument('--timeout', type=float, default=30.0)
    # Ignore ROS args that get passed automatically
    parser.add_argument('--ros-args', nargs=argparse.REMAINDER, help=argparse.SUPPRESS)
    args = parser.parse_args()

    joints = [j.strip() for j in args.joints.split(',') if j.strip()]

    rclpy.init()
    node = WaitForRos2Control(args.controller_manager, joints, args.timeout)
    ok = node.wait()
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if ok else 1)

if __name__ == '__main__':
    main()