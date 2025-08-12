#!/usr/bin/env python3

"""
Convert cmd_vel messages to ackermann steering commands for the rover.
This node subscribes to /cmd_vel and publishes to /ackermann_cmd.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDriveStamped
import math

class CmdVelToAckermann(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_ackermann')
        
        # Parameters
        self.declare_parameter('wheelbase', 0.567)  # Distance between front and rear axles (meters)
        self.declare_parameter('max_steering_angle', 0.7854)  # 45 degrees in radians
        self.declare_parameter('max_speed', 2.0)  # Maximum speed in m/s
        
        self.wheelbase = self.get_parameter('wheelbase').get_parameter_value().double_value
        self.max_steering_angle = self.get_parameter('max_steering_angle').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        
        # Publishers and subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.ackermann_pub = self.create_publisher(
            AckermannDriveStamped,
            '/ackermann_cmd',
            10
        )
        
        self.get_logger().info('CMD_VEL to Ackermann converter started')
        self.get_logger().info(f'Wheelbase: {self.wheelbase}m')
        self.get_logger().info(f'Max steering angle: {math.degrees(self.max_steering_angle):.1f}°')
        self.get_logger().info(f'Max speed: {self.max_speed} m/s')
    
    def cmd_vel_callback(self, msg):
        """
        Convert Twist message to AckermannDriveStamped message.
        
        The conversion uses bicycle model kinematics:
        steering_angle = atan(angular_velocity * wheelbase / linear_velocity)
        
        For safety, we limit both speed and steering angle.
        """
        ackermann_msg = AckermannDriveStamped()
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = 'base_link'
        
        # Extract linear and angular velocities
        linear_vel = msg.linear.x
        angular_vel = msg.angular.z
        
        # Limit linear velocity
        if abs(linear_vel) > self.max_speed:
            linear_vel = math.copysign(self.max_speed, linear_vel)
        
        # Calculate steering angle using bicycle model
        if abs(linear_vel) < 0.01:  # Avoid division by zero for stationary rotation
            if abs(angular_vel) > 0.01:
                # For pure rotation, use maximum steering angle
                steering_angle = math.copysign(self.max_steering_angle, angular_vel)
                # Set a small forward velocity to enable rotation
                linear_vel = 0.1
            else:
                steering_angle = 0.0
        else:
            # Normal case: calculate steering angle from bicycle model
            # steering_angle = atan(angular_vel * wheelbase / linear_vel)
            steering_angle = math.atan(angular_vel * self.wheelbase / linear_vel)
        
        # Limit steering angle
        if abs(steering_angle) > self.max_steering_angle:
            steering_angle = math.copysign(self.max_steering_angle, steering_angle)
        
        # Populate Ackermann message
        ackermann_msg.drive.speed = linear_vel
        ackermann_msg.drive.steering_angle = steering_angle
        
        # Publish the message
        self.ackermann_pub.publish(ackermann_msg)
        
        # Debug logging (throttled)
        if abs(linear_vel) > 0.01 or abs(steering_angle) > 0.01:
            self.get_logger().debug(
                f'CMD: vel={linear_vel:.2f}m/s, steer={math.degrees(steering_angle):.1f}°'
            )

def main(args=None):
    rclpy.init(args=args)
    
    converter = CmdVelToAckermann()
    
    try:
        rclpy.spin(converter)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            converter.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

if __name__ == '__main__':
    main()
