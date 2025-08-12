#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import math

class ManualOdomPublisher(Node):
    def __init__(self):
        super().__init__('manual_odom_publisher')
        
        # Transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer to publish transform at 50Hz
        self.timer = self.create_timer(0.02, self.publish_transform)
        
        self.get_logger().info('Manual odom->base_link transform publisher started')
        
        # Initialize pose (stationary for now)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
    def publish_transform(self):
        # Create transform message
        t = TransformStamped()
        
        # Header
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        
        # Translation (currently stationary)
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        
        # Rotation (quaternion from theta)
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(self.theta / 2.0)
        t.transform.rotation.w = math.cos(self.theta / 2.0)
        
        # Send the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    
    odom_publisher = ManualOdomPublisher()
    
    try:
        rclpy.spin(odom_publisher)
    except KeyboardInterrupt:
        pass
    
    odom_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
