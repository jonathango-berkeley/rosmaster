#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TwistPublisher(Node):
    def __init__(self):
        super().__init__('twist_publisher')

        # Declare parameters with default values
        self.declare_parameter('linear_x', 0.0)
        self.declare_parameter('linear_y', 0.0)
        self.declare_parameter('linear_z', 0.0)
        self.declare_parameter('angular_x', 0.0)
        self.declare_parameter('angular_y', 0.0)
        self.declare_parameter('angular_z', 0.0)

        # Create publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create timer to publish at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_twist)

    def publish_twist(self):
        msg = Twist()
        msg.linear.x = self.get_parameter('linear_x').get_parameter_value().double_value
        msg.linear.y = self.get_parameter('linear_y').get_parameter_value().double_value
        msg.linear.z = self.get_parameter('linear_z').get_parameter_value().double_value
        msg.angular.x = self.get_parameter('angular_x').get_parameter_value().double_value
        msg.angular.y = self.get_parameter('angular_y').get_parameter_value().double_value
        msg.angular.z = self.get_parameter('angular_z').get_parameter_value().double_value

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing Twist: {msg}')

def main(args=None):
    rclpy.init(args=args)
    node = TwistPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()