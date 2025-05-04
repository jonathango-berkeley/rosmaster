#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_broadcaster import TransformBroadcaster
import tf2_ros
import time


class MapToOdomPublisher(Node):
    def __init__(self):
        super().__init__('get_position')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher (can also use a broadcaster if desired)
        self.publisher = self.create_publisher(TransformStamped, '/current_position', 10)

        # Timer to publish regularly
        self.timer = self.create_timer(0.1, self.publish_transform)  # 10 Hz

    def publish_transform(self):
        try:
            now = rclpy.time.Time()
            # Lookup transform from 'map' to 'odom'
            transform = self.tf_buffer.lookup_transform('map', 'base_footprint', now)

            self.publisher.publish(transform)
            self.get_logger().info(f"Published map -> odom: {transform.transform.translation}")

        except Exception as e:
            self.get_logger().warn(f"Could not lookup transform from map to odom: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = MapToOdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
