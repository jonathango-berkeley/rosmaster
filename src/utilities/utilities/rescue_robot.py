#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

import math

class Robot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('robot_main')
        self.node.get_logger().info("Robot initialization started.")

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Map
        self.map_data = None

        # Subscriptions
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Timer for pose updates
        self.timer = self.node.create_timer(0.1, self.update_position)

        self.node.get_logger().info("Robot ready.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        self.node.get_logger().info_once("Map received and updated.")
        # Optional: print map size and resolution
        self.node.get_logger().debug(
            f"Map size: {msg.info.width}x{msg.info.height}, resolution: {msg.info.resolution} m/cell"
        )

    def update_position(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )
            self.x = trans.transform.translation.x
            self.y = trans.transform.translation.y

            # Convert quaternion to yaw
            q = trans.transform.rotation
            siny_cosp = 2 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
            self.yaw = math.atan2(siny_cosp, cosy_cosp)

            self.node.get_logger().info_throttle(1.0, f"Robot pose: x={self.x:.2f}, y={self.y:.2f}, yaw={math.degrees(self.yaw):.1f}°")

        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")

    def spin(self):
        self.node.get_logger().info("Robot is running...")
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down robot...")
        finally:
            self.shutdown()

    def shutdown(self):
        self.node.get_logger().info("Cleaning up resources.")
        self.node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    robot = Robot()
    robot.spin()
