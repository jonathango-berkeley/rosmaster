#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid

import math

class RescueRobot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('rescue_robot_main')
        self.node.get_logger().info("RescueRobot initialization started.")

        # Robot state
        self.rescue_mode = False

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Map
        self.map_data = None
        self.map_received = False  # <- log-once flag

        # Aruco Data
        self.aruco_sub = self.node.create_subscription(
            TransformStamped,
            '/aruco/transform',
            self.aruco_callback,
            10
        )
        self.aruco_queue = {}
        self.aruco_saved = []

        # Subscriptions
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.node.get_logger().info("RescueRobot ready.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if not self.map_received:
            self.node.get_logger().info("Map received and updated.")
            self.map_received = True

    def aruco_callback(self, msg: TransformStamped):
        if msg.child_frame_id in self.aruco_saved:
            return

        found_location = self.get_position()
        if msg.child_frame_id in self.aruco_queue.keys:
            self.aruco_queue[msg.child_frame_id]["found_location"] = found_location
            if len(self.aruco_queue[msg.child_frame_id]["last_10"]) > 10:
                self.aruco_queue[msg.child_frame_id]["last_10"].pop(0)

            self.aruco_queue[msg.child_frame_id]["last_10"].append(msg)
            self.aruco_queue[msg.child_frame_id]["location"] = self.filter_location(self.aruco_queue[msg.child_frame_id]["last_10"])
        else:
            self.aruco_queue[msg.child_frame_id] = {
                "location": msg,
                "found_location": found_location,
                "last_10": [msg]
            }
    
    def remove_object(self, child_frame_id):
        try:
            self.aruco_queue.pop(child_frame_id)
            self.aruco_saved.append(child_frame_id)
        except KeyError:
            self.node.get_logger().error(f"Key {child_frame_id} not found")

    def get_position(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )

            return trans

        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")

    def filter_location(self, last_10):
        # TODO
        filtered_location = last_10[-1]

        return filtered_location

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

def main():
    robot = RescueRobot()
    robot.spin()

if __name__ == '__main__':
    main()
