#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        self.subscription = self.create_subscription(
            TransformStamped,
            'aruco/filtered_transform',
            self.listener_callback,
            10)

        self.pose_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.pose_published = False  # Ensure only one message is sent

    def listener_callback(self, msg):
        if not self.pose_published:
            pose_msg = PoseStamped()
            pose_msg.header.stamp = msg.header.stamp
            pose_msg.header.frame_id = msg.header.frame_id  # e.g., "map"

            pose_msg.pose.position.x = msg.transform.translation.x
            pose_msg.pose.position.y = msg.transform.translation.y
            pose_msg.pose.position.z = msg.transform.translation.z

            pose_msg.pose.orientation.x = msg.transform.rotation.x
            pose_msg.pose.orientation.y = msg.transform.rotation.y
            pose_msg.pose.orientation.z = msg.transform.rotation.z
            pose_msg.pose.orientation.w = msg.transform.rotation.w

            self.pose_publisher.publish(pose_msg)
            self.get_logger().info("Published PoseStamped to /goal_pose")
            self.pose_published = True  # Avoid republishing

def main(args=None):
    rclpy.init(args=args)
    node = PosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()