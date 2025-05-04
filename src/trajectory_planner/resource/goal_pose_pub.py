#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from builtin_interfaces.msg import Time

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(1.0, self.publish_goal)
        self.goal_sent = False

    def publish_goal(self):
        if self.goal_sent:
            return  # Only send once

        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()

        # Goal position (1 meter in front of origin)
        goal.pose.position.x = 0.0
        goal.pose.position.y = -1.0
        goal.pose.position.z = 0.0

        # Goal orientation (facing straight ahead)
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.publisher_.publish(goal)
        self.get_logger().info("Published goal pose.")
        self.goal_sent = True  # only publish once

def main(args=None):
    rclpy.init(args=args)
    node = GoalPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()