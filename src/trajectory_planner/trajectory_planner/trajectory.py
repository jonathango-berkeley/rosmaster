import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

import sys
import os

sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import a_star_Gutsav_v2 as a_star
import numpy as np


class Trajectory(Node):
    def __init__(self):
        super().__init__('trajectory_planner')

        # Subscriber to the /map topic
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscriber to the /goal_pose topic
        self.goal_subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        # Publisher for the /trajectory topic
        self.trajectory_publisher = self.create_publisher(
            Path,
            '/trajectory',
            10
        )

        # Internal storage for map and goal
        self.current_map = None
        self.goal_pose = None

        self.get_logger().info("TrajectoryPlanner node initialized.")

    def map_callback(self, map_msg: OccupancyGrid):
        """
        Callback function for the /map topic. Stores the received map
        and triggers trajectory planning if a goal is available.

        :param map_msg: The OccupancyGrid message from the /map topic.
        """
        self.get_logger().info("Received a map message.")
        self.current_map = map_msg

        if self.goal_pose:
            self.plan_and_publish_trajectory()

    def goal_callback(self, goal_msg: PoseStamped):
        """
        Callback function for the /goal_pose topic. Stores the received goal
        and triggers trajectory planning if a map is available.

        :param goal_msg: The PoseStamped message from the /goal_pose topic.
        """
        self.get_logger().info("Received a goal pose.")
        self.goal_pose = goal_msg

        if self.current_map:
            self.plan_and_publish_trajectory()

    def plan_and_publish_trajectory(self):
        """
        Plans a trajectory based on the current map and goal, and publishes it.
        """
        if not self.current_map or not self.goal_pose:
            self.get_logger().warn("Map or goal pose is missing. Cannot plan trajectory.")
            return

        # Generate the trajectory
        trajectory = self.generate_trajectory(self.current_map, self.goal_pose)

        # Publish the trajectory
        self.trajectory_publisher.publish(trajectory)
        self.get_logger().info("Published trajectory.")

    def generate_trajectory(self, map_msg: OccupancyGrid, goal_pose: PoseStamped) -> Path:
        """
        Generates a trajectory based on the input map and goal.

        :param map_msg: The OccupancyGrid message.
        :param goal_pose: The PoseStamped message representing the goal.
        :return: A nav_msgs/Path message.
        """
        path = Path()
        path.header = map_msg.header

        # TODO: Replace with actual algorithm to generate the path
        a_star.plot(map_msg, np.array([1,1]), np.array([98,98]))

        # Add the goal as the last waypoint for demonstration
        goal_waypoint = PoseStamped()
        goal_waypoint.header = goal_pose.header
        goal_waypoint.pose = goal_pose.pose
        path.poses.append(goal_waypoint)

        return path


def main(args=None):
    rclpy.init(args=args)
    node = Trajectory()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TrajectoryPlanner node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
