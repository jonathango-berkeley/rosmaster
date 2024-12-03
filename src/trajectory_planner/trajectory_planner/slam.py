import rclpy
from rclpy.node import Node
import numpy as np
from math import cos, sin, pi
from typing import Optional

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose


class SLAM(Node):
    def __init__(self):
        super().__init__('slam_node')

        self.current_map: Optional[OccupancyGrid] = None

        # Subscribe to /scan topic
        self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        # Publisher for OccupancyGrid
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.get_logger().info("SLAM node initialized, subscribed to '/scan', and publishing to '/map'.")

    def lidar_callback(self, msg: LaserScan) -> None:
        """Callback to process LaserScan data and generate an OccupancyGrid map."""
        if not msg.ranges:
            self.get_logger().warn("No data in LaserScan message.")
            return

        # Define map parameters
        map_size: tuple[int, int] = (100, 100)
        resolution: float = 0.1  # 0.1 meters per grid cell
        map_origin: tuple[float, float] = (-5.0, -5.0)

        # Create an empty grid (unknown = -1)
        occupancy_grid: np.ndarray = np.full(map_size, -1, dtype=int)

        # Process LaserScan data into occupancy grid
        occupancy_grid = self._generate_map(occupancy_grid, msg)

        # Create the OccupancyGrid message
        occupancy_msg: OccupancyGrid = OccupancyGrid()
        occupancy_msg.header.frame_id = "map"
        occupancy_msg.header.stamp = self.get_clock().now().to_msg()
        occupancy_msg.info.resolution = resolution
        occupancy_msg.info.width = map_size[0]
        occupancy_msg.info.height = map_size[1]
        occupancy_msg.info.origin = Pose()
        occupancy_msg.info.origin.position.x = map_origin[0]
        occupancy_msg.info.origin.position.y = map_origin[1]
        occupancy_msg.data = occupancy_grid.flatten().tolist()

        # Store and publish the generated map
        self.current_map = occupancy_msg
        self.map_publisher.publish(self.current_map)
        self.get_logger().info("Published updated map to '/map'.")

    def _generate_map(self, occupancy_grid: np.ndarray, scan: LaserScan) -> np.ndarray:
        """Convert LaserScan data into an occupancy grid."""

        return occupancy_grid


def main():
    rclpy.init()

    # Initialize the SLAM node
    slam_node = SLAM()

    try:
        slam_node.get_logger().info("SLAM node is running. Subscribed to '/scan'.")
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.get_logger().info("Shutting down SLAM node.")
    finally:
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
