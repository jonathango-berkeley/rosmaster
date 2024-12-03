import numpy as np
import math
from typing import Optional

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.node import Node
import rclpy

from sensor_interface.lidar import LIDAR
# from sensor_interface.sensor_interface.depth_camera import DepthCamera
# from sensor_interface.sensor_interface.mipi_camera import MIPICamera


class SLAM(Node):
    def __init__(self, lidar: LIDAR):
        super().__init__('slam_node')

        self.lidar: LIDAR = lidar
        self.current_map: Optional[OccupancyGrid] = None

        # Register the SLAM's update_map method as the callback for LiDAR updates
        self.lidar.set_data_callback(self.update_map)

        # OccupancyGrid publisher
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)
        self.get_logger().info("SLAM node initialized and '/map' publisher created.")

    def update_map(self) -> None:
        """Generate and publish an OccupancyGrid map based on the latest LiDAR data."""
        self.get_logger().info("SLAM: update_map triggered.")
        
        if self.lidar.ranges is None:
            self.get_logger().warn("No LiDAR data available to generate the map.")
            return

        self.get_logger().info("Updating map based on new LiDAR data.")

        # Define map parameters
        map_size: tuple[int, int] = (100, 100)  # Map is map_size x map_size
        resolution: float = 0.1  # 0.1 meters per grid cell
        map_origin: tuple[float, float] = (-5.0, -5.0)  # Lower-left corner of the map

        # Create an empty grid (unknown = -1)
        occupancy_grid: np.ndarray = np.full(map_size, -1, dtype=int)

        # Debug: Log map initialization
        self.get_logger().debug(f"Initial occupancy grid created with size: {map_size}")

        # Generate the map
        occupancy_grid = self._generate_map(occupancy_grid)

        # Debug: Log map update
        self.get_logger().debug(f"Updated occupancy grid generated.")

        # Create the OccupancyGrid message
        occupancy_msg: OccupancyGrid = OccupancyGrid()
        occupancy_msg.header.frame_id = "map"
        occupancy_msg.info.resolution = resolution
        occupancy_msg.info.width = map_size[0]
        occupancy_msg.info.height = map_size[1]
        occupancy_msg.info.origin = Pose()
        occupancy_msg.info.origin.position.x = map_origin[0]
        occupancy_msg.info.origin.position.y = map_origin[1]
        occupancy_msg.data = occupancy_grid.flatten().tolist()

        # Store the generated map
        self.current_map = occupancy_msg

        # Publish the map
        self.map_publisher.publish(self.current_map)
        self.get_logger().info("Published updated map to '/map'.")

    def _generate_map(self, occupancy_grid: np.ndarray) -> np.ndarray:
        """Use the SLAM algorithm to create an updated Occupancy Grid."""
        # Debug: Placeholder SLAM generation
        self.get_logger().debug("Generating map with placeholder SLAM algorithm.")

        # TODO: Implement SLAM algorithm here
        occupancy_grid[50, 50] = 100  # Example: mark a single cell as occupied for testing
        self.get_logger().debug("Marked cell (50, 50) as occupied for debugging purposes.")
        return occupancy_grid


def main():
    rclpy.init()

    # Initialize the LIDAR and SLAM system
    lidar = LIDAR()
    slam_node = SLAM(lidar)

    try:
        slam_node.get_logger().info("SLAM node is running. Waiting for LiDAR updates...")
        rclpy.spin(slam_node)
    except KeyboardInterrupt:
        slam_node.get_logger().info("Shutting down SLAM node.")
    finally:
        lidar.destroy_node()
        slam_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
