import numpy as np
import math
from typing import Optional

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
from rclpy.node import Node

from sensor_interface.sensor_interface.lidar import LIDAR
from sensor_interface.sensor_interface.depth_camera import DepthCamera
from sensor_interface.sensor_interface.mipi_camera import MIPICamera


class SLAM(Node):
    def __init__(self, lidar: LIDAR):
        super().__init__('slam_node')

        self.lidar: LIDAR = lidar
        self.current_map: Optional[OccupancyGrid] = None

        # Register the SLAM's update_map method as the callback for LiDAR updates
        self.lidar.set_data_callback(self.update_map)

        # OccupancyGrid publisher
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map', 10)

    def update_map(self) -> None:
        """Generate and publish an OccupancyGrid map based on the latest LiDAR data."""
        if self.lidar.ranges is None:
            self.get_logger().warn("No LiDAR data available to generate the map.")
            return

        # Define map parameters
        map_size: tuple[int, int] = (100, 100)  # Map is map_size x map_size
        resolution: float = 0.1  # 0.1 meters per grid cell
        map_origin: tuple[float, float] = (-5.0, -5.0)  # Lower-left corner of the map

        # Create an empty grid (unknown = -1)
        occupancy_grid: np.ndarray = np.full(map_size, -1, dtype=int)
        occupancy_grid = self._generate_map(occupancy_grid)

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
        self.get_logger().info("Published updated map.")

    def _generate_map(self, occupancy_grid: np.ndarray) -> np.ndarray:
        """Use the SLAM algorithm to create an updated Occupancy Grid."""
        # TODO: implement SLAM algorithm
        return occupancy_grid
