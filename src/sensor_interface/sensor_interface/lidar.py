import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from typing import Optional, List, Callable


class LIDAR(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')

        # LiDAR data attributes
        self.ranges: Optional[List[float]] = None
        self.angle_min: Optional[float] = None
        self.angle_max: Optional[float] = None
        self.angle_increment: Optional[float] = None
        self.range_min: Optional[float] = None
        self.range_max: Optional[float] = None

        # Callback to notify when new data is received
        self.data_callback: Optional[Callable] = None

        # Subscribe to the /scan topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.get_logger().info("LIDAR node initialized and subscribed to /scan.")

    def set_data_callback(self, callback: Callable):
        """Set a callback to be triggered when new LiDAR data is received."""
        self.data_callback = callback
        self.get_logger().info("Data callback set.")

    def listener_callback(self, msg: LaserScan):
        """Handle incoming LiDAR data from the /scan topic."""
        self.get_logger().info("Received LaserScan message.")

        # Update LiDAR data
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Log key LiDAR parameters for debugging
        self.get_logger().debug(
            f"Updated LiDAR data: angle_min={self.angle_min}, "
            f"angle_max={self.angle_max}, angle_increment={self.angle_increment}, "
            f"range_min={self.range_min}, range_max={self.range_max}, "
            f"number of ranges={len(self.ranges) if self.ranges else 0}."
        )

        # Trigger the callback if set
        if self.data_callback:
            self.get_logger().info("LIDAR: Triggering the data callback.")
            self.data_callback()
        else:
            self.get_logger().info("LIDAR: No data callback is set.")



def main():
    rclpy.init()

    # Create and spin the LIDAR node
    lidar = LIDAR()

    try:
        rclpy.spin(lidar)
    except KeyboardInterrupt:
        lidar.get_logger().info("LIDAR node shutting down.")
    finally:
        lidar.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
