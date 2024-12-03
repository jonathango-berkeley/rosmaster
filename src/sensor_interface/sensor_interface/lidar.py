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

    def set_data_callback(self, callback: Callable):
        """Set a callback to be triggered when new LiDAR data is received."""
        self.data_callback = callback

    def listener_callback(self, msg: LaserScan):
        # Update LiDAR data
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_max = msg.angle_max
        self.angle_increment = msg.angle_increment
        self.range_min = msg.range_min
        self.range_max = msg.range_max

        # Trigger the callback if set
        if self.data_callback:
            self.data_callback()
