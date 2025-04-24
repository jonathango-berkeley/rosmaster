#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from geometry_msgs.msg import Pose
import json
import os

filename = os.path.join(os.path.dirname(__file__), 'occupancy_grid.json')

class MapPublisher(Node):
    def __init__(self):
        super().__init__('map_publisher')

        self.publisher_ = self.create_publisher(OccupancyGrid, '/map', 10)
        timer_period = 1.0  # Sekündlich
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Datei einmal beim Start laden
        self.grid = self.load_grid(filename)

    def load_grid(self, path):
        if not os.path.exists(path):
            self.get_logger().error(f'Datei nicht gefunden: {path}')
            return None

        with open(path, 'r') as f:
            data = json.load(f)

        # JSON-Felder parsen
        width = data["info"]["width"]
        height = data["info"]["height"]
        resolution = data["info"]["resolution"]
        origin_x = data["info"]["origin"]["position"]["x"]
        origin_y = data["info"]["origin"]["position"]["y"]
        orientation = data["info"]["origin"]["orientation"]["w"]
        grid_data = data["data"]

        if width is None or height is None or grid_data is None:
            self.get_logger().error('Fehlende Daten im JSON.')
            return None

        return {
            'width': width,
            'height': height,
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'orientation': orientation,
            'data': grid_data
        }

    def timer_callback(self):
        if self.grid is None:
            return

        msg = OccupancyGrid()

        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        msg.info.resolution = self.grid['resolution']
        msg.info.width = self.grid['width']
        msg.info.height = self.grid['height']

        # Ursprungsposition
        msg.info.origin = Pose()
        msg.info.origin.position.x = self.grid['origin_x']
        msg.info.origin.position.y = self.grid['origin_y']
        msg.info.origin.orientation.w = self.grid['orientation']

        msg.data = self.grid['data']

        self.publisher_.publish(msg)
        self.get_logger().info('OccupancyGrid veröffentlicht.')

def main(args=None):
    rclpy.init(args=args)
    node = MapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

