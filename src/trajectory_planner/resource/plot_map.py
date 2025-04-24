#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

import matplotlib.pyplot as plt
import numpy as np

class MapPlotter(Node):
    def __init__(self):
        super().__init__('map_plotter')
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.pose_sub = self.create_subscription(PoseStamped, '/goal_pose', self.pose_callback, 10)

        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.waypoints = []

        self.fig, self.ax = plt.subplots()

        self.get_logger().info("Map plotter node started!")

    def map_callback(self, msg):
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x, msg.info.origin.position.y)
        width = msg.info.width
        height = msg.info.height

        data = np.array(msg.data, dtype=np.int8).reshape((height, width))
        visual_map = np.full((height, width), 128, dtype=np.uint8)
        visual_map[data == 0] = 255
        visual_map[data == 100] = 0
        self.map_data = visual_map

        self.update_plot()

    def pose_callback(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.waypoints.append((x, y))
        self.update_plot()

    def update_plot(self):
        if self.map_data is None:
            return

        self.ax.clear()
        """
        height, width = self.map_data.shape
        self.ax.imshow(self.map_data, cmap='gray', origin='lower', extent=[
            self.map_origin[0],
            self.map_origin[0] + width * self.map_resolution,
            self.map_origin[1],
            self.map_origin[1] + height * self.map_resolution
        ])
        """

        if self.waypoints:
            xs, ys = zip(*self.waypoints)
            self.ax.plot(xs, ys, 'bo-', label='Waypoints')
            self.ax.plot(xs[-1], ys[-1], 'ro', label='Current')
            self.ax.legend()

        self.ax.set_title("Live Map and Waypoints")
        self.ax.set_xlabel("x [m]")
        self.ax.set_ylabel("y [m]")
        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = MapPlotter()
    try:
        # Start interactive mode
        plt.ion()
        
        # Keep spinning and updating the plot
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        # Ensure plt.show() runs in the main thread at the end of spin

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

