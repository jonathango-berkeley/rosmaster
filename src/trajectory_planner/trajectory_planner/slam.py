import numpy as np

from sensor_interface.sensor_interface.lidar import LIDAR
from sensor_interface.sensor_interface.depth_camera import DepthCamera
from sensor_interface.sensor_interface.mipi_camera import MIPICamera

class Map:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=np.int8)


class SLAM:
    def __init__(self, lidar: LIDAR, depth_cam: DepthCamera, mipi_cam: MIPICamera):
        self.lidar = lidar
        self.depth_cam = depth_cam
        self.mipi_cam = mipi_cam

        self.map = Map(100,100)

    def generate_map(self):
        pass

    def get_map(self):
        return self.map
