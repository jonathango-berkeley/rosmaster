#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from SunriseRobotLib import Mipi_Camera

class MipiCameraPublisher(Node):
    def __init__(self):
        super().__init__('camera')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        
        # Initialize the MIPI Camera
        self.camera = Mipi_Camera(width=1280, height=720, debug=False)
        if self.camera.isOpened():
            self.get_logger().info("Mipi Camera initialized successfully.")
        else:
            self.get_logger().error("Failed to initialize Mipi Camera.")
            raise RuntimeError("Camera initialization failed.")

        self.bridge = CvBridge()
        
        # Publish at ~30 FPS
        self.timer = self.create_timer( 1/30.0 , self.timer_callback)

    def timer_callback(self):
        ret, frame = self.camera.get_frame()
        if frame is not None:
            try:
                # Convert to ROS Image and publish
                msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                self.publisher_.publish(msg)
                self.get_logger().debug("Published a camera image.")
            except Exception as e:
                self.get_logger().error(f"Error publishing frame: {e}")
        else:
            self.get_logger().error("Failed to capture image.")

    def destroy_node(self):
        # Release the camera resource when the node is destroyed
        self.camera.release()
        self.get_logger().info("Mipi Camera released.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
