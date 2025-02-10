import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from SunriseRobotLib import Mipi_Camera

class MipiCameraNode(Node):
    def __init__(self):
        super().__init__('mipi_camera_node')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize the Mipi Camera
        self.camera = Mipi_Camera(width=320, height=240, debug=False)
        if self.camera.isOpened():
            self.get_logger().info("Mipi Camera initialized successfully.")
        else:
            self.get_logger().error("Failed to initialize Mipi Camera.")
            raise RuntimeError("Camera initialization failed.")

        # Set up a timer to publish frames
        self.timer = self.create_timer(0.033, self.publish_frame)  # Approx. 30 FPS

    def publish_frame(self):
        ret, frame = self.camera.get_frame()
        if frame is not None:
            try:
                # Convert the frame to a ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.publisher_.publish(ros_image)
                # self.get_logger().info("Published a frame.")
            except Exception as e:
                self.get_logger().error(f"Error publishing frame: {e}")
        else:
            self.get_logger().warning("Failed to capture frame.")

    def __del__(self):
        # Clean up the camera when the node is destroyed
        self.camera.release()
        self.get_logger().info("Mipi Camera released.")

def main(args=None):
    rclpy.init(args=args)
    node = MipiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
