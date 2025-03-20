#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation as R  # Using SciPy for quaternion conversion

def drawAxisCustom(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    """
    Draws a 3D coordinate axis on the given image using the provided camera calibration parameters,
    rotation vector, and translation vector.
    """
    axis = np.float32([[0, 0, 0],
                       [length, 0, 0],
                       [0, length, 0],
                       [0, 0, length]]).reshape(-1, 3)
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = np.int32(imgpts).reshape(-1, 2)
    origin = tuple(imgpts[0])
    img = cv2.line(img, origin, tuple(imgpts[1]), (0, 0, 255), 3)  # x-axis
    img = cv2.line(img, origin, tuple(imgpts[2]), (0, 255, 0), 3)  # y-axis
    img = cv2.line(img, origin, tuple(imgpts[3]), (255, 0, 0), 3)  # z-axis
    return img

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        
        # Subscribe to the camera image topic
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)
        
        self.bridge = CvBridge()
        
        # Publisher for PoseStamped messages
        self.pose_pub = self.create_publisher(PoseStamped, 'aruco/pose', 10)
        
        # Publisher for Boolean message indicating marker detection
        self.is_human_pub = self.create_publisher(Bool, '/is_human', 10)

        # ArUco Dictionary & Parameters
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        
        # Define marker side length (in meters)
        self.marker_length = 0.05
        
        # Camera calibration parameters (adjust these if you have calibration data)
        self.camera_matrix = np.array([[2640.2, 0, 1640],
                                       [0, 2640.2, 1232],
                                       [0, 0, 1]], dtype=np.float64)
        
        # Assuming minimal distortion; update with your calibration data if available
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            
            # Detect ArUco markers in the grayscale image
            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
            
            # Create Boolean message indicating marker detection
            is_human_msg = Bool()
            is_human_msg.data = ids is not None
            self.is_human_pub.publish(is_human_msg)

            if ids is not None:
                self.get_logger().info("ArUco Marker detected!")
                # Estimate pose of each detected marker
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)
                
                # Draw detected markers on the image
                cv_image = aruco.drawDetectedMarkers(cv_image, corners, ids)
                
                # Loop over each marker to draw its pose axis and publish PoseStamped
                for i in range(len(ids)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]
                    cv_image = drawAxisCustom(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, self.marker_length * 0.5)
                    
                    # Convert rotation vector to rotation matrix
                    rmat, _ = cv2.Rodrigues(rvec)
                    
                    # Convert rotation matrix to quaternion
                    quat = R.from_matrix(rmat).as_quat()
                    
                    # Create and populate PoseStamped message
                    pose_msg = PoseStamped()
                    pose_msg.header.stamp = self.get_clock().now().to_msg()
                    pose_msg.header.frame_id = "camera_link"  # Change if needed
                    pose_msg.pose.position.x = float(tvec[0][0])
                    pose_msg.pose.position.y = float(tvec[0][1])
                    pose_msg.pose.position.z = float(tvec[0][2])
                    pose_msg.pose.orientation.x = float(quat[0])
                    pose_msg.pose.orientation.y = float(quat[1])
                    pose_msg.pose.orientation.z = float(quat[2])
                    pose_msg.pose.orientation.w = float(quat[3])
                    
                    # Publish pose
                    self.pose_pub.publish(pose_msg)
                    
                    self.get_logger().info(f"Marker ID {ids[i][0]}: rvec = {rvec.flatten()}, tvec = {tvec.flatten()}")

            # Show the resulting image with markers and pose axes
            cv2.imshow("Aruco Detection with Pose", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()