#!/usr/bin/env python3
import os
os.environ["DISPLAY"] = ":0"

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import cv2.aruco as aruco
import numpy as np
from scipy.spatial.transform import Rotation as R
import tf2_ros

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('detect_object')

        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            10)

        self.bridge = CvBridge()
        self.transform_pub = self.create_publisher(TransformStamped, 'aruco/transform', 10)
        self.object_detected_pub = self.create_publisher(Bool, '/object_detected', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters()
        self.marker_length = 0.032  # meters

        self.camera_matrix = np.array([[526, 0, 320],
                                       [0, 526, 240],
                                       [0, 0, 1]], dtype=np.float64)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
<<<<<<< HEAD
        
        self.T_opencv_to_ros = R.from_quat([0.5, -0.5, -0.5, 0.5])
        
=======

        # Correct rotation from OpenCV to ROS REP-103
        self.T_opencv_to_ros = R.from_quat([0.5, -0.5, -0.5, 0.5])

>>>>>>> 7a31b1a (Update detect_object.py)
    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

            object_detected_msg = Bool()
            object_detected_msg.data = ids is not None
            self.object_detected_pub.publish(object_detected_msg)

            if ids is not None:
                self.get_logger().info("ArUco Marker detected!")
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]

<<<<<<< HEAD
                    # Convert rotation to matrix and adjust from OpenCV to ROS camera frame
                    rmat, _ = cv2.Rodrigues(rvec)
                    r_opencv = R.from_matrix(rmat)
<<<<<<< HEAD
                    T_opencv_to_ros_base = R.from_quat([0.5, -0.5, 0.5, -0.5])
                    R_flip_xz = R.from_euler('xyz', [-90,90,0], degrees=True)

                    T_opencv_to_ros = R_flip_xz * T_opencv_to_ros_base

                    r_ros = T_opencv_to_ros * r_opencv
                    quat = r_ros.as_quat()

                    tvec_rotated = T_opencv_to_ros.apply(tvec[0])
=======
                    r_ros = T_opencv_to_ros * r_opencv
                    quat = r_ros.as_quat()
                    
                    tvec_ros = self.T_opencv_to_ros.apply(tvec[0])
>>>>>>> 77c9716 (try to fix axes)
=======
                    # Convert OpenCV rotation to quaternion in ROS frame
                    rmat, _ = cv2.Rodrigues(rvec)
                    r_opencv = R.from_matrix(rmat)
                    r_ros = self.T_opencv_to_ros * r_opencv
                    quat = r_ros.as_quat()

                    # Rotate translation vector to ROS camera frame
                    tvec_ros = self.T_opencv_to_ros.apply(tvec[0])

                    # Create TransformStamped message
                    transform_msg = TransformStamped()
                    transform_msg.header.stamp = self.get_clock().now().to_msg()
                    transform_msg.header.frame_id = "camera_link"
                    transform_msg.child_frame_id = f"aruco_marker_{ids[i][0]}"
                    transform_msg.transform.translation.x = x_ros
                    transform_msg.transform.translation.y = y_ros
                    transform_msg.transform.translation.z = z_ros
                    transform_msg.transform.rotation.x = float(quat[0])
                    transform_msg.transform.rotation.y = float(quat[1])
                    transform_msg.transform.rotation.z = float(quat[2])
                    transform_msg.transform.rotation.w = float(quat[3])

                    self.transform_pub.publish(transform_msg)
                    self.tf_broadcaster.sendTransform(transform_msg)
<<<<<<< HEAD
                    self.get_logger().info(f"Published camera_link → aruco_marker_{ids[i][0]}")
=======

                    self.get_logger().info(f"Published TF: camera_link → aruco_marker_{ids[i][0]}")

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
