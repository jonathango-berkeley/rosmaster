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
from scipy.spatial.transform import Rotation as R  # Using SciPy for quaternion conversion
import tf2_ros


def drawAxisCustom(img, camera_matrix, dist_coeffs, rvec, tvec, length=0.05):
    axis = np.float32([[0, 0, 0],
                       [length, 0, 0],
                       [0, length, 0],
                       [0, 0, length]]).reshape(-1, 3)
    imgpts, _ = cv2.projectPoints(axis, rvec, tvec, camera_matrix, dist_coeffs)
    imgpts = np.int32(imgpts).reshape(-1, 2)
    origin = tuple(imgpts[0])
    img = cv2.line(img, origin, tuple(imgpts[1]), (0, 0, 255), 3)
    img = cv2.line(img, origin, tuple(imgpts[2]), (0, 255, 0), 3)
    img = cv2.line(img, origin, tuple(imgpts[3]), (255, 0, 0), 3)
    return img


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

        self.marker_length = 0.05  # meters

        self.camera_matrix = np.array([[2640.2, 0, 1640],
                                       [0, 2640.2, 1232],
                                       [0, 0, 1]], dtype=np.float64)

        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        # TF Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

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
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, self.marker_length, self.camera_matrix, self.dist_coeffs)

                for i in range(len(ids)):
                    rvec = rvecs[i]
                    tvec = tvecs[i]

                    rmat, _ = cv2.Rodrigues(rvec)
                    quat = R.from_matrix(rmat).as_quat()

                    # Transform to map frame
                    try:
                        transform_map_to_camera = self.tf_buffer.lookup_transform("map", "camera_link", rclpy.time.Time())

                        # Extract translation and rotation from transform
                        t = transform_map_to_camera.transform.translation
                        r = transform_map_to_camera.transform.rotation

                        # Apply transformation to position
                        tvec_transformed = np.array([tvec[0][0] + t.x, tvec[0][1] + t.y, tvec[0][2] + t.z])

                        # Apply quaternion multiplication manually
                        q1 = R.from_quat([r.x, r.y, r.z, r.w])  # Map -> Camera
                        q2 = R.from_quat([quat[0], quat[1], quat[2], quat[3]])  # Camera -> Object
                        q_result = q1 * q2  # Equivalent to quaternion multiplication

                        transformed_msg = TransformStamped()
                        transformed_msg.header.stamp = self.get_clock().now().to_msg()
                        transformed_msg.header.frame_id = "map"
                        transformed_msg.child_frame_id = f"aruco_marker_{ids[i][0]}"
                        transformed_msg.transform.translation.x = tvec_transformed[0]
                        transformed_msg.transform.translation.y = tvec_transformed[1]
                        transformed_msg.transform.translation.z = tvec_transformed[2]
                        transformed_msg.transform.rotation.x = q_result.as_quat()[0]
                        transformed_msg.transform.rotation.y = q_result.as_quat()[1]
                        transformed_msg.transform.rotation.z = q_result.as_quat()[2]
                        transformed_msg.transform.rotation.w = q_result.as_quat()[3]

                        self.transform_pub.publish(transformed_msg)
                        self.tf_broadcaster.sendTransform(transformed_msg)

                        self.get_logger().info(f"Publishing Transform to aruco/transform: {transformed_msg.child_frame_id}")
                        self.get_logger().info(f"Translation: x={transformed_msg.transform.translation.x}, y={transformed_msg.transform.translation.y}, z={transformed_msg.transform.translation.z}")
                        self.get_logger().info(f"Rotation: x={transformed_msg.transform.rotation.x}, y={transformed_msg.transform.rotation.y}, z={transformed_msg.transform.rotation.z}, w={transformed_msg.transform.rotation.w}")

                        self.get_logger().info("Transform successfully published!")
                        self.get_logger().info(f"Published Transform: {transformed_msg.header.frame_id}")

                    except tf2_ros.LookupException:
                        self.get_logger().warn("TF Lookup failed: map -> camera_link")
                    except tf2_ros.ConnectivityException:
                        self.get_logger().warn("TF Connectivity issue")
                    except tf2_ros.ExtrapolationException:
                        self.get_logger().warn("TF Extrapolation issue")

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