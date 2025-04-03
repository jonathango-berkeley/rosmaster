#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import numpy as np
from scipy.spatial.transform import Rotation as R
from collections import deque

class TransformFilter(Node):
    def __init__(self):
        super().__init__('transform_filter')

        self.subscription = self.create_subscription(
            TransformStamped,
            'aruco/transform',
            self.listener_callback,
            10)

        self.publisher_ = self.create_publisher(TransformStamped, 'aruco/filtered_transform', 10)

        # Rolling buffer for transforms
        self.buffer = deque(maxlen=10)

    def listener_callback(self, msg):
        # Store latest transform in the buffer
        self.buffer.append(msg)

        # Only average and publish if the buffer is full
        if len(self.buffer) == self.buffer.maxlen:
            avg_transform = self.average_transforms(list(self.buffer))
            self.publisher_.publish(avg_transform)
            self.get_logger().info("Published filtered transform.")
            self.buffer.clear()  # Clear buffer after publishing

    def average_transforms(self, transforms):
        positions = []
        quaternions = []

        for msg in transforms:
            t = msg.transform.translation
            q = msg.transform.rotation
            positions.append([t.x, t.y, t.z])
            quaternions.append([q.x, q.y, q.z, q.w])

        # Average position
        avg_pos = np.mean(positions, axis=0)

        # Average quaternion using scipy Rotation
        avg_rot = R.from_quat(quaternions).mean().as_quat()

        # Use the last transform's header and child_frame_id
        last_msg = transforms[-1]
        filtered_msg = TransformStamped()
        filtered_msg.header.stamp = last_msg.header.stamp
        filtered_msg.header.frame_id = last_msg.header.frame_id
        filtered_msg.child_frame_id = last_msg.child_frame_id

        filtered_msg.transform.translation.x = avg_pos[0]
        filtered_msg.transform.translation.y = avg_pos[1]
        filtered_msg.transform.translation.z = avg_pos[2]
        filtered_msg.transform.rotation.x = avg_rot[0]
        filtered_msg.transform.rotation.y = avg_rot[1]
        filtered_msg.transform.rotation.z = avg_rot[2]
        filtered_msg.transform.rotation.w = avg_rot[3]

        return filtered_msg

def main(args=None):
    rclpy.init(args=args)
    node = TransformFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()