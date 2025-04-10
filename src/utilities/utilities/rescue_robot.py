#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

import numpy as np
from scipy.spatial.transform import Rotation as R

from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import OccupancyGrid

import sys
import signal
import Hobot.GPIO as GPIO

import math

def clean_exit(signal, frame):
    sys.exit(0)

class RescueRobot:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('rescue_robot_main')
        self.node.get_logger().info("RescueRobot initialization started.")
        super().__init__('rescue_robot_main')
        # Robot state
        self.rescue_mode = False

        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)

        # Map
        self.map_data = None
        self.map_received = False  # <- log-once flag

        # Aruco Data
        self.aruco_sub = self.node.create_subscription(
            TransformStamped,
            '/aruco/transform',
            self.aruco_callback,
            10
        )
        self.aruco_queue = {}
        self.aruco_saved = []

        # Subscriptions
        self.map_sub = self.node.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Publisher
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )
        # HZ Subscribe to the /goal_pose topic to receive target pose messages
        self.goal_pose_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_pose_callback,
            10
        )
        # HZ Store the goal pose
        self.goal_pose = None
        # Setup Magnet
        self.PIN = 32
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.PIN, GPIO.OUT, initial=GPIO.LOW)

        self.node.get_logger().info("RescueRobot ready.")

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = msg
        if not self.map_received:
            self.node.get_logger().info("Map received and updated.")
            self.map_received = True

    def aruco_callback(self, msg: TransformStamped):
        if msg.child_frame_id in self.aruco_saved:
            return

        found_location = self.get_position()
        if msg.child_frame_id in self.aruco_queue.keys:
            self.aruco_queue[msg.child_frame_id]["found_location"] = found_location
            if len(self.aruco_queue[msg.child_frame_id]["last_10"]) > 10:
                self.aruco_queue[msg.child_frame_id]["last_10"].pop(0)

            self.aruco_queue[msg.child_frame_id]["last_10"].append(msg)
            self.aruco_queue[msg.child_frame_id]["location"] = self.filter_location(self.aruco_queue[msg.child_frame_id]["last_10"])
        else:
            self.aruco_queue[msg.child_frame_id] = {
                "location": msg,
                "found_location": found_location,
                "last_10": [msg]
            }
    
    def remove_object(self, child_frame_id):
        try:
            self.aruco_queue.pop(child_frame_id)
            self.aruco_saved.append(child_frame_id)
        except KeyError:
            self.node.get_logger().error(f"Key {child_frame_id} not found")
    def goal_pose_callback(self, msg: PoseStamped):

        # HZ Callback function: When receiving the /goal_pose message, stores the target pose in self.goal_pose.

        self.goal_pose = msg
        self.get_logger().info("Received goal_pose update")
    def get_position(self):
        try:
            now = rclpy.time.Time()
            trans: TransformStamped = self.tf_buffer.lookup_transform(
                "map", 'base_link', now, timeout=rclpy.duration.Duration(seconds=1.0)
            )

            return trans

        except Exception as e:
            self.node.get_logger().warn(f"TF lookup failed: {e}")

    def filter_location(self, transforms):
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
    
    def run_robot(self, pose):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = pose.header.stamp
        pose_msg.header.frame_id = pose.header.frame_id  # e.g., "map"

        pose_msg.pose.position.x = pose.transform.translation.x
        pose_msg.pose.position.y = pose.transform.translation.y
        pose_msg.pose.position.z = pose.transform.translation.z

        pose_msg.pose.orientation.x = pose.transform.rotation.x
        pose_msg.pose.orientation.y = pose.transform.rotation.y
        pose_msg.pose.orientation.z = pose.transform.rotation.z
        pose_msg.pose.orientation.w = pose.transform.rotation.w

        self.pose_publisher.publish(pose_msg)
        self.get_logger().info("Published PoseStamped to /goal_pose")

    def switch_magnet(self, state):
        if state:
            GPIO.output(self.PIN, GPIO.HIGH)
            self.get_logger().info("Set Magnet to ON")
        else:
            GPIO.output(self.PIN, GPIO.LOW)
            self.get_logger().info("Set Magnet to OFF")


    def is_arrived(self):
        position_threshold = 0.1
        orientation_threshold = math.radians(5.0)
        if self.goal_pose is None:
            self.get_logger().warn("goal pose has not been received")
            return False

        # Gets the current robot pose
        current_tf = self.get_position()
        if current_tf is None:
            self.get_logger().warn("Failed to get current pose")
            return False

        # Calculate the Euclidean distance between the current pose and the target pose
        dx = current_tf.transform.translation.x - self.goal_pose.pose.position.x
        dy = current_tf.transform.translation.y - self.goal_pose.pose.position.y
        distance = math.sqrt(dx**2 + dy**2)
        self.get_logger().info(f"Current distance from goal: {distance:.3f}")
        # Current orientation quaternion
        q_current = [
            current_tf.transform.rotation.x,
            current_tf.transform.rotation.y,
            current_tf.transform.rotation.z,
            current_tf.transform.rotation.w
        ]
        # Goal orientation quaternion
        q_goal = [
            self.goal_pose.pose.orientation.x,
            self.goal_pose.pose.orientation.y,
            self.goal_pose.pose.orientation.z,
            self.goal_pose.pose.orientation.w
        ]
        # Wrap the current and goal quaternion into a Rotation object
        r_current = R.from_quat(q_current)
        r_goal = R.from_quat(q_goal)

        # The relative rotation is computed by the inverse of the current orientation multiplied by the target orientation.
        r_diff = r_current.inv() * r_goal
        # 'magnitude()' returns the rotation angle in radians for the relative rotation 'r_diff'.
        angle_diff = r_diff.magnitude()
        
        self.get_logger().info(f"Orientation difference from goal: {math.degrees(angle_diff):.2f} degrees")
        
        if distance <= position_threshold and angle_diff <= orientation_threshold:
            self.get_logger().info("Arrived at target position and orientation.")
            # wait 2 seconds after arrival
            time.sleep(2)
            return True

        return False
  

    def search_and_rescue(self):
        pass

    def spin(self):
        self.node.get_logger().info("Robot is running...")
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            self.node.get_logger().info("Shutting down robot...")
        finally:
            self.shutdown()

    def shutdown(self):
        GPIO.cleanup()
        self.node.get_logger().info("Cleaning up resources.")
        self.node.destroy_node()
        rclpy.shutdown()

def main():
    signal.signal(signal.SIGINT, clean_exit)
    robot = RescueRobot()
    robot.spin()

if __name__ == '__main__':
    main()
