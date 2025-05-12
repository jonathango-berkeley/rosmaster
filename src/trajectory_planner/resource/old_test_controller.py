#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from std_msgs.msg import Bool
import math

class GoalFollower(Node):
    def __init__(self):
        super().__init__('goal_follower')

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.pose_sub = self.create_subscription(TransformStamped, '/current_position', self.pose_callback, 10)
        self.bool_publisher = self.create_publisher(Bool, '/next_waypoint', 10)

        self.timer = self.create_timer(0.1, self.control_loop)

        self.current_position = None  # Now a TransformStamped
        self.goal_pose = None
        self.state = 'idle'  # 'rotate', 'forward', 'align', 'done'

        self.linear_speed = 0.05
        #self.angular_speed = 0.1
        self.position_tolerance = 0.01
        #self.angle_tolerance = 0.1

        self.get_logger().info("Controller active.")

    def goal_callback(self, msg):
        self.goal_pose = msg
        self.state = 'y'
        self.get_logger().info("Received new goal.")

    def pose_callback(self, msg):
        self.current_position = msg
        self.get_logger().info("Received current_position.")  # Optional debug

    def control_loop(self):
        if self.current_position is None or self.goal_pose is None:
            return

        # Get current and goal positions
        x = self.current_position.transform.translation.x
        y = self.current_position.transform.translation.y
        gx = self.goal_pose.pose.position.x
        gy = self.goal_pose.pose.position.y

        # Get current yaw
        #q = self.current_position.transform.rotation
        #yaw = self.yaw_from_quaternion(q)

        # Compute goal direction
        dx = gx - x
        dy = gy - y

        twist = Twist()
        """
        distance = math.hypot(dx, dy)
        goal_theta = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(goal_theta - yaw)

        twist = Twist()

        if self.state == 'rotate':
            if abs(angle_diff) > self.angle_tolerance:
                twist.angular.z = self.angular_speed if angle_diff > 0 else -self.angular_speed
                self.get_logger().info(f"Outside tolerance.{angle_diff}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'forward'
                self.get_logger().info("Rotation done. Driving forward.")
        """
        
        """
        elif self.state == 'align':
            goal_q = self.goal_pose.pose.orientation
            goal_yaw = self.yaw_from_quaternion(goal_q)
            final_diff = self.normalize_angle(goal_yaw - yaw)

            if abs(final_diff) > self.angle_tolerance:
                twist.angular.z = self.angular_speed if final_diff > 0 else -self.angular_speed
                self.get_logger().info(f"Outside tolerance.{final_diff}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'done'
                self.get_logger().info("Goal reached and orientation aligned.")
        """
        if self.state == 'y':
            if abs(dy) > self.position_tolerance:
                twist.linear.y = self.linear_speed if dy > 0 else -self.linear_speed
                self.get_logger().info(f"Outside tolerance.{dy}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'x'
                self.get_logger().info("Position reached. Y.")

        elif self.state == 'x':
            if abs(dx) > self.position_tolerance:
                twist.linear.x = self.linear_speed if dx > 0 else -self.linear_speed
                self.get_logger().info(f"Outside tolerance.{dx}")
            else:
                twist = Twist()
                self.get_logger().info("Stopped!")
                self.state = 'done'
                self.get_logger().info("Position reached. X.")

        elif self.state == 'done':
            twist = Twist()
            self.get_logger().info("Done!")
            self.state = 'idle'
            msg = Bool()
            msg.data = True
            self.bool_publisher.publish(msg)

        self.publisher_.publish(twist)

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

    def yaw_from_quaternion(self, q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

def main(args=None):
    rclpy.init(args=args)
    node = GoalFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()