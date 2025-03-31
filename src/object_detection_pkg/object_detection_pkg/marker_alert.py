import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from builtin_interfaces.msg import Duration

class MarkerAlertNode(Node):
    def __init__(self):
        super().__init__('marker_alert_node')

        self.marker_pub = self.create_publisher(Marker, '/object_marker_alert', 10)

        self.subscription = self.create_subscription(
            Bool,
            '/is_human',
            self.detected_callback,
            10)

        self.get_logger().info('Marker Alert Node initialized.')

    def detected_callback(self, msg: Bool):
        marker = Marker()
        marker.header.frame_id = "map"  # 更改为map坐标系
        marker.header.stamp = self.get_clock().now().to_msg()

        marker.ns = "object_alert"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING

        if msg.data:
            marker.action = Marker.ADD
            marker.text = "Survivor Detected!"
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 1.5

            marker.scale.z = 0.4  # 增大字体尺寸

            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            marker.lifetime = Duration(sec=1)

            self.get_logger().info('Survivor Detected! Marker Displayed.')
        else:
            marker.action = Marker.DELETE
            self.get_logger().info('No Survivor Detected. Marker Removed.')

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarkerAlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

