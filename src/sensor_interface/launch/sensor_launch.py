from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sensor_interface',
            executable='camera',
            name='camera_node',
            output='screen'
        ),
        Node(
            package='sensor_interface',
            executable='detect_object',
            name='detect_object_node',
            output='screen'
        )
    ])
