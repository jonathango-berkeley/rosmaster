from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='trajectory_planner',
            executable='get_position',
            name='get_position',
            output='screen'
        )
    ])
