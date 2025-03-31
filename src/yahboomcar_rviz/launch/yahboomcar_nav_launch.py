from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_path('yahboomcar_rviz')
    default_rviz_config_path = package_path / 'rviz/nav.rviz'
    rviz_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )

    marker_alert_node = Node(
        package='object_detection_pkg',
        executable='marker_alert',
        name='marker_alert_node',
        output='screen'
    )

    aruco_detector_node = Node(
        package='object_detection_pkg',
        executable='aruco_detector_subscriber',
        name='aruco_detector_subscriber',
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        rviz_node,
        marker_alert_node,
        aruco_detector_node
    ])

