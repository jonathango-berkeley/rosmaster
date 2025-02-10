from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # MS200 LiDAR Node
    ms200_lidar_node = Node(
        package='oradar_lidar',  # Replace with your LiDAR package name
        executable='oradar_scan',  # Replace with your LiDAR executable name
        name='MS200',  # Node name
        output='screen',
        parameters=[
            {'port_name': '/dev/oradar'},         # Serial port for the LiDAR
            {'baudrate': 230400},                # Baud rate for the serial connection
            {'angle_min': 0.0},                  # Minimum scan angle (deg)
            {'angle_max': 360.0},                  # Maximum scan angle (deg)
            {'range_min': 0.05},                 # Minimum range (meters)
            {'range_max': 20.0},                 # Maximum range (meters)
            {'clockwise': False},                # Point cloud direction
            {'motor_speed': 10},                 # Motor speed (Hz)
            {'device_model': 'MS200'},           # Device model
            {'frame_id': 'lidar_link'},         # Frame ID for the scan data
            {'scan_topic': 'scan'}         # Topic name for the scan
        ]
    )

    mipi_camera_node = Node(
        package='sensor_interface',
        executable='mipi_camera'
    )



    # Add other sensor nodes here as needed

    return LaunchDescription([
        ms200_lidar_node,
        # mipi_camera_node,
    ])
