from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Path configurations for the yahboomcar_nav package and nav2_bringup
    package_path = get_package_share_directory('yahboomcar_nav')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('sensor_interface'), 'launch', 'sensor_launch.py'
            )
        )
    )

    # Configuration for nav2_bringup
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_yaml_path = LaunchConfiguration(
        'map', default=os.path.join(package_path, 'maps', 'maplorong.yaml'))
    nav2_param_path = LaunchConfiguration('params_file', default=os.path.join(
        package_path, 'params', 'teb_nav_params.yaml'))

    # LiDAR publisher node  - using for cartographer
    cartographer_ms200_scan_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('oradar_lidar'), 'launch'),
         '/ms200_scan.launch.py'])
      )
    
   # LiDAR publisher node  - using gmapping
    gmapping_m200_scan_node = Node(
      package='oradar_lidar',
      executable='oradar_scan',
      name='MS200',
      output='screen',
      parameters=[
        {'device_model': 'MS200'},
        # {'frame_id': 'laser_frame'},
        # {'scan_topic': 'MS200/scan'},
        {'frame_id': 'lidar_link'},
        {'scan_topic': 'scan'},
        {'port_name': '/dev/oradar'},
        {'baudrate': 230400},
        {'angle_min': 0.0},
        {'angle_max': 0.0},
        {'range_min': 0.15},
        {'range_max': 20.0},
        {'clockwise': False},
        {'motor_speed': 10}
      ]
    )
    

    # Add Mcnamu Driver Node - the whole robot driver basically
    mcnamu_driver_node = Node(
        package='yahboomcar_bringup',  # Replace with your package name if different
        executable='Mcnamu_driver',  # Use the script file directly or the registered executable name    
        )


    #Launch Argument for base_node
    pub_odom_tf_arg = DeclareLaunchArgument('pub_odom_tf', default_value='true',
                                            description='Whether to publish the tf from the original odom to the base_footprint')

    # Spongebob Base Node
    base_node = Node(
        package='yahboomcar_base_node',  # Replace with the appropriate package name
        executable='base_node',          # Replace with the appropriate executable name
        parameters=[{'pub_odom_tf': LaunchConfiguration('pub_odom_tf')}],
        output='screen'
    )


    #IMU filter
    imu_filter_config = os.path.join(
         get_package_share_directory('yahboomcar_bringup'), 
         'params',
         'imu_filter_param.yaml'
      )
    
    imu_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        parameters=[imu_filter_config],
        name='imu_filter_madgwick'
    )


    # ekf_node
    ekf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('robot_localization'), 'launch'),
         '/ekf.launch.py'])
      )
    

    #joystick controller node
    yahboom_joy_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_ctrl'), 'launch'),
         '/yahboomcar_joy_launch.py'])
    )
    

    # robot description - URDF
    yahboom_description_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('yahboomcar_description'), 'launch'),
         '/description_launch.py'])
    )

    #slam gmapping launch
    slam_gmapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('slam_gmapping'), 'launch'),
         '/slam_gmapping.launch.py'])
    )

    cartographer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
        get_package_share_directory('yahboomcar_nav'), 'launch'),
         '/cartographer_launch.py'
        ])
    )

    # # Add Navigation Node (example placeholder for navigation stack)
    # navigation_node = Node(
    #     package='your_navigation_package',  # Replace with your navigation package
    #     executable='navigation_node',  # Replace with your navigation executable
    #     name='navigation_node',
    #     output='screen',
    #     parameters=[
    #         {'param_name': 'param_value'},  # Replace with actual parameters
    #     ]
    # )

    # NAV TEB
    # Add the nav2_bringup integration
    nav2_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [nav2_bringup_dir, '/launch', '/bringup_launch.py']),
        launch_arguments={
            'map': map_yaml_path,
            'use_sim_time': use_sim_time,
            'params_file': nav2_param_path
        }.items(),
    )

    # Combine everything into a single launch description
    return LaunchDescription([
        sensor_launch,

        cartographer_ms200_scan_node,

        mcnamu_driver_node,
        pub_odom_tf_arg,
        base_node,
        imu_filter_node,
        ekf_node,
        yahboom_description_node,
        yahboom_joy_node,

        # Include Navigation 2 bringup
        # nav2_bringup_launch,
        # slam_gmapping_launch,
        cartographer_launch
        

    ])
