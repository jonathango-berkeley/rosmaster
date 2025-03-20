from setuptools import setup
import os
from glob import glob

package_name = 'sensor_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rosmaster',
    maintainer_email='jonathango@berkeley.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mipi_camera_publisher = sensor_interface.mipi_camera:main',
            'aruco_detector_subscriber = sensor_interface.aruco_detector_subscriber:main',
            'aruco_map_transform = sensor_interface.aruco_map_transform:main',
        ],
    },
)
