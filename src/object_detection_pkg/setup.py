from setuptools import setup

package_name = 'object_detection_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yahboom',
    maintainer_email='1461190907@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = object_detection_pkg.object_detector:main',
            'popup_alert = object_detection_pkg.popup_alert:main',
            'marker_alert = object_detection_pkg.marker_alert:main',
        ],
    },
)


