from glob import glob
from setuptools import setup

package_name = 'crop_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'sensor_msgs',
        'cv_bridge',
        'image_transport',
        'visualization_msgs',
        'nav2_simple_commander',
    ],
    zip_safe=True,
    maintainer='karthik',
    maintainer_email='karthik@todo.todo',
    description='Autonomous Crop Monitoring Robot',
    license='MIT',
    entry_points={
        'console_scripts': [
            'monitor_node = crop_monitor.monitor_node:main',
            'patrol_node = crop_monitor.patrol_node:main',
            'image_publisher = crop_monitor.image_publisher:main',
        ],
    },
)
