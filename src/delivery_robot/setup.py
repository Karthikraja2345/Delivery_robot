from setuptools import setup

package_name = 'delivery_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Simple delivery robot using ROS2 Navigation',
    entry_points={
        'console_scripts': [
            'delivery_robot = delivery_robot.delivery_robot:main',
        ],
    },
)

