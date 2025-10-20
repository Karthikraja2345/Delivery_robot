from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='crop_monitor', executable='image_publisher', output='screen'),
        Node(package='crop_monitor', executable='monitor_node', output='screen'),
        # Node(package='crop_monitor', executable='patrol_node', output='screen'),  # Uncomment if nav ready
        Node(package='rviz2', executable='rviz2',
             arguments=['-d', '/home/karthik/ros2_ws/src/crop_monitor/rviz/monitor.rviz'],
             output='screen'),
    ])
