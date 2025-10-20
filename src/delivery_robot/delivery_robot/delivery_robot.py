#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import time

class DeliveryRobot(Node):
    def __init__(self):
        super().__init__('delivery_robot')
        self.navigator = BasicNavigator()
        
        def create_pose(x, y, yaw):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.navigator.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.z = yaw
            pose.pose.orientation.w = 1.0
            return pose

        initial_pose = create_pose(0.0, 0.0, 0.0)
        self.navigator.setInitialPose(initial_pose)
        self.navigator.waitUntilNav2Active()

        self.pickup_pose = create_pose(1.0, 0.0, 0.0)
        self.dropoff_pose = create_pose(0.0, 1.0, 1.57)

    def run_delivery(self):
        while rclpy.ok():
            self.get_logger().info('Navigating to pick-up point...')
            self.navigator.goToPose(self.pickup_pose)
            while not self.navigator.isTaskComplete():
                time.sleep(0.5)
            self.get_logger().info('Arrived at pick-up point. Picking up package...')
            time.sleep(3)

            self.get_logger().info('Navigating to drop-off point...')
            self.navigator.goToPose(self.dropoff_pose)
            while not self.navigator.isTaskComplete():
                time.sleep(0.5)
            self.get_logger().info('Arrived at drop-off point. Delivering package...')
            time.sleep(3)

            self.get_logger().info('Delivery cycle complete. Restarting...')
            time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    delivery_robot = DeliveryRobot()
    delivery_robot.run_delivery()
    delivery_robot.navigator.lifecycleShutdown()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
