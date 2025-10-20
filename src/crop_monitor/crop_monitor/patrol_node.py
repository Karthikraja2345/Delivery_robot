import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')
        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.waypoints = [(0.5, 0.0), (0.5, 1.0), (0.0, 1.0), (0.0, 0.0)]
        self.current_index = 0

    def patrol(self):
        while rclpy.ok():
            goal = self.waypoints[self.current_index]
            pose = {'position': {'x': goal[0], 'y': goal[1], 'z': 0},
                    'orientation': {'z': 0}}
            self.navigator.goToPose(pose)
            self.navigator.waitUntilNav2GoalComplete()
            self.current_index = (self.current_index + 1) % len(self.waypoints)

def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    node.patrol()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
