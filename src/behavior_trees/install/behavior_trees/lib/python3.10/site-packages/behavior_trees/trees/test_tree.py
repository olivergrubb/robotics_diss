import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from behavior_trees.behaviors.follow_wall import FollowWall
import py_trees.composites as composites
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class VacuumPlanner(Node):
    def __init__(self):
        super().__init__('vacuum_planner')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.most_recent_lidar_data = None
        self.get_logger().info("Starting vacuum_planner Node")

    def lidar_callback(self, msg):
        self.most_recent_lidar_data = msg


def create_behavior_tree(node):
    # Root of the behavior tree
    root = composites.Sequence("Root Sequence", memory=True)

    # Add custom behaviors
    follow_wall_left = FollowWall(node, speed=0.3, wall_side="left", wall_offset=1)
    # Add behaviors to the root sequence
    root.add_children([follow_wall_left])

    return root

def main(args=None):
    rclpy.init(args=args)
    node = VacuumPlanner()

    # Create and setup the behavior tree
    behavior_tree = py_trees_ros.trees.BehaviourTree(create_behavior_tree(node))
    behavior_tree.setup(timeout=600.0)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            behavior_tree.root.tick_once()
            status = behavior_tree.root.status
            node.get_logger().info(f"Status: {status}")
            if status == py_trees.common.Status.SUCCESS:
                node.get_logger().info("Behavior Tree completed successfully.")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
