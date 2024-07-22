import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from behavior_trees.behaviors.face_closest_obs import FaceClosestObs
from behavior_trees.behaviors.move_towards_obs import MoveTowardsObs
from behavior_trees.behaviors.rotate_relative_to_obs import RotateRelativeToObs
from behavior_trees.behaviors.follow_wall import FollowWall
from behavior_trees.behaviors.move_forward import MoveForward
from behavior_trees.behaviors.go_to import GoTo
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

    face_closest_obs = FaceClosestObs(node, speed=0.3)
    move_towards_obs_1 = MoveTowardsObs(node, speed=0.3, obs_offset=0.8)
    rotate_relative_to_obs_1 = RotateRelativeToObs(node, obs_rel_pos=270)
    follow_wall_right = FollowWall(node, speed=0.3, wall_side="right", wall_offset=0.7)
    rotate_relative_to_obs_2 = RotateRelativeToObs(node, obs_rel_pos=270)
    move_forward_1 = MoveForward(node, duration=2.0, speed=0.2)
    rotate_relative_to_obs_3 = RotateRelativeToObs(node, obs_rel_pos=180)
    follow_wall_left = FollowWall(node, speed=0.3, wall_side="left", wall_offset=1.4)
    move_forward_2 = MoveForward(node, duration=5.0, speed=0.2)
    
    root.add_children([face_closest_obs,
                        move_towards_obs_1, 
                        rotate_relative_to_obs_1, 
                        follow_wall_right, 
                        rotate_relative_to_obs_2, 
                        move_forward_1, 
                        rotate_relative_to_obs_3,
                        move_forward_2, 
                        follow_wall_left])

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
