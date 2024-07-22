import py_trees
import time
from geometry_msgs.msg import Twist
import numpy as np

class FaceClosestObs(py_trees.behaviour.Behaviour):
    def __init__(self, node, speed=0.3, name="Face closest obstacle"):
        super(FaceClosestObs, self).__init__(name)
        self.node = node
        self.speed = speed
        self.start_time = None
        self.facing_wall = False
        self.lidar_msg = None

    def initialise(self):
        self.node.get_logger().info("Starting face obstacle")
        while self.lidar_msg is None:
            self.node.get_logger().info("Waiting for lidar data")
            self.lidar_msg = self.node.most_recent_lidar_data
            time.sleep(0.1)

    def update(self):
        self.node.get_logger().info("Update")
        self.lidar_msg = self.node.most_recent_lidar_data
        if not self.facing_wall:
            ranges = np.array(self.lidar_msg.ranges)
            
            min_range = np.inf
            min_index = -1
            for i, range in enumerate(ranges):
                if float(range) < min_range:
                    min_range = range
                    min_index = i
        
            self.node.get_logger().info("Min distance: {} - Min Index: {}".format(min_range, min_index))

            twist = Twist()
            if min_index < 3 or min_index > 357:
                self.facing_wall = True
                self.node.get_logger().info("Facing Obstacle")
                twist.angular.z = 0.0
            else:
                if min_index < 180:
                    twist.angular.z = 0.2
                else:
                    twist.angular.z = -0.2

            self.node.cmd_vel_publisher.publish(twist)

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("Stopped moving")
