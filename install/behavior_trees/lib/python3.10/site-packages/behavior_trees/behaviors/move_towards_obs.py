import py_trees
import time
from geometry_msgs.msg import Twist
import numpy as np

class MoveTowardsObs(py_trees.behaviour.Behaviour):
    def __init__(self, node, speed=0.3, obs_offset=0.5, name="Move towards obstacle"):
        super(MoveTowardsObs, self).__init__(name)
        self.node = node
        self.speed = speed
        self.obs_offset = obs_offset
        self.at_obs = False
        self.lidar_msg = None

    def initialise(self):
        self.node.get_logger().info("Starting to move towards obstacle")
        while self.lidar_msg is None:
            self.node.get_logger().info("Waiting for lidar data")
            self.lidar_msg = self.node.most_recent_lidar_data
            time.sleep(0.1)

    def update(self):
        self.node.get_logger().info("Update")
        self.lidar_msg = self.node.most_recent_lidar_data
        if not self.at_obs:
            ranges = np.array(self.lidar_msg.ranges)
            
            min_range = np.inf
            for i, range in enumerate(ranges):
                if float(range) < min_range:
                    min_range = range
        
            self.node.get_logger().info("Distance from obs: {}".format(min_range))

            twist = Twist()
            if min_range < self.obs_offset:
                self.at_obs = True
                self.node.get_logger().info("At Obstacle")
                twist.linear.x = 0.0
            else:
                twist.linear.x = self.speed

            self.node.cmd_vel_publisher.publish(twist)

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("Stopped moving")
