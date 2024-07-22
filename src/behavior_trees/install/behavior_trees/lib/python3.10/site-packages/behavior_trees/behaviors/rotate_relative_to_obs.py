import py_trees
import time
from geometry_msgs.msg import Twist
import numpy as np

class RotateRelativeToObs(py_trees.behaviour.Behaviour):
    def __init__(self, node, obs_rel_pos, name="Rotating relative to obstacle"):
        super(RotateRelativeToObs, self).__init__(name)
        self.node = node
        if obs_rel_pos > 357 or obs_rel_pos < 3:
            self.obs_rel_pos = 0
        else:
            self.obs_rel_pos = obs_rel_pos
        self.lidar_msg = None
        self.orientated_relative_to_obs = False

    def initialise(self):
        self.node.get_logger().info("Starting face obstacle")
        while self.lidar_msg is None:
            self.node.get_logger().info("Waiting for lidar data")
            self.lidar_msg = self.node.most_recent_lidar_data
            time.sleep(0.1)

    def update(self):
        self.node.get_logger().info("Update")
        self.lidar_msg = self.node.most_recent_lidar_data
        if not self.orientated_relative_to_obs:
            ranges = np.array(self.lidar_msg.ranges)
            
            min_range = np.inf
            min_index = -1
            for i, dist in enumerate(ranges):
                if float(dist) < min_range:
                    min_range = dist
                    min_index = i
        
            self.node.get_logger().info("Min distance: {} - Min Index: {}".format(min_range, min_index))

            twist = Twist()
            twist.angular.z = 0.0
            if min_index < self.obs_rel_pos + 3 and min_index > self.obs_rel_pos - 3:
                self.orientated_relative_to_obs = True
                self.node.get_logger().info("Orientated relative to obstacle")
                twist.angular.z = 0.0
            else:
                if min_index < (self.obs_rel_pos + 180):
                    twist.angular.z = 0.2
                else:
                    twist.angular.z = -0.2

            self.node.cmd_vel_publisher.publish(twist)

            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.node.get_logger().info("Stopped moving")
