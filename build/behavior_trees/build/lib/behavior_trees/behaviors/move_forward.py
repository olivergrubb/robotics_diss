import py_trees
import time
from geometry_msgs.msg import Twist

class MoveForward(py_trees.behaviour.Behaviour):
    def __init__(self, node, duration, speed, name="Move Forward"):
        super(MoveForward, self).__init__(name)
        self.node = node
        self.duration = duration
        self.speed = speed
        self.start_time = None

    def initialise(self):
        self.node.get_logger().info("Starting to move forward")
        self.start_time = time.time()
        self.twist_msg = Twist()
        self.twist_msg.linear.x = self.speed

    def update(self):
        self.node.get_logger().info("Update")
        if time.time() - self.start_time < self.duration:
            self.node.get_logger().info("Moving forward")
            self.node.cmd_vel_publisher.publish(self.twist_msg)
            return py_trees.common.Status.RUNNING
        else:
            self.twist_msg.linear.x = 0.0
            self.node.cmd_vel_publisher.publish(self.twist_msg)
            self.node.get_logger().info("Finished moving forward")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        self.twist_msg.linear.x = 0.0
        self.node.cmd_vel_publisher.publish(self.twist_msg)
        self.node.get_logger().info("Stopped moving")
