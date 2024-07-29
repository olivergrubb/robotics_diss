import py_trees
import math
from geometry_msgs.msg import Quaternion, PoseStamped

class ProcessRoute(py_trees.behaviour.Behaviour):
    def __init__(self, node, blackboard):
        super(ProcessRoute, self).__init__("Go To")
        self.node = node
        self.blackboard = blackboard
        self._status = py_trees.common.Status.RUNNING
        self.first_run = True

    def initialise(self):
        self.current_pose = self.blackboard.get("current_pose")
        instructions = self.blackboard.get("path_instructions")
        self.current_instruction = instructions.pop(0)
        self.blackboard.set("current_instruction", self.current_instruction)
        self.node.get_logger().info(f"Processing instruction: {self.current_instruction}")

    def update(self):
        if self.current_instruction:
            required_pose = self.instruction_to_pose(self.current_instruction)
            self.blackboard.set("required_pose", required_pose)
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("Failed to process route")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass

    def instruction_to_pose(self, instruction):
        pose = PoseStamped()
        pose.pose.position.x = self.current_pose.pose.position.x
        pose.pose.position.y = self.current_pose.pose.position.y
        pose.pose.position.z = self.current_pose.pose.position.z
        pose.pose.orientation = self.current_pose.pose.orientation
        grid_size = 0.45

        if instruction[0] == "Up":
            pose.pose.position.y = (grid_size * float(instruction[1])) + self.current_pose.pose.position.y
            pose.pose.orientation = self.euler_to_quaternion(0, 0, math.pi/2)
        elif instruction[0] == "Down":
            pose.pose.position.y = (-grid_size * float(instruction[1])) + self.current_pose.pose.position.y
            pose.pose.orientation = self.euler_to_quaternion(0, 0, -math.pi/2)
        elif instruction[0] == "Left":
            pose.pose.position.x = (-grid_size * float(instruction[1])) + self.current_pose.pose.position.x
            pose.pose.orientation = self.euler_to_quaternion(0, 0, math.pi)
        elif instruction[0] == "Right":
            pose.pose.position.x = (grid_size * float(instruction[1])) + self.current_pose.pose.position.x
            pose.pose.orientation = self.euler_to_quaternion(0, 0, 0)

        return pose
    
    def euler_to_quaternion(self, roll, pitch, yaw):

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = Quaternion()
        q.x = cy * cp * sr - sy * sp * cr
        q.y = sy * cp * sr + cy * sp * cr
        q.z = sy * cp * cr - cy * sp * sr
        q.w = cy * cp * cr + sy * sp * sr

        return q
