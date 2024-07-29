import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
import py_trees

class GoTo(py_trees.behaviour.Behaviour):
    def __init__(self, node, blackboard, pose = None):
        super(GoTo, self).__init__("Go To")
        self.blackboard = blackboard
        self.action_client = None
        self.node = node
        self.pose = pose

    def setup(self, **kwargs):
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.action_client.wait_for_server()
        return py_trees.common.Status.SUCCESS

    def initialise(self):
        self._goal_handle = None
        self._status = py_trees.common.Status.RUNNING
        if self.blackboard.get("required_pose") is not None:
            self.pose = self.blackboard.get("required_pose")

        self.goal_msg = NavigateToPose.Goal()
        self.goal_msg.pose.header.frame_id = 'map' 
        self.goal_msg.pose.pose = self.pose.pose
        self.goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        self.node.get_logger().info(f"X and y destination: {self.pose.pose.position.x}, {self.pose.pose.position.y}")

        self._send_goal_future = self.action_client.send_goal_async(
            self.goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.node.get_logger().info('Goal rejected')
            self._status = py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().info('Goal accepted')
            self._get_result_future = self._goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)
            self.blackboard.set("current_goal_handle", self._goal_handle)
            self._status = py_trees.common.Status.RUNNING

    def get_result_callback(self, future):
        result = future.result()
        if result.status == GoalStatus.STATUS_SUCCEEDED:
            self.blackboard.set("current_pose", self.pose)
            self.update_map(self.blackboard.get("current_instruction"))
            self.node.get_logger().info("SUCCESSFULLY NAVIGATED")
            self._status = py_trees.common.Status.SUCCESS
        elif result.status == GoalStatus.STATUS_CANCELED:
            # Only invoked during error recovery - need this to accept cancelation and wait for recovery action to complete
            self._status = py_trees.common.Status.SUCCESS
        elif result.status == GoalStatus.STATUS_ABORTED:
            self.node.get_logger().info('Goal aborted')
            self._status = py_trees.common.Status.FAILURE
        else:
            self.node.get_logger().info('Goal failed')
            self._status = py_trees.common.Status.FAILURE

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    def update(self):
        if self._goal_handle is None:
            return py_trees.common.Status.RUNNING

        if self._goal_handle.status in {GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED, GoalStatus.STATUS_CANCELED}:
            return self._status

        return py_trees.common.Status.RUNNING
    
        
    def update_map(self, instruction):
        current_map_location = self.blackboard.get("current_map_location")
        current_map = self.blackboard.get("map")
        
        # Mark the current location as 'X' before moving
        current_map[current_map_location[0]][current_map_location[1]] = 'X'
        
        if instruction[0] == "Up":
            for i in range(1, int(instruction[1]) + 1):
                current_map[current_map_location[0] - i][current_map_location[1]] = 'R'
            current_map_location = (current_map_location[0] - int(instruction[1]), current_map_location[1])
        elif instruction[0] == "Down":
            for i in range(1, int(instruction[1]) + 1):
                current_map[current_map_location[0] + i][current_map_location[1]] = 'R'
            current_map_location = (current_map_location[0] + int(instruction[1]), current_map_location[1])
        elif instruction[0] == "Left":
            for i in range(1, int(instruction[1]) + 1):
                current_map[current_map_location[0]][current_map_location[1] - i] = 'R'
            current_map_location = (current_map_location[0], current_map_location[1] - int(instruction[1]))
        elif instruction[0] == "Right":
            for i in range(1, int(instruction[1]) + 1):
                current_map[current_map_location[0]][current_map_location[1] + i] = 'R'
            current_map_location = (current_map_location[0], current_map_location[1] + int(instruction[1]))
        
        self.blackboard.set("current_map_location", current_map_location)
        self.blackboard.set("map", current_map)

        print_message = ""
        for row in current_map:
            for cell in row:
                print_message += f"{cell} "
            print_message += "\n"
        self.node.get_logger().info(f"Updated map: \n{print_message}")

    def terminate(self, new_status):
        if self._goal_handle is not None and self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
            self._goal_handle.cancel_goal_async()
