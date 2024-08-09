from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
import py_trees
import rclpy

class CancelGoal(py_trees.behaviour.Behaviour):
    def __init__(self, node, blackboard):
        super(CancelGoal, self).__init__("Cancel Goal")
        self.blackboard = blackboard
        self.action_client = None
        self.node = node
        self.cancel_future = None
        self.attempt_counter = 0

    def setup(self, **kwargs):
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')
        self.action_client.wait_for_server()
        return True

    def initialise(self):
        self.goal_handle = self.blackboard.get("current_goal_handle")
        if self.goal_handle is None:
            self.node.get_logger().warn("No goal handle found on blackboard")
            self._status = py_trees.common.Status.FAILURE
        else:
            self._status = py_trees.common.Status.RUNNING
            self.cancel_future = self.goal_handle.cancel_goal_async()

    def update(self):
        if self._status != py_trees.common.Status.RUNNING:
            return self._status

        if self.cancel_future is None:
            self.node.get_logger().error("Cancel future is None")
            return py_trees.common.Status.FAILURE

        rclpy.spin_until_future_complete(self.node, self.cancel_future, timeout_sec=1.0)

        if self.cancel_future.done():
            cancel_response = self.cancel_future.result()
            if cancel_response.return_code == 0:
                self.node.get_logger().info("Current goal successfully canceled")
                self.blackboard.set("current_goal_handle", None)
                return py_trees.common.Status.SUCCESS
            elif self.attempt_counter >= 3:
                self.node.get_logger().error(f"Goal cancellation failed after {self.attempt_counter} attempts \n RESTART THE SYSTEM")
                return py_trees.common.Status.FAILURE
            else:
                self.node.get_logger().error("Goal cancellation was rejected \n Trying again...")
                self.cancel_future = self.goal_handle.cancel_goal_async()
                self.attempt_counter += 1
                return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.cancel_future = None