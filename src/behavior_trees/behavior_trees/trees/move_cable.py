from behavior_trees.behaviors.behaviors import ProcessRoute, CheckIfFinished, GoTo, PlanRoute
import py_trees
from py_trees import composites
import rclpy
from rclpy.node import Node


class MoveCable(Node):
    def __init__(self):
        super().__init__("Move_Cable_Responder_Tree_Executer")
        
        
    def get_tree_name(self):
        return "move_cable"

    def execute_tree(self, instructions, blackboard_state):
        self.get_logger().info("Executing Move Cable Responder Tree")
        original_instructions = blackboard_state.get("path_instructions")
        last_instruction_before_error = blackboard_state.get("current_instruction")

        blackboard_state.set("path_instructions", instructions)

        root = composites.Sequence("Root Sequence", memory=True)
        cable_recovery = composites.Sequence("Cable Recovery", memory=True)

        process_route = ProcessRoute(self, blackboard_state)
        go_to = GoTo(self, blackboard_state)
        check_if_finished = CheckIfFinished(self, blackboard_state)
        replan_route = PlanRoute(self, blackboard_state, start_location=blackboard_state.get("current_map_location"), re_plan_flag=True)

        cable_recovery.add_children([process_route, go_to, check_if_finished])

        repeat_until_fail = py_trees.decorators.Inverter(
            name="RepeatUntilFail",
            child=py_trees.decorators.Repeat(
                name="Repeat",
                child=cable_recovery,
                num_success=-1
            )
        )
        root.add_children([repeat_until_fail, replan_route])
        tree = py_trees.trees.BehaviourTree(root)
        tree.setup()
        
        tree.root.tick_once()
        while tree.root.status == py_trees.common.Status.RUNNING:
            rclpy.spin_once(self, timeout_sec=0.1)
            tree.root.tick_once()
            status = tree.root.status
            if status == py_trees.common.Status.SUCCESS:
                self.get_logger().info("Recovery Behavior Tree completed successfully.")
                break
        
        blackboard_state.set("path_instructions", original_instructions)
        blackboard_state.set("current_instruction", last_instruction_before_error)
        if tree.root.status == py_trees.common.Status.SUCCESS:
            return True
        else:
            return False
        