import py_trees

class CheckIfFinished(py_trees.behaviour.Behaviour):
    def __init__(self, node, blackboard):
        super(CheckIfFinished, self).__init__("Check if finished")
        self.node = node
        self.blackboard = blackboard

    def initialise(self):
        pass


    def update(self):
        instructions = self.blackboard.get("path_instructions")
        self.node.get_logger().info(f"Number of instructions left {len(instructions)}")
        if len(instructions) == 0:
            # Must be fail to break out of execution loop
            return py_trees.common.Status.FAILURE
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        pass
