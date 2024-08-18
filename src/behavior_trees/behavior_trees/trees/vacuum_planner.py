import rclpy
from rclpy.node import Node
import py_trees
import py_trees_ros
from behavior_trees.behaviors.behaviors import CancelGoal, CheckIfFinished, GoTo, ProcessRoute, PlanRoute
from std_srvs.srv import Trigger
import py_trees.composites as composites
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from action_interfaces.action import ExecuteRecoveryTree
import time
from behavior_trees.trees.responder_trees import ALL_RESPONDER_TREES
import ast
from rclpy.action import ActionServer

# This class is the main node for the vacuum planner and is responsible for setting up the behavior tree and handling service / action requests from
# error responders
class VacuumPlanner(Node):
    def __init__(self):
        super().__init__('vacuum_planner')

        self.declare_parameter('map_name', 'altered_map')
        self.declare_parameter('start_position_x', -8.6894)
        self.declare_parameter('start_position_y', 7.69215)
        self.declare_parameter('x_offset', 0.4)
        self.declare_parameter('y_offset', -0.7)
        self.declare_parameter('map_resolution', 9)
        self.declare_parameter('row_start_location', 1)
        self.declare_parameter('column_start_location', 1)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.blackboard_service = self.create_service(Trigger, 'get_blackboard', self.get_blackboard_callback)
        self.action_server = ActionServer(self, ExecuteRecoveryTree, 'execute_recovery_tree', self.execute_recovery_tree_callback)
        self.lidar_subscriber = self.create_subscription(LaserScan, 'scan', self.lidar_callback, 10)
        self.error_subscriber = self.create_subscription(String, 'error_responder', self.error_callback, 10)
        self.error_state = False
        self.error_msg = None
        self.error_handled = False
        self.most_recent_lidar_data = None
        self.get_logger().info("Starting vacuum_planner Node")
    
    def get_param_values(self):
        map_name = self.get_parameter('map_name').get_parameter_value().string_value
        start_position_x = self.get_parameter('start_position_x').get_parameter_value().double_value
        start_position_y = self.get_parameter('start_position_y').get_parameter_value().double_value
        x_offset = self.get_parameter('x_offset').get_parameter_value().double_value
        y_offset = self.get_parameter('y_offset').get_parameter_value().double_value
        map_resolution = self.get_parameter('map_resolution').get_parameter_value().integer_value
        row_start_location = self.get_parameter('row_start_location').get_parameter_value().integer_value
        column_start_location = self.get_parameter('column_start_location').get_parameter_value().integer_value

        start_location = (row_start_location, column_start_location)


        return map_name, start_position_x, start_position_y, x_offset, y_offset, map_resolution, start_location

    def execute_recovery_tree_callback(self, goal_handle):
        global FATAL_ERROR
        if FATAL_ERROR:
            response = ExecuteRecoveryTree.Result()
            response.success = False
            return response
        else:
            request_params = ast.literal_eval(goal_handle.request.message)
            tree_to_execute = request_params['tree_to_execute']
            instructions = request_params['instructions']
            self.get_logger().warn(f"Received request to execute recovery tree: {tree_to_execute}")

            map_name, start_position_x, start_position_y, x_offset, y_offset, map_resolution, start_location = self.get_param_values()
            
            success = False
            for responder_trees in ALL_RESPONDER_TREES:
                if responder_trees.get_tree_name(self) == tree_to_execute:
                    responder_tree_instance = responder_trees()
                    success = responder_tree_instance.execute_tree(instructions, blackboard, map_name, map_resolution)
            
            self.get_logger().info(f"Recovery behaviour tree done with success {success}")
            
            response = ExecuteRecoveryTree.Result()
            response.success = success
            return response
    
    def get_blackboard_callback(self, request, response):
        try:
            blackboard_state = self.create_blackboard_msg(blackboard)
            msg = String()
            msg.data = str(blackboard_state)
            blackboard_state = msg
            response.message = blackboard_state.data
            response.success = True
        except Exception as e:
            self.get_logger().error(f"Error creating response for blackboard service: {str(e)}")
            response.success = False
            response.message = "Error occurred"
                
        return response

    def lidar_callback(self, msg):
        self.most_recent_lidar_data = msg
    
    def error_callback(self, msg):
        self.error_message = msg.data
        if self.error_message == 'responding':
            self.error_state = True
        else:
            self.error_state = False
            # Reset current instruction
            path_instructions = blackboard.get("path_instructions")
            current_instruction = blackboard.get("current_instruction")
            path_instructions.insert(0, current_instruction)
            blackboard.set("path_instructions", path_instructions)

    
    def create_blackboard_msg(self, blackboard):
        current_map_location = blackboard.get("current_map_location")
        current_pose = blackboard.get("current_pose")
        required_pose = blackboard.get("required_pose")
        path_instructions = blackboard.get("path_instructions")
        map_data = blackboard.get("map")
        current_instruction = blackboard.get("current_instruction")

        blackboard = {
            "current_map_location": current_map_location,
            # Poses must be strings to be parsed client side
            "current_pose": str(current_pose),
            "required_pose": str(required_pose),
            "path_instructions": path_instructions,
            "map": map_data,
            "current_instruction": current_instruction
        }
        
        return blackboard

def create_behavior_tree(node, map_name, start_position_x, start_position_y, x_offset, y_offset, map_resolution, start_location):
    current_pose = PoseStamped()

    current_pose.pose.position.x = start_position_x + x_offset
    current_pose.pose.position.y = start_position_y + y_offset
    blackboard.set("current_pose", current_pose)
    
    # Setup tree
    root = composites.Sequence("Root Sequence", memory=True)
    vacuum_sequence = composites.Sequence("Vacuum Sequence", memory=True)
    init_sequence = composites.Sequence("Init Sequence", memory=True)

    route_plan = PlanRoute(node, blackboard, start_location, map_name, map_resolution)
    process_route = ProcessRoute(node, blackboard)
    go_to = GoTo(node, blackboard)
    check_if_finished = CheckIfFinished(node, blackboard)

    init_sequence.add_children([route_plan])
    vacuum_sequence.add_children([process_route, go_to, check_if_finished])

    repeat_until_fail = py_trees.decorators.Inverter(
        name="RepeatUntilFail",
        child=py_trees.decorators.Repeat(
            name="Repeat",
            child=vacuum_sequence,
            num_success=-1
        )
    )
    root.add_children([init_sequence, repeat_until_fail])

    return root

def check_for_interrupts(node, blackboard):
    if node.error_state and not node.error_handled:
        node.get_logger().info("Error detected. Pausing vacuum planner...")
        recover_sequence = composites.Sequence("Recover Sequence", memory=True)
        cancel_goal = CancelGoal(node, blackboard)
        recover_sequence.add_children([cancel_goal])
        goal_cancelled = False
        while not goal_cancelled:
            recover_sequence.tick_once()
            status = recover_sequence.status
            if status == py_trees.common.Status.SUCCESS:
                goal_cancelled = True
                node.error_handled = True
                node.get_logger().info("Goal cancelled successfully.")
            elif status == py_trees.common.Status.FAILURE:
                node.error_handled = True
                global FATAL_ERROR
                FATAL_ERROR = True
                break
            time.sleep(0.1)
        node.error_handled = True
        node.get_logger().info("Waiting for error to be resolved...")

FATAL_ERROR = False

def main(args=None):
    rclpy.init(args=args)
    node = VacuumPlanner()
    node.error_handled = False

    map_name, start_position_x, start_position_y, x_offset, y_offset, map_resolution, start_location = VacuumPlanner().get_param_values()

    global blackboard
    blackboard = py_trees.blackboard.Client(name="Blackboard")
    blackboard.register_key(key="path_instructions", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="path_instructions", access=py_trees.common.Access.READ)

    blackboard.register_key(key="map", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="map", access=py_trees.common.Access.READ)

    blackboard.register_key(key="current_map_location", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="current_map_location", access=py_trees.common.Access.READ)
    blackboard.set("current_map_location", start_location)

    blackboard.register_key(key="current_instruction", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="current_instruction", access=py_trees.common.Access.READ)
    blackboard.set("current_instruction", None)

    blackboard.register_key(key="current_goal_handle", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="current_goal_handle", access=py_trees.common.Access.READ)

    blackboard.register_key(key="required_pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="required_pose", access=py_trees.common.Access.READ)
    blackboard.set("required_pose", None)

    blackboard.register_key(key="current_pose", access=py_trees.common.Access.WRITE)
    blackboard.register_key(key="current_pose", access=py_trees.common.Access.READ)

    behavior_tree = py_trees_ros.trees.BehaviourTree(create_behavior_tree(node, map_name, start_position_x, start_position_y, x_offset, y_offset, map_resolution, start_location))
    behavior_tree.setup(timeout=600.0)

    global FATAL_ERROR

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            if node.error_state:
                check_for_interrupts(node, blackboard)
                if FATAL_ERROR:
                    node.get_logger().error("Fatal error occurred. Shutting down...")
                    break
            else:
                node.error_handled = False  
                behavior_tree.root.tick_once()
                status = behavior_tree.root.status
                if status == py_trees.common.Status.SUCCESS:
                    node.get_logger().info("Behavior Tree completed successfully.")
                    break
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

if __name__ == '__main__':
    main()
