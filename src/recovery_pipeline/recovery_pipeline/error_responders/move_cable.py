import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
#from action_interfaces.action import ExecuteRecoveryTree
import ast

class MoveCableErrorResponder(Node):
    def __init__(self):
        super().__init__('move_cable')
        self.blackboard_client = self.create_client(Trigger, 'get_blackboard')
        self.execute_recovery_tree_client = ActionClient(self, ExecuteRecoveryTree, 'execute_recovery_tree')
        self.publisher = self.create_publisher(String, 'error_responder', 10)
        self.blackboard_callback_msg = None
        while not self.blackboard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.blackboard_req = Trigger.Request()

    def send_blackboard_request(self):
        while not self.blackboard_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        
        self.future = self.blackboard_client.call_async(self.blackboard_req)        
        timeout_secs = 20.0
        try:
            rclpy.spin_until_future_complete(self, self.future, timeout_sec=timeout_secs)
        except Exception as e:
            self.get_logger().error(f'Exception during spin: {str(e)}')
        
        if self.future.done():
            try:
                response = self.future.result()
                return response
            except Exception as e:
                self.get_logger().error(f'Service call failed: {str(e)}')
        else:
            self.get_logger().error(f'Service call did not complete within {timeout_secs} seconds.')
        
        return None
    
    def send_execute_recovery_tree_request(self, tree_to_execute, instructions):
        request = ExecuteRecoveryTree.Goal()
        request.message = str({'tree_to_execute': tree_to_execute,
                                           'instructions': instructions})
        self.execute_recovery_tree_client.wait_for_server()
        self.future = self.execute_recovery_tree_client.send_goal_async(request)
        self.future.add_done_callback(self.future_callback)
        rclpy.spin_until_future_complete(self, self.future)

    def future_callback(self, future):
        goal_handle = future.result()
        self.get_logger().info('future_callback')
        if not goal_handle.accepted:
            self.get_logger().info('Recovery Tree Recovery Rejected')
        else:
            self.get_logger().info('Recovery Tree Recovery Accepted')
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)
            rclpy.spin_until_future_complete(self, self.get_result_future)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"RESULT: {result}")
        error_responder_msg = String()

        if result.success:
            self.get_logger().info('Successfully corrected error')
            error_responder_msg.data = 'finished'
        else:
            self.get_logger().error('Failed to correct error - manual intervention required. Press ENTER when robot is ready to proceed')
            input()
            error_responder_msg.data = 'finished'
        self.publisher.publish(error_responder_msg)
            

    def respond(self):
        self.get_logger().error('CABLE DETECTED: Attempting to correct...')
        self.move_cable()


    def get_solution_name(self):
        return 'BT: move_cable'

    # Actual logic for recovery
    def move_cable(self):
        current_blackboard_state = self.send_blackboard_request()
        current_blackboard_state = ast.literal_eval(str(current_blackboard_state.message))
        current_map_location = current_blackboard_state["current_map_location"]

        #TODO

        instructions = generate_instructions()

        if instructions is not None:
            tree_to_execute = 'perpendicular_alignment_to_doorway_and_speed'
            self.send_execute_recovery_tree_request(tree_to_execute, instructions)
        else:
            error_responder_msg = String()
            self.get_logger().error('Failed to correct error - manual intervention required. Press ENTER when robot is ready to proceed')
            input()
            error_responder_msg.data = 'finished'
            self.publisher.publish(error_responder_msg)


def generate_instructions():
    pass

def longest_accessible_side_adjacent_to_cable(map_array):
    if not map_array or not map_array[0]:
        return []

    rows, cols = len(map_array), len(map_array[0])
    directions = [(0, 1, 'left'), (0, -1, 'right'), (1, 0, 'up'), (-1, 0, 'down')]
    longest_paths = []
    max_length = 0

    def is_valid(x, y):
        return 0 <= x < rows and 0 <= y < cols

    def is_accessible(x, y):
        return map_array[x][y] in ['□', 'X']

    def is_object(x, y):
        return map_array[x][y] == 'C'

    def find_path(x, y, dx, dy):
        path = []
        while is_valid(x, y) and is_accessible(x, y):
            path.append((x, y))
            x, y = x + dx, y + dy
        return path

    for i in range(rows):
        for j in range(cols):
            if is_object(i, j):
                for dx, dy, direction in directions:
                    nx, ny = i + dx, j + dy
                    if is_valid(nx, ny) and is_accessible(nx, ny):
                        path = find_path(nx, ny, -dx, -dy)  # Note the negative direction
                        if len(path) > max_length:
                            max_length = len(path)
                            longest_paths = [(path, direction)]
                        elif len(path) == max_length:
                            longest_paths.append((path, direction))

    # Group the results by direction
    result = {}
    for path, direction in longest_paths:
        if direction not in result:
            result[direction] = path
        else:
            result[direction].extend(path)
    
    # Remove duplicates from each direction's path
    for direction in result:
        result[direction] = list(set(result[direction]))

    return result
    
    

if __name__ == '__main__':
    current_map_location = (13, 15)
    map_array = [['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', 'C', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', 'C', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', 'C', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', 'C', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '■', '□', '■', '□', '□', '□'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□']]
    longest_paths = longest_accessible_side_adjacent_to_cable(map_array)
    print(longest_paths)

    # Visualize the result
    for r, row in enumerate(map_array):
        for c, cell in enumerate(row):
            print(cell, end=' ')
        print()