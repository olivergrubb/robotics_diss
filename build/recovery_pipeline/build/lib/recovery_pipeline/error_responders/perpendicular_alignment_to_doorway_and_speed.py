import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from action_interfaces.action import ExecuteRecoveryTree
import ast

class PerpendicularAlignmentToDoorwayAndSpeedErrorResponder(Node):
    def __init__(self):
        super().__init__('perpendicular_alignment_to_doorway_and_speed_error_responder')
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
        self.get_logger().error('DOORWAY ERROR DETECTED: Attempting to correct...')
        self.doorway_alignment_and_speed()


    def get_solution_name(self):
        return 'BT: perpendicular_alignment_to_doorway_and_speed'

    # Actual logic for recovery
    def doorway_alignment_and_speed(self):
        current_blackboard_state = self.send_blackboard_request()
        current_blackboard_state = ast.literal_eval(str(current_blackboard_state.message))
        current_map_location = current_blackboard_state["current_map_location"]
        self.get_logger().info(f'Current map location: {current_map_location}')

        doorways = detect_doorways(current_blackboard_state["map"])
        closest_doorway_location = closest_doorway(current_map_location, doorways)

        target_cell = get_target_cell(current_map_location, closest_doorway_location)
        instructions = generate_instructions(current_map_location, target_cell)

        if instructions is not None:
            tree_to_execute = 'perpendicular_alignment_to_doorway_and_speed'
            self.send_execute_recovery_tree_request(tree_to_execute, instructions)
        else:
            error_responder_msg = String()
            self.get_logger().error('Failed to correct error - manual intervention required. Press ENTER when robot is ready to proceed')
            input()
            error_responder_msg.data = 'finished'
            self.publisher.publish(error_responder_msg)

def detect_doorways(map_array):
    rows = len(map_array)
    cols = len(map_array[0])
    doorways = []

    def is_obstacle(r, c):
        return 0 <= r < rows and 0 <= c < cols and map_array[r][c] == '■'

    def is_free_space(r, c):
        return 0 <= r < rows and 0 <= c < cols and map_array[r][c] in ['□', 'X', 'R']

    for r in range(rows):
        for c in range(cols):
            if is_free_space(r, c):
                # Check horizontal doorway
                if (is_obstacle(r, c-1) and is_obstacle(r, c+1) and 
                    is_free_space(r-1, c) and is_free_space(r+1, c)):
                    doorways.append((r, c))
                # Check vertical doorway
                elif (is_obstacle(r-1, c) and is_obstacle(r+1, c) and 
                      is_free_space(r, c-1) and is_free_space(r, c+1)):
                    doorways.append((r, c))

    return doorways

def closest_doorway(current_location, doorways):
    distance = min(doorways, key=lambda d: abs(d[0] - current_location[0]) + abs(d[1] - current_location[1]))
    rms = ((distance[0] - current_location[0])**2 + (distance[1] - current_location[1])**2)**0.5
    if rms < 2:
        return distance
    else:
        return None

def get_target_cell(current_location, closest_doorway_location):
    if closest_doorway_location is None:
        return None
    if current_location[0] == closest_doorway_location[0] and current_location[1] == closest_doorway_location[1]:
        return None
    if current_location[0] == closest_doorway_location[0]:
        if current_location[1] < closest_doorway_location[1]:
            return (current_location[0], closest_doorway_location[1] - 2)
        else:
            return (current_location[0], closest_doorway_location[1] + 2)
    elif current_location[1] == closest_doorway_location[1]:
        if current_location[0] < closest_doorway_location[0]:
            return (closest_doorway_location[0] - 2, current_location[1])
        else:
            return (closest_doorway_location[0] + 2, current_location[1])
    else:
        return None

def generate_instructions(current_location, target_cell):
    if target_cell is None:
        return None
    instructions = []
    # Move to target cell then move to the cell perpendicular to the doorway (needs to be adjusted based on grid resolution)
    if current_location[0] == target_cell[0]:
        if current_location[1] < target_cell[1]:
            instructions.append(('Right', target_cell[1] - current_location[1]))
            instructions.append(('Left', 1))
        else:
            instructions.append(('Left', current_location[1] - target_cell[1]))
            instructions.append(('Right', 1))
    elif current_location[1] == target_cell[1]:
        if current_location[0] < target_cell[0]:
            instructions.append(('Down', target_cell[0] - current_location[0]))
            instructions.append(('Up', 1))
        else:
            instructions.append(('Up', current_location[0] - target_cell[0]))
            instructions.append(('Down', 1))
    return instructions

if __name__ == '__main__':
    current_map_location = (19, 7)
    map_array = [['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '■', '□', '■', '□', '□', '□'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□']]
    doorways = detect_doorways(map_array)
    closest_doorway_location = closest_doorway(current_map_location, doorways)
    target_cell = get_target_cell(current_map_location, closest_doorway_location)
    instructions = generate_instructions(current_map_location, target_cell)

    print("instructions:", instructions)

    # Visualize the result
    for r, row in enumerate(map_array):
        for c, cell in enumerate(row):
            if (r, c) in doorways:
                print('D', end=' ')
            elif (r, c) == current_map_location:
                print('R', end=' ')
            elif (r, c) == target_cell:
                print('T', end=' ')
            else:
                print(cell, end=' ')
        print()