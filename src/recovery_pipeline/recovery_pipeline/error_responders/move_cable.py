import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from rclpy.action import ActionClient
from action_interfaces.action import ExecuteRecoveryTree
import ast
import heapq

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
        map_array = current_blackboard_state["map"]

        # Inject cable into map for testing at (18, 3), (18, 3), (18, 4), (18, 5), (18, 6), (18, 7)
        map_array[18][2] = 'C'
        map_array[18][3] = 'C'
        map_array[18][4] = 'C'
        map_array[18][5] = 'C'
        map_array[18][6] = 'C'
        map_array[18][7] = 'C'

        longest_paths = longest_accessible_side_adjacent_to_cable(map_array)
        starting_cells = best_side_to_move(map_array, longest_paths)
        instructions = generate_instructions(map_array, starting_cells, current_map_location)


        if instructions is not None:
            tree_to_execute = 'move_cable'
            self.send_execute_recovery_tree_request(tree_to_execute, instructions)
        else:
            error_responder_msg = String()
            self.get_logger().error('Failed to correct error - manual intervention required. Press ENTER when robot is ready to proceed')
            input()
            error_responder_msg.data = 'finished'
            self.publisher.publish(error_responder_msg)

def longest_accessible_side_adjacent_to_cable(map_array):
    if not map_array or not map_array[0]:
        return []

    rows, cols = len(map_array), len(map_array[0])
    directions = [(0, 1, 'Left'), (0, -1, 'Right'), (1, 0, 'Up'), (-1, 0, 'Down')]
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
                    new_x, new_y = i + dx, j + dy
                    if is_valid(new_x, new_y) and is_accessible(new_x, new_y):
                        path = find_path(new_x, new_y, -dx, -dy)
                        if len(path) > max_length:
                            max_length = len(path)
                            longest_paths = [(path, direction)]
                        elif len(path) == max_length:
                            longest_paths.append((path, direction))

    pre_result = {}
    result = {}
    for path, direction in longest_paths:
        if direction not in pre_result:
            pre_result[direction] = path
        else:
            pre_result[direction].extend(path)
    
    for direction in pre_result:
        pre_result[direction] = list(set(pre_result[direction]))
    if len(pre_result) == 0:
        return []
    max_length = max(len(pre_result[direction]) for direction in pre_result)

    for direction in pre_result:
        if len(pre_result[direction]) >= max_length:
            result[direction] = pre_result[direction]

    return result

def best_side_to_move(map_array, accessible_sides):
    # find the number of cells (area) between the accessible path and the closest wall or already traversed cell
    if len(accessible_sides) == 0:
        return None
    elif len(accessible_sides) == 1:
        return accessible_sides
    
    # Want largest as we will be pushing the cable, so want robot to be in the larger area pushing towards the smaller area
    largest_area = 0
    current_largest = None
    row, col = len(map_array), len(map_array[0])
    for direction in accessible_sides:
        path = accessible_sides[direction]
        cell_count = 0
        count = 0

        for i in range(len(path)):
            while True:
                if direction == 'Left':
                    if path[i][1] + count >= col:
                        break
                    elif map_array[path[i][0]][path[i][1] + count] == '■' or map_array[path[i][0]][path[i][1] + count] == 'X':
                        break
                elif direction == 'Right':
                    if path[i][1] - count < 0:
                        break
                    elif map_array[path[i][0]][path[i][1] - count] == '■' or map_array[path[i][0]][path[i][1] - count] == 'X':
                        break
                elif direction == 'Up':
                    if path[i][0] + count >= row:
                        break
                    elif map_array[path[i][0] + count][path[i][1]] == '■' or map_array[path[i][0] + count][path[i][1]] == 'X':
                        break
                elif direction == 'Down':
                    if path[i][0] - count < 0:
                        break
                    elif map_array[path[i][0] - count][path[i][1]] == '■' or map_array[path[i][0] - count][path[i][1]] == 'X':
                        break
                cell_count += 1
                count += 1
            count = 0

        if cell_count > largest_area:
            largest_area = cell_count
            current_largest = direction
        cell_count = 0

    best_side = accessible_sides[current_largest]

    # Order best_side to be in sequential order
    if current_largest in ['Up', 'Down']:
        best_side = sorted(best_side, key=lambda x: x[1], reverse=False)
    elif current_largest in ['Left', 'Right']:
        best_side = sorted(best_side, key=lambda x: x[0], reverse=False)
    
    return (current_largest, best_side)

def generate_instructions(map_array, starting_cells, current_map_location):
    instructions = []
    original_starting_position = current_map_location
    
    if starting_cells is None:
        return None

    def is_valid_position(pos):
        row, col = pos
        return (0 <= row < len(map_array) and
                0 <= col < len(map_array[0]) and
                (map_array[row][col] != '■' and map_array[row][col] != 'C'))

    def get_next_position(pos, direction):
        row, col = pos
        if direction == 'Up':
            return (row - 1, col)
        elif direction == 'Down':
            return (row + 1, col)
        elif direction == 'Left':
            return (row, col - 1)
        elif direction == 'Right':
            return (row, col + 1)

    def heuristic(a, b):
        return abs(b[0] - a[0]) + abs(b[1] - a[1])

    # A* algorithm to generate path (Pseudocode taken from https://en.wikipedia.org/wiki/A*_search_algorithm)
    def generate_path(start, target):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
        close_set = set()
        came_from = {}
        gscore = {start:0}
        fscore = {start:heuristic(start, target)}
        open_heap = []
        heapq.heappush(open_heap, (fscore[start], start))
        
        while open_heap:
            current = heapq.heappop(open_heap)[1]
            
            if current == target:
                path = []
                while current in came_from:
                    prev = came_from[current]
                    if prev[0] < current[0]:
                        path.append(('Down', 1))
                    elif prev[0] > current[0]:
                        path.append(('Up', 1))
                    elif prev[1] < current[1]:
                        path.append(('Right', 1))
                    else:
                        path.append(('Left', 1))
                    current = prev
                path.reverse()
                return path

            close_set.add(current)

            for i, j in neighbors:
                neighbor = current[0] + i, current[1] + j
                tentative_g_score = gscore[current] + 1
                if not is_valid_position(neighbor):
                    continue
                
                if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                    continue
                
                if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in open_heap]:
                    came_from[neighbor] = current
                    gscore[neighbor] = tentative_g_score
                    fscore[neighbor] = gscore[neighbor] + heuristic(neighbor, target)
                    heapq.heappush(open_heap, (fscore[neighbor], neighbor))
        
        return None

    first_cell = starting_cells[1][0]
    path_to_start = generate_path(current_map_location, first_cell)
    if path_to_start:
        instructions.extend(path_to_start)

    def sweep_push(cells, advance_direction, sweep_direction):
        opposite_direction = {'Up': 'Down', 'Down': 'Up', 'Left': 'Right', 'Right': 'Left'}

        for i in range(len(cells)):
            if i == len(cells) - 1:
                instructions.append((advance_direction, 1))
            else:
                instructions.append((advance_direction, 1))
                instructions.append((opposite_direction[advance_direction], 1))
                instructions.append((sweep_direction, 1))
        
        new_cells = []
        for i in range(len(cells)):
            # Hacky - cells[i] not always correct position of robot
            temp = get_next_position(cells[i], advance_direction)
            if map_array[temp[0]][temp[1]] == 'X' or map_array[temp[0]][temp[1]] == '□' or map_array[temp[0]][temp[1]] == 'C':
                new_cells.append(temp)

        new_cells.reverse()
        return new_cells

    def finished_push(cells, advance_direction):
        for i in range(len(cells) - 1):
            next_pos = get_next_position(cells[i], advance_direction)
            if map_array[next_pos[0]][next_pos[1]] != '■' and map_array[next_pos[0]][next_pos[1]] != 'X':
                return False
        return True
    
    def calculate_sweep_direction(start, end):
        if start[0] == end[0]:
            return 'Left' if start[1] > end[1] else 'Right'
        elif start[1] == end[1]:
            return 'Up' if start[0] > end[0] else 'Down'

    # Begin sweeping push
    cells = starting_cells[1]
    finished = False
    while not finished:
        sweep_direction = calculate_sweep_direction(cells[0], cells[-1])
        cells = sweep_push(cells, starting_cells[0], sweep_direction)
        finished = finished_push(cells, starting_cells[0])

    # Once finished, move back to the starting position
    print(cells[0])
    path_to_start = generate_path(cells[0], original_starting_position)
    if path_to_start:
        instructions.extend(path_to_start)
    print(instructions)
    return instructions
    

if __name__ == '__main__':
    current_map_location = (15, 7)
    map_array = [['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '■', '□', '□', '■', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '■', '■', '□', '■', '■', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', 'C', 'C', 'C', 'C', 'C', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '□', '□', '□', '□', '□', '□', '□', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '■', '□', '■', '□', '□', '□'], ['□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '■', '■', '■', '■', '■', '■', '■'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□'], ['■', '■', '■', '■', '■', '■', '■', '■', '■', '□', '□', '□', '■', '■', '□', '□', '□', '□', '□', '□', '□']]
    longest_paths = longest_accessible_side_adjacent_to_cable(map_array)
    starting_cells = best_side_to_move(map_array, longest_paths)
    instructions = generate_instructions(map_array, starting_cells, current_map_location)

    # Visualize the result
    for r, row in enumerate(map_array):
        for c, cell in enumerate(row):
            if current_map_location == (r, c):
                print('R', end=' ')
            else:
                print(cell, end=' ')
        print()