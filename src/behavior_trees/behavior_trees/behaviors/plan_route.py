import numpy as np
from PIL import Image
from collections import deque
import py_trees


class PlanRoute(py_trees.behaviour.Behaviour):
    def __init__(self, node, blackboard, start_location, re_plan_flag = False, name="Plan Route"):
        super(PlanRoute, self).__init__(name)
        self.node = node
        self.blackboard = blackboard
        self.start_location = start_location
        self.re_plan_flag = re_plan_flag

    def initialise(self):
        self.node.get_logger().info("Planning Route")
        if not self.re_plan_flag:
            self.map_array = pgm_to_binary_2d_array('src/behavior_trees/behavior_trees/resources/altered_map.pgm', 9)
            self.path_instructions = vacuum_path(self.map_array, self.start_location)
            self.encoded_instructions = encode_instructions(self.path_instructions, 1)
            map = [['□' if cell == 0 else '■' for cell in row] for row in self.map_array]
            map[self.start_location[0]][self.start_location[1]] = 'R'
            self.blackboard.set("path_instructions", self.encoded_instructions)
            self.blackboard.set("map", map)
        else:
            self.map_array = self.blackboard.get("map")
            self.map_array = [[0 if cell == '□' else 1 for cell in row] for row in self.map_array]
            self.path_instructions = vacuum_path(self.map_array, self.start_location)
            self.encoded_instructions = encode_instructions(self.path_instructions, 1)
            self.blackboard.set("path_instructions", self.encoded_instructions)
            self.node.get_logger().info(f"ROUTE RE-PLANNED \n {self.encoded_instructions}")
            
    def update(self):
        if self.encoded_instructions:
            self.node.get_logger().info("Route planned")
            return py_trees.common.Status.SUCCESS
        else:
            self.node.get_logger().info("Failed to plan route")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass


def pgm_to_binary_2d_array(file_path, tile_size=9):
    
    image = Image.open(file_path)
    image_array = np.array(image)
    height, width = image_array.shape
    
    new_height = height // tile_size
    new_width = width // tile_size
    
    binary_array = np.zeros((new_height, new_width), dtype=int)
    
    for i in range(new_height):
        for j in range(new_width):
            # Look at a tile_size x tile_size block of pixels in the original image
            block = image_array[i*tile_size:(i+1)*tile_size, j*tile_size:(j+1)*tile_size]
            
            # Mark blocks with all pixels close to white as free space, otherwise as obstacles
            if np.all(block > 250):
                binary_array[i, j] = 0
            else:
                binary_array[i, j] = 1
                
    return binary_array

def vacuum_path(map_array, start):

    if start[0] >= len(map_array) or start[1] >= len(map_array[0]):
        raise ValueError("Start position is out of bounds")
    if start[0] < 0 or start[1] < 0:
        raise ValueError("Start position is out of bounds")
    if map_array[start[0]][start[1]] == 1:
        raise ValueError("Start position is on an obstacle")
    
    rows, cols = len(map_array), len(map_array[0])
    visited = set()
    path = []
    
    # Directions relative to map array where (0, 0) is the top-left corner and is indexed by (row, col)
    directions = [("Up", -1, 0), ("Right", 0, 1), ("Down", 1, 0), ("Left", 0, -1)]
    
    def is_valid(x, y):
        return 0 <= x < rows and 0 <= y < cols and map_array[x][y] == 0
    
    def bfs(start_x, start_y):
        queue = deque([(start_x, start_y, [])])
        local_visited = set()
        while queue:
            x, y, local_path = queue.popleft()
            if (x, y) in visited:
                continue
            visited.add((x, y))
            local_visited.add((x, y))
            for direction, dx, dy in directions:
                nx, ny = x + dx, y + dy
                if is_valid(nx, ny) and (nx, ny) not in local_visited:
                    new_path = local_path + [direction]
                    if (nx, ny) not in visited:
                        return new_path
                    queue.append((nx, ny, new_path))
        return local_path

    # Main planning loop
    current_x, current_y = start
    while True:
        local_path = bfs(current_x, current_y)
        if not local_path:
            break
        path.extend(local_path)
        # Update current position
        for direction in local_path:
            dx, dy = next((dx, dy) for dir, dx, dy in directions if dir == direction)
            current_x += dx
            current_y += dy
        
        # Check if all free spaces are visited
        if all((i, j) in visited or map_array[i][j] == 1 for i in range(rows) for j in range(cols)):
            break
        
        # Find nearest unvisited free space
        unvisited = [(i, j) for i in range(rows) for j in range(cols) 
                     if is_valid(i, j) and (i, j) not in visited]
        if not unvisited:
            break
        
        target = min(unvisited, key=lambda p: abs(p[0]-current_x) + abs(p[1]-current_y))
        path_to_target = navigate_to(current_x, current_y, target[0], target[1], map_array)
        path.extend(path_to_target)
        
        # Update current position
        for direction in path_to_target:
            dx, dy = next((dx, dy) for dir, dx, dy in directions if dir == direction)
            current_x += dx
            current_y += dy
    
    # Return to start
    path_to_start = navigate_to(current_x, current_y, start[0], start[1], map_array)
    path.extend(path_to_start)
    
    return path

def navigate_to(start_x, start_y, end_x, end_y, map_array):
    rows, cols = len(map_array), len(map_array[0])
    queue = deque([(start_x, start_y, [])])
    local_visited = set()
    
    directions = [("Up", -1, 0), ("Right", 0, 1), ("Down", 1, 0), ("Left", 0, -1)]
    
    while queue:
        x, y, path = queue.popleft()
        if (x, y) == (end_x, end_y):
            return path
        
        if (x, y) in local_visited:
            continue
        
        local_visited.add((x, y))
        
        for direction, dx, dy in directions:
            nx, ny = x + dx, y + dy
            if 0 <= nx < rows and 0 <= ny < cols and map_array[nx][ny] == 0:
                queue.append((nx, ny, path + [direction]))
    
    return []  # This should never happen if the map is fully connected

# Instruction limit is used to group identical instructions. I.e. if the robot moves forward 3 times, it will be encoded as ("Up", 3). Set to 1
# to encode each individual instruction separately because if error is detected it's most recent successful navigation will at max be 1 grid cell away
def encode_instructions(path_instructions, instruction_limit=1):
    if not path_instructions:
        return []

    encoded_instructions = []
    current_instruction = path_instructions[0]
    count = 1
    
    for i in range(1, len(path_instructions)):
        if path_instructions[i] == current_instruction and count < instruction_limit:
            count += 1
        else:
            encoded_instructions.append((current_instruction, count))
            current_instruction = path_instructions[i]
            count = 1
    
    # Add the last instruction and its count
    encoded_instructions.append((current_instruction, count))
    
    return encoded_instructions

def visualize_path(map_array, start, instructions):
    rows, cols = len(map_array), len(map_array[0])
    
    visual_map = [['□' if cell == 0 else '■' for cell in row] for row in map_array]
    
    moves = {
        'Up': (-1, 0),
        'Down': (1, 0),
        'Left': (0, -1),
        'Right': (0, 1)
    }
    
    x, y = start
    visual_map[x][y] = 'S'

    for i, instruction in enumerate(instructions):
        dx, dy = moves[instruction]
        x, y = x + dx, y + dy
        
        if 0 <= x < rows and 0 <= y < cols:
            if visual_map[x][y] == '□':
                visual_map[x][y] = str(i % 10)
            elif visual_map[x][y] == 'S':
                visual_map[x][y] = 'E'
        else:
            print(f"Warning: Instruction {i} ({instruction}) leads out of bounds")
    
    for row in visual_map:
        print(' '.join(row))
    
    print("\nLegend:")
    print("S: Start position")
    print("E: End position (if different from start)")
    print("□: Unvisited free space")
    print("■: Obstacle")
    print("0-9: Path taken (cyclic)")

if __name__ == '__main__':
    # Uncomment for easier testing
    """ map = [
        [0, 1, 0, 0, 0],
        [0, 1, 1, 1, 0],
        [0, 0, 0, 1, 0],
        [1, 1, 0, 1, 0],
        [0, 0, 0, 0, 0]
    ] """

    # Tile size set to 9 - same as in main vacuum behavior tree
    map = pgm_to_binary_2d_array('src/behavior_trees/behavior_trees/resources/altered_map.pgm', 9)
    for row in map:
        for cell in row:
            print(cell, end=' ')
        print()
    start = (1, 1)
    path_instructions = vacuum_path(map, start)

    print(encode_instructions(path_instructions, 1))
    visualize_path(map, start, path_instructions)
    print(len(map), len(map[0]))
