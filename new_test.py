import time

# Direction mappings
directions = {1: "North", 2: "East", 3: "South", 4: "West"}

# Movement mapping: (x, y)
move_offsets = {
    1: (0, 1),  # North (+y)
    2: (1, 0),  # East (+x)
    3: (0, -1), # South (-y)
    4: (-1, 0)  # West (-x)
}

# Simulated Known Walls (Initially Unknown)
known_walls = {}

# Simulated "sensor" function (real robots would use actual sensor readings)
def scan_tile(pos):
    """Simulates scanning the surroundings upon reaching a tile."""
    if pos in walls:  # Only add to known walls after reaching a tile
        known_walls[pos] = walls[pos]
        return known_walls[pos]
    return [1, 1, 1, 1]  # Assume all walls until scanned

def get_tiles(pos):
    """Scans for valid moves dynamically using the robot's sensor."""
    valid_tiles = []

    if pos not in known_walls:  # If not scanned before, scan it now
        scan_tile(pos)

    wall_data = known_walls[pos]  # Get the scanned wall information

    for direction, (dx, dy) in move_offsets.items():
        if wall_data[direction - 1] == 0:  # 0 means no wall
            new_x, new_y = pos[0] + dx, pos[1] + dy
            valid_tiles.append((direction, (new_x, new_y)))

    return valid_tiles

def execute_commands(start, current_direction):
    """Moves the robot when popping a tile from the stack."""
    print(f"Moving to {start}, facing {directions[current_direction]}")
    time.sleep(0.5)  # Simulate movement delay
    scan_tile(start)  # Scan surroundings upon arrival
    return current_direction  # Keep current direction

def explore_maze(start, end):
    """DFS-based real-time maze exploration, dynamically scanning surroundings."""
    orientation = 1  # Start facing North
    stack = [(start, orientation)]
    visited = set()

    while stack:
        tile, orientation = stack.pop()

        # Execute movement only when a tile is actually reached
        orientation = execute_commands(tile, orientation)

        print(f"### At tile {tile}, facing {directions[orientation]}")

        if tile == end:
            print("Reached destination!")
            return

        visited.add(tile)

        # Scan the surroundings every time a new tile is visited
        tiles = get_tiles(tile)

        if not tiles:  # Dead end, backtrack
            print(f"Dead end at {tile}, backtracking...")
            continue  # Skip this iteration

        # Add tiles to stack in the order they should be explored
        for valid_orientation, valid_tile in tiles:
            if valid_tile not in visited:
                stack.append((valid_tile, valid_orientation))  # Push to stack

    print("Maze fully explored or no path found.")

# Simulated "actual" maze (robot does NOT know this at the start)
walls = {
    (1, 0): [0, 1, 1, 1],  # Open North
    (1, 1): [0, 1, 0, 1],  # Open North & South
    (1, 2): [1, 0, 0, 0],  # Open East & South
    (0, 2): [1, 0, 1, 1],  # Open East
    (2, 2): [1, 1, 1, 0]   # Open West
}

# Start real-time maze exploration from (1,0) to (2,0)
explore_maze((1,0), (2,0))
