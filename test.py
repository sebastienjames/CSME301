import time


directions = {1: "North", 2: "East", 3: "South", 4: "West"}
dirs = {"North":[-3,1,5], "East":[-2,2,6], "South":[-1,3,7], "West":[0,4,8]}
dir_num = {-3:1, 1:1, 5:1, -2:2, 2:2, 6:2, -1:3, 3:3, 7:3, 0:4, 4:4, 8:4}


#3x3 maze, T map, key represents the tile, value represents[ wall north, wall east, wall south, wall west]
walls = {(1,0): [0,1,0,1], (1,1): [0,1,0,1], (1,2):[1,0,0,0], (0,2): [1,0,1,1], (2,2):[1,1,1,0] }


def sensor_turn(tile):
    for wall in walls[tile]:
        if wall or dir == 4:
            return False
        else:
            return True


def get_tiles(start_dir, pos):
    valid_tiles = []
    print("currently facing", directions[start_dir])

    front = [-1,0,1]

    for dir in front:
        curr_dir = (start_dir+dir)
        curr_dir = dir_num[curr_dir]

        print("checking", directions[curr_dir])

        #print(sensor_turn(curr_dir))
        print(walls[pos])

        wall = walls[pos][curr_dir-1]


        if wall == 0:
            print("no wall")
            if curr_dir == 1:
                valid_tiles.append((curr_dir, (pos[0],pos[1]+1)))
            elif curr_dir == 3:
                valid_tiles.append((curr_dir, (pos[0], pos[1]-1)))
            elif curr_dir == 2:
                valid_tiles.append((curr_dir, (pos[0]+1,pos[1])))
            elif curr_dir == 4:
                valid_tiles.append((curr_dir, (pos[0]-1, pos[1])))
        else:
            print("wall")

    return valid_tiles


def execute_commands(instructions, current_direction):
    start = instructions[0]

    for c in instructions[1:]:
        #print("Current direction", directions[current_direction])

        if start[0] > c[0]: # UP
            # go_direction(current_direction, 1)
            # walk()
            print("Going up", c, 1)
        elif start[0] < c[0]: #DOWN
            # go_direction(current_direction, 3)
            # walk()
            print("Going down", c, 3)
        elif start[1] > c[1]: # LEFT
            # go_direction(current_direction, 4)
            # walk()
            print("Going left", c, 4)
        else:
            # go_direction(current_direction, 2)
            # walk()
            print("Going right", c, 2)

        start = c

def execute_commands2(instructions, current_direction):
    if len(instructions) == 1:
        return
        

def dfs_implement(start, end):
    orientation = 1 #north

    stack = [(start, orientation, [start])]
    visited = set()

    if start == end:
        return [start]
    
    while stack:
        tile, orientation, path = stack.pop()

        print("###At tile", tile)

        if tile == end:
            print(path)

            return path
        
        if tile not in visited:
            visited.add(tile)

        tiles = get_tiles(orientation, tile)

        #print(tiles)

        if tiles == None:
            print("dead end")
            execute_commands([path[-1], path[-2]], orientation)

        
        for valid_orientation, valid_tile in tiles:
            if valid_tile not in visited:
                visited.add(tile)
                stack.append((valid_tile, valid_orientation, path + [valid_tile]))

        if tile != start:
            execute_commands([path[-2], path[-1]], orientation)




dfs_implement((1,0),(2,2))