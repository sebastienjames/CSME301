#!/usr/bin/env python
import sys

# Usage:
#
# #Create object
# your_map = CSME301Map()
#
# #Use Object (examples)
# your_map.printObstacleMap()
# your_map.clearObstacleMap()
# your_map.printCostMap()
# your_map.setObstacle(3, 4, 1, DIRECTION.North)
# isBlocked = your_map.getObstacle(3, 4, DIRECTION.North)
# cell_cost = your_map.getCost(3, 4)

def enum(**enums):
    return type('Enum', (), enums)

DIRECTION = enum(North=1, East=2, South=3, West=4)

class CSME301Map():
    def __init__(self):

        n_row = 4
        n_col = 6

        self.obstacle_size_row = n_row
        self.obstacle_size_col = n_col
        self.costmap_size_row = n_row
        self.costmap_size_col = n_col
        
        self.horizontalWalls = [[0 for x in range(n_col)] for x in range(n_row+1)]
        self.verticalWalls = [[0 for x in range(n_col+1)] for x in range(n_row)]
        self.costMap = [[0 for x in range(n_col)] for x in range(n_row)]

        # from IPython import embed; embed()
        
        for i in range(n_row):
            for j in range(n_col):
                self.costMap[i][j] = 0
        
        self.horizontalWalls[0][0] = 1
        self.horizontalWalls[0][1] = 1
        self.horizontalWalls[0][2] = 1
        self.horizontalWalls[0][3] = 1
        self.horizontalWalls[0][4] = 1
        self.horizontalWalls[0][5] = 1

        self.horizontalWalls[1][0] = 0
        self.horizontalWalls[1][1] = 1
        self.horizontalWalls[1][2] = 0
        self.horizontalWalls[1][3] = 1
        self.horizontalWalls[1][4] = 1
        self.horizontalWalls[1][5] = 0
        
        self.horizontalWalls[2][0] = 0
        self.horizontalWalls[2][1] = 0
        self.horizontalWalls[2][2] = 0
        self.horizontalWalls[2][3] = 0
        self.horizontalWalls[2][4] = 0
        self.horizontalWalls[2][5] = 0

        self.horizontalWalls[3][0] = 1
        self.horizontalWalls[3][1] = 1
        self.horizontalWalls[3][2] = 0
        self.horizontalWalls[3][3] = 0
        self.horizontalWalls[3][4] = 0
        self.horizontalWalls[3][5] = 0

        self.horizontalWalls[4][0] = 1
        self.horizontalWalls[4][1] = 1
        self.horizontalWalls[4][2] = 1
        self.horizontalWalls[4][3] = 1
        self.horizontalWalls[4][4] = 1
        self.horizontalWalls[4][5] = 1

        self.verticalWalls[0][0] = 1
        self.verticalWalls[0][1] = 0
        self.verticalWalls[0][2] = 0
        self.verticalWalls[0][3] = 1
        # self.verticalWalls[0][4] = 1
        self.verticalWalls[0][4] = 0  # no wall on the right side of the map 
        self.verticalWalls[0][5] = 0
        self.verticalWalls[0][6] = 1

        self.verticalWalls[1][0] = 1
        self.verticalWalls[1][1] = 1
        self.verticalWalls[1][2] = 1
        self.verticalWalls[1][3] = 1
        self.verticalWalls[1][4] = 0
        self.verticalWalls[1][5] = 1
        self.verticalWalls[1][6] = 1

        self.verticalWalls[2][0] = 1
        self.verticalWalls[2][1] = 0
        self.verticalWalls[2][2] = 1
        self.verticalWalls[2][3] = 1
        self.verticalWalls[2][4] = 1
        self.verticalWalls[2][5] = 1
        self.verticalWalls[2][6] = 1

        self.verticalWalls[3][0] = 1
        self.verticalWalls[3][1] = 0
        self.verticalWalls[3][2] = 0
        self.verticalWalls[3][3] = 0
        self.verticalWalls[3][4] = 1
        self.verticalWalls[3][5] = 0
        self.verticalWalls[3][6] = 1

        
    # ***********************************************************************
    # Function Name : getNeighborObstacle
    # Description   : Checks if the neighboring cell is blocked on the map.
    # Input         : i: The row coordinate of the current cell on the map.
    #               : j: The column coordinate of the current cell on the map
    #               : dir: A Direction enumeration (North, South, East, West)
    #               :      indicating which neighboring cell to check for
    #               :      obstacles
    # Output        : None
    # Return        : 1 if neighboring cell is blocked, 0 if neighboring cell
    #               : is clear, -1 if index i or j is out of bounds
    # ***********************************************************************/
    def getNeighborObstacle(self, i, j, dir):
        if (((i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j >  (self.costmap_size_col))
             and (dir == DIRECTION.West or dir == DIRECTION.East)) 
            and ((j < 0 or j > (self.costmap_size_col - 1) or i < 0 or i > self.costmap_size_row)
                 and (dir == DIRECTION.North or dir == DIRECTION.South))):
            print("ERROR (getNeighborObstacle): index out of range")
            return -1

        isBlocked = 0
        if dir == DIRECTION.North:
            isBlocked = self.horizontalWalls[i][j]
        elif dir == DIRECTION.South:
            isBlocked = self.horizontalWalls[i+1][j]
        elif dir == DIRECTION.West:
            isBlocked = self.verticalWalls[i][j]
        elif dir == DIRECTION.East:
            isBlocked = self.verticalWalls[i][j+1]

        return isBlocked

    # ******************************************************************************
    # Function Name  : setObstacle
    # Description    : Used for map building, sets the obstacle status of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : isBlocked: A boolean (0 or 1) value indicated if the cell is blocked
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to set for obstacles
    # Output         : None
    # Return         : 0 if successful, -11 if i or j is out of map bounds, -2 if isBlocked is not 0 or 1
    # *****************************************************************************/
    def setObstacle(self, i, j, isBlocked, dir):
        if (((i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col))
             and (dir == DIRECTION.West or dir == DIRECTION.East))
             or ((j < 0 or j > (self.costmap_size_col - 1) or i < 0 or i > (self.costmap_size_row))
                 and (dir == DIRECTION.North or dir == DIRECTION.South))):
            print("ERROR (setObstacle): index out of range, obstacle not set")
            return -1

        if isBlocked > 1:
            print("ERROR (setObstacle): isBlocked not a valid input, obstacle not set")
            return -2

        if dir == DIRECTION.North:
            self.horizontalWalls[i][j] = isBlocked
        elif dir == DIRECTION.South:
            self.horizontalWalls[i+1][j] = isBlocked
        elif dir == DIRECTION.West:
            self.verticalWalls[i][j] = isBlocked
        elif dir == DIRECTION.East:
            self.verticalWalls[i][j+1] = isBlocked

        return 0

    # ******************************************************************************
    # Function Name  : getNeighborCost
    # Description    : Retrieves the calculated cost of a neighboring cell on the map.
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to retrieve the cost.
    # Output         : None
    # Return         : Positive float valued cost for the neighboring cell, -1 on error
    # *****************************************************************************/
    def getNeighborCost(self, i, j, dir):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (getNeighborCost): index out of range")
            return -1

        cellValue = 0
        if dir == DIRECTION.North:
            if (i == 0):
                cellValue = 1000
            else:
                cellValue = self.costMap[i-1][j]
        elif dir == DIRECTION.South:
            if(i == (self.costmap_size_row - 1)):
                cellValue = 1000
            else:
                cellValue = self.costMap[i+1][j]
        elif dir == DIRECTION.West:
            if (j == 0):
                cellValue = 1000
            else:
                cellValue = self.costMap[i][j-1]
        elif dir == DIRECTION.East:
            if (j == (self.costmap_size_col - 1)):
                cellValue = 1000
            else:
                cellValue = self.costMap[i][j+1]

        return cellValue

    # ******************************************************************************
    # Function Name  : setNeighborCost
    # Description    : Sets the calculated cost of a neighboring cell on the map.
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : dir: A Direction enumeration (North, South, East, West) indicating
    #                :      which neighboring cell to retrieve the cost.
    #                : val: Positive float valued cost for the neighboring cell
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def setNeighborCost(self, i, j, dir, val):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (setNeighborCost): index out of range, value not set")
            return

        if dir == DIRECTION.North:
            if (i > 0):
                self.costMap[i-1][j] = val
        elif dir == DIRECTION.South:
            if (i < (self.costmap_size_row - 1)):
                self.costMap[i+1][j] = val
        elif dir == DIRECTION.West:
            if (j > 0):
                self.costMap[i][j-1] = val
        elif dir == DIRECTION.East:
            if (j < (self.costmap_size_col - 1)):
                self.costMap[i][j+1] = val

    # ******************************************************************************
    # Function Name  : setCost
    # Description    : Used for map building, sets the calculated cost of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    #                : val: An integer value (0 to 1023) indicated the cost of a map cell
    # Output         : None
    # Return         : 0 if successful, -1 if i or j is out of map bounds
    # *****************************************************************************/
    def setCost(self, i, j, val):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (setCost): index out of range")
            return -1

        self.costMap[i][j] = val
        return 0

    # ******************************************************************************
    # Function Name  : getCost
    # Description    : Used for map building, gets the calculated cost of a given map cell
    # Input          : i: The row coordinate of the current cell on the map.
    #                : j: The column coordinate of the current cell on the map
    # Output         : None
    # Return         : cost >= 0 if successful, -1 if i or j is out of map bounds
    # *****************************************************************************/
    def getCost(self, i, j):
        if (i < 0 or i > (self.costmap_size_row - 1) or j < 0 or j > (self.costmap_size_col - 1)):
            print("ERROR (getCost): index out of range")
            return -1

        return self.costMap[i][j]

    # ******************************************************************************
    # Function Name  : clearCostMap
    # Description    : Sets all of the values in the cost map to 0
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def clearCostMap(self):
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                self.costMap[i][j] = 0

    # ******************************************************************************
    # Function Name  : clearObstacleMap
    # Description    : Sets all of the values in the obstacle map to 0
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def clearObstacleMap(self):
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col + 1):
                self.verticalWalls[i][j] = 0

        for i in range(self.costmap_size_row + 1):
            for j in range(self.costmap_size_col):
                self.horizontalWalls[i][j] = 0

    # ******************************************************************************
    # Function Name  : printCostMap
    # Description    : When connected to a terminal, will print out the 4x6 cost map
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def printCostMap(self):
        print("Cost Map:")
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                print(str(self.costMap[i][j])),
            # from IPython import embed; embed()

            print(" ")

    # ******************************************************************************
    # Function Name  : printObstacleMap
    # Description    : When connected to a terminal, will print out the 4x6 obstacle map
    # Input          : None
    # Output         : None
    # Return         : None
    # *****************************************************************************/
    def printObstacleMap(self):
        print("Obstacle Map: ")
        for i in range(self.costmap_size_row):
            for j in range(self.costmap_size_col):
                if (self.horizontalWalls[i][j] == 0):
                    if i == 0:
                        sys.stdout.write(" ---")
                    else:
                        sys.stdout.write("    ")
                else:
                    sys.stdout.write(" ---")

            print(" ")
            for j in range(self.costmap_size_col):
                if (self.verticalWalls[i][j] == 0):
                    if j == self.costmap_size_col - 1:
                        sys.stdout.write("  O |")
                    elif j == 0:
                        sys.stdout.write("| O ")
                    else:
                        sys.stdout.write("  O ")
                else:
                    if j == self.costmap_size_col - 1:
                        sys.stdout.write("| O |")
                    else:
                        sys.stdout.write("| O ")
            print(" ")
        for j in range(self.costmap_size_col):
                sys.stdout.write(" ---")
        print(" ")

    # ******************************************************************************
    # Function Name  : getCostmapSize
    # Description    : Retrieve the size of a given dimension of the costmap
    # Input          : bool rowDim (true for row dimension, false for column dimension)
    # Output         : None
    # Return         : costmap size in the requested dimension
    # *****************************************************************************/
    def getCostmapSize(self, rowDim):
        if (rowDim):
            return self.costmap_size_row
        else:
            return self.costmap_size_col

    # ******************************************************************************
    # Function Name  : getObstacleMapSize
    # Description    : Retrieve the size of a given dimension of the Obstacle Map
    # Input          : bool rowDim (true for row dimension, false for col dimension)
    # Output         : None
    # Return         : obstacle map size in the requested dimension
    # *****************************************************************************/
    def getObstacleMapSize(self, rowDim):
        if rowDim:
            return self.obstacle_size_row
        else:
            return self.obstacle_size_col

def breadth_first_search(time_map, start, end):
	queue = deque([(start,[start])])
	visited = set()

	if start == end:
		return [start]

	while queue:
		node, path = queue.popleft()

		if node == end:
			return path
		
		if node not in visited:
			visited.add(node)

		for neighbor in expand(node, time_map):
			if neighbor not in visited:

				visited.add(neighbor)
				queue.append((neighbor, path + [neighbor]))

	return None

def get_neighbors(m, node):
    n = []
    # print(i, j)
    for d in range(1, 5):
        if m.getNeighborObstacle(node[0], node[1], d) == 0:
            if d == 1:
                # print("UP")
                if node[0]-1 >= 0:
                    n.append((node[0]-1, node[1]))
            elif d == 2:
                # print("RIGHT")
                if node[1]+1 < m.costmap_size_col:
                    # print("YAY")
                    n.append((node[0], node[1]+1))
            elif d == 3:
                # print("DOWN")
                if node[0]+1 < m.costmap_size_row:
                    # print("YAY")
                    n.append((node[0]+1, node[1]))
            else:
                # print("LEFT")
                if node[1]-1 >= 0:
                    # print("YAY")
                    n.append((node[0], node[1]-1))

    return n

def breadth_first_search(time_map, start, end):
	queue = [(start,[start])]
	visited = set()

	if start == end:
		return [start]

	while queue:
		node, path = queue.pop(0)

		if node == end:
			return path
		
		if node not in visited:
			visited.add(node)

		for neighbor in get_neighbors(time_map, node):
			if neighbor not in visited:

				visited.add(neighbor)
				queue.append((neighbor, path + [neighbor]))

	return None

def get_directions(instructions):

    start = instructions[0]
    for c in instructions[1:]:
        if start[0] > c[0]: # UP
            # turn up
            # walk
            print(c, 1)
            pass
        elif start[0] < c[0]: #DOWN
            # turn down
            # walk
            print(c, 3)
            pass
        elif start[1] > c[1]: # LEFT
            # turn left
            # walk
            print(c, 4)
            pass
        else:
            # turn right
            # walk
            print(c, 2)
            pass

        start = c
             

def main():
    your_map = CSME301Map()

    your_map.printObstacleMap()
    your_map.costmap_size_col
    x, y, d = 0, 0, 3 #map(int, input("Start coords x x x:   ").split(" "))
    goal_x, goal_y, goal_d = 0,3 , 4

    # print(get_neighbors(your_map, 0, 1))
    
    instructions = breadth_first_search(your_map, (x, y), (goal_x, goal_y))

    print(get_directions(instructions))


if __name__ == "__main__":
    main()