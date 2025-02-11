import sys
import time
import signal
import datetime
import threading
import ros_robot_controller_sdk as rrc
import sonar
import math
import map_
import threading

# 14 x 5
# 7.5 x 2 x 5
# 

DEFAULT_HIPS = 500
DEFAULT_KNEES_RIGHT = 600 #600
DEFAULT_KNEES_LEFT = 400 #400
DEFAULT_ANKLES_RIGHT = 750 #750
DEFAULT_ANKLES_LEFT = 250 #250
DEFAULT_HEAD = 0

HIPS = [1, 4, 7, 16, 13, 10]
KNEES = [2, 5, 8, 17, 14, 11]
ANKLES = [3, 6, 9, 12, 15, 18]
HEAD = 21


board = rrc.Board()
s = sonar.Sonar()
start = True

# 关闭前处理(process before closing)
def Stop(signum, frame):
    global start
    start = False
    reset()
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)




def reset(corr=True):
    move_hips(HIPS, 0, 0.3)
    move_knees(KNEES, 0, 0.3)
    move_ankles(ANKLES, 0, 0.3)
    time.sleep(0.3)
    if corr:
        move_ankles([6], 100)
        move_ankles([9], 100)
        move_ankles([12], 50)
    

def move_head(amount=0, interval=1):
    board.bus_servo_set_position(interval, [[HEAD, DEFAULT_ANKLES_RIGHT+amount]])
            

def move_ankles(ankles, amount=0, interval=1):
    for leg in ankles:
        if leg < 10:
            board.bus_servo_set_position(interval, [[leg, DEFAULT_ANKLES_LEFT-amount]])
        else:
            board.bus_servo_set_position(interval, [[leg, DEFAULT_ANKLES_RIGHT+amount]])
            
    

def move_knees(knees, amount=0, interval=1):
    for leg in knees:
        if leg < 10:
            board.bus_servo_set_position(interval, [[leg, DEFAULT_KNEES_LEFT-amount]])
        else:
            board.bus_servo_set_position(interval, [[leg, DEFAULT_KNEES_RIGHT+amount]])
    

def move_hips(hips, amount=250, interval=1):
    for hip in hips:
        if hip < 10:
            board.bus_servo_set_position(interval, [[hip, DEFAULT_HIPS+amount]])
        else:
            board.bus_servo_set_position(interval, [[hip, DEFAULT_HIPS-amount]])

def turn_left(degrees):
    reset(False)
    degrees = int(degrees * 333 / 45)
    while degrees > 0:
        turn_amount = min(333, degrees)
        degrees -= turn_amount

        move_knees(KNEES, 250)
        time.sleep(1)
        

        for i in HIPS:
            board.bus_servo_set_position(0.4, [[i, 500 + turn_amount]])
        time.sleep(0.4)
        move_knees(KNEES, 0)
        time.sleep(1)
        reset()
        if not start:
            board.bus_servo_stop([1])
            #time.sleep(1)
            print('已关闭')
            break

def turn_right(degrees):
    reset(False)
    degrees = int(degrees * 333 // 45)
    while degrees > 0:
        turn_amount = min(333, degrees)
        degrees -= turn_amount
        move_knees(KNEES, 250)
        time.sleep(1)
        

        for i in HIPS:
            board.bus_servo_set_position(0.4, [[i, 500 - turn_amount]])
            
        time.sleep(0.4)
        move_knees(KNEES, 0)
        time.sleep(1)
        reset()
        if not start:
            board.bus_servo_stop([1])
            #time.sleep(1)
            print('已关闭')
            break




def walk():
    delay = 0.1
    #GR1: hips 1, 7, 13
    #     knees 2, 8, 14
    GR1_HIPS = [1, 7, 13]
    GR1_KNEES = [2, 8, 14]
    #GR2: hips 4, 10, 16
    #     knees 5, 11, 17
    GR2_HIPS = [4, 10, 16]
    GR2_KNEES = [11, 5, 17]

    SWING = 50
    CORRECTION = 10
    LIFT = 100

    # STATE 1:
    # GR1 UP, FORWARD
    # GR2 DOWN, BACKWARD

    
    move_knees(GR2_KNEES, 0, delay)
    time.sleep(delay)
    move_knees(GR1_KNEES, LIFT, delay)
    time.sleep(delay)
    move_hips(GR1_HIPS, -SWING, delay)

    move_hips(GR2_HIPS, SWING, delay)
    # move_hips([10], CORRECTION, delay)
    time.sleep(delay)

    # reset()

    # STATE 2:
    # GR1 DOWN, BACKWARD
    # GR2 UP, FORWARD

    
    move_knees(GR1_KNEES, 0, delay)
    time.sleep(delay)
    move_knees(GR2_KNEES, LIFT, delay)
    time.sleep(delay)
    move_hips(GR1_HIPS, SWING, delay)

    move_hips(GR2_HIPS, -SWING, delay)
    move_hips([10], -CORRECTION, delay)
    time.sleep(delay)

    # reset()





def measure_distance():
    x = []
    for i in range(10):
        d = s.getDistance()
        while d == 5000:
            d = s.getDistance()
        x.append(d)
        time.sleep(0.01)
    print(x)
    x.sort()
    return x[4]


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

    
             
def go_direction(curr, to):
    if curr - to == 0:
        return
    if curr - to == 1 or curr - to == -3:
        turn_left()
        return
    if curr - to == -1 or curr - to == 3:
        turn_right()
    else:
        turn_right()
        turn_right()


## Main program

def main():
    your_map = map_.CSME301Map()

    your_map.printObstacleMap()
    your_map.costmap_size_col
    x, y, d = map(int, input("Start coords x y d:   ").split(" "))
    goal_x, goal_y, goal_d = map(int, input("Goal coords x y d:   ").split(" "))

    
    
    instructions = breadth_first_search(your_map, (x, y), (goal_x, goal_y))

    start = instructions[0]
    current_direction = d

    for c in instructions[1:]:
        if start[0] > c[0]: # UP
            go_direction(current_direction, 1)
            walk()
        elif start[0] < c[0]: #DOWN
            go_direction(current_direction, 3)
            walk()
            print(c, 3)
        elif start[1] > c[1]: # LEFT
            go_direction(current_direction, 4)
            walk()
            print(c, 4)
        else:
            go_direction(current_direction, 2)
            walk()
            print(c, 2)

        start = c


if __name__ == "__main__":
    main()