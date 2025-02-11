import sys
import time
import signal
import datetime
import threading
import ros_robot_controller_sdk as rrc
import sonar
import math

# 14 x 5
# 7.5 x 2 x 5
# 

print("START TEXT")

DEFAULT_HIPS = 500
DEFAULT_KNEES_RIGHT = 600 #600
DEFAULT_KNEES_LEFT = 400 #400
DEFAULT_ANKLES_RIGHT = 750 #750
DEFAULT_ANKLES_LEFT = 250 #250

HIPS = [1, 4, 7, 16, 13, 10]
KNEES = [2, 5, 8, 17, 14, 11]
ANKLES = [3, 6, 9, 12, 15, 18]


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







## Main program
# reset()



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

# print(measure_distance())

# reset()

# move_knees([17], 50)
# move_hips([16], -90)
# time.sleep(1)
# move_ankles([18], -150)
# move_knees([17], -50)

# time.sleep(2)

# move_hips([16], 90)
# move_ankles([18], 0)
# time.sleep(1)


# time.sleep(3)
# reset()

# Following right wall

start = datetime.datetime.now()




inital_distance = measure_distance() + 10
total_travel = 0
print(inital_distance, total_travel)

while 1:
    walk()
    total_travel += 90
    wall_distance = measure_distance()
    print("INITIAL:", inital_distance)
    print("WALL:", wall_distance)
    print(total_travel)
    if inital_distance - wall_distance < -60: # We are too far from the wall
        # print("right turn", math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi * 1.1)
        reset()
        # move_ankles([6, 9], 50)
        turn_right(math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
        time.sleep(1)
        total_travel = 0
        # inital_distance = measure_distance()
    elif inital_distance - wall_distance < 0: # We are too far from the wall
        print("small right")
        move_ankles([6, 9], 0)
    elif inital_distance - wall_distance > 60:
        # print("left turn", math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi * 1.1)
        reset()
        # move_ankles([6, 9], 50)
        turn_left(math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
        time.sleep(1)
        total_travel = 0
        # inital_distance = measure_distance()
    elif inital_distance - wall_distance > 0: # We are too close from the wall
        print("small left")
        move_ankles([6, 9], int(100 + ((inital_distance - wall_distance) * 0.5)))
    
    walk()

reset()

# Following left wall

# inital_distance = 350
# total_travel = 0
# print(inital_distance, total_travel)

# for i in range(20):
#     walk()
#     total_travel += 100
#     wall_distance = measure_distance()
#     print("INITIAL:", inital_distance)
#     print("WALL:", wall_distance)
#     print(total_travel)
#     if inital_distance - wall_distance < -20: # We are too far from the wall
#         print("right turn", math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
#         reset()
#         turn_left(math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
#         time.sleep(1)
#         inital_distance = 350
#         total_travel = 0
        
#     elif inital_distance - wall_distance > 20:
#         print("left turn", math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
#         reset()
#         turn_right(math.atan(abs(inital_distance - wall_distance) / total_travel) * 180 / math.pi)
#         time.sleep(1)
#         inital_distance = 350
#         total_travel = 0
#     walk()

# reset()


# start = 500
# n = 0
# reading = False

# for i in range(50000):
#     if start - s.getDistance() > 100:
#         # print("DETECT\r")
#         reading = True
#         if n == 0:
#             t = datetime.datetime.now()
#             print("START LISTEN")
            
#             n += 1
#         else:
#             print((datetime.datetime.now() - t).total_seconds())
#             n += 1
        
#         while s.getDistance() < 500:
#             print("waiting")
#             time.sleep(0.15)

#     if reading:
#         if (datetime.datetime.now() - t).total_seconds() > 4:
#             print("WAVE", n)
            
#             if n == 1:
#                 print("right")
#                 turn_right(90)
#             if n == 2:
#                 print("eft")
#                 turn_left(90)
#             if n == 3:
#                 print("around")
#                 turn_left(180)
            
#             n = 0
#             reading = False
        

reset(corr=False)