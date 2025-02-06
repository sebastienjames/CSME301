import sys
import time
import signal
import threading
import ros_robot_controller_sdk as rrc
import sonar

print('''
**********************************************************
********功能:幻尔科技树莓派扩展板，控制总线舵机转动(Function:Hiwonder Raspberry Pi expansion board, bus servo rotation control)**********
**********************************************************
----------------------------------------------------------
Official website:https://www.hiwonder.com
Online mall:https://hiwonder.tmall.com
----------------------------------------------------------
Tips:
 * 按下Ctrl+C可关闭此次程序运行，若失败请多次尝试！(Press "Ctrl+C" to exit the program. If it fails, please try again multiple times!)
----------------------------------------------------------
''')

board = rrc.Board()
s = sonar.Sonar()
start = True

# 关闭前处理(process before closing)
def Stop(signum, frame):
    global start
    start = False
    print('关闭中...')

signal.signal(signal.SIGINT, Stop)

hips = [1, 4, 7, 16, 13, 10]
knees = [2, 5, 8, 17, 14, 11]
ankles = [3, 6, 9, 12, 15, 18]


def reset():
    for i in range(1, 19):
        board.bus_servo_set_position(1, [[i, 500]])

def reset_legs(joints=knees):
    for i in joints:
        board.bus_servo_set_position(1, [[i, 500]])

    



def turn_left():

    for j in range(2):
        time.sleep(1)

        reset()

        time.sleep(0.1)

        t = 0.5

        for i in range(3):
            board.bus_servo_set_position(1, [[knees[i], 250]])
        for i in range(3, 6):
            board.bus_servo_set_position(1, [[knees[i], 750]])
        time.sleep(t)
        for i in hips:
            board.bus_servo_set_position(t, [[i, 795]])
        time.sleep(t)
        for i in knees:
            board.bus_servo_set_position(t, [[i, 500]])
        time.sleep(t)
        for i in range(1, 19):
            board.bus_servo_set_position(t, [[i, 500]])
        if not start:
            board.bus_servo_stop([1])
            #time.sleep(1)
            print('已关闭')
            break


def turn_right():

    for j in range(2):
        time.sleep(1)

        reset()

        time.sleep(0.1)

        for i in range(3):
            board.bus_servo_set_position(1, [[knees[i], 250]])
        for i in range(3, 6):
            board.bus_servo_set_position(1, [[knees[i], 750]])
        time.sleep(1)
        for i in hips:
            board.bus_servo_set_position(1, [[i, 225]])
        time.sleep(1)
        for i in knees:
            board.bus_servo_set_position(1, [[i, 500]])
        time.sleep(1)
        for i in range(1, 19):
            board.bus_servo_set_position(1, [[i, 500]])
        if not start:
            board.bus_servo_stop([1])
            #time.sleep(1)
            print('已关闭')
            break


def lift_legs(legs, amount=50, interval=1):
    for leg in legs:
        print(type(leg), leg)
        if leg < 10:
            board.bus_servo_set_position(interval, [[leg, 500-amount]])
        else:
            board.bus_servo_set_position(interval, [[leg, 500+amount]])

def move_legs(hips, amount=250, interval=1):
    for hip in hips:
        if hip < 10:
            board.bus_servo_set_position(interval, [[hip, 500+amount]])
        else:
            board.bus_servo_set_position(interval, [[hip, 500-amount]])

def walk():
    delay = 0.5
    #GR1: hips 1, 4, 7
    #     knees 2, 5, 8
    GR1_HIPS = [1, 13, 7]
    GR1_KNEES = [2, 14, 8]
    #GR2: hips 10, 13, 16
    #     knees 11, 14, 17
    GR2_HIPS = [10, 4, 16]
    GR2_KNEES = [11, 5, 17]

    # STATE 1:
    # GR1 UP, FORWARD
    # GR2 DOWN, BACKWARD

    lift_legs(GR1_KNEES, 200, delay)
    lift_legs(GR2_KNEES, 0, delay)
    time.sleep(delay)
    move_legs(GR1_HIPS, -50, delay)
    move_legs(GR2_HIPS, 50, delay)

    time.sleep(delay)


    # STATE 2:
    # GR1 DOWN, BACKWARD
    # GR2 UP, FORWARD

    lift_legs(GR2_KNEES, 200, delay)
    lift_legs(GR1_KNEES, 0, delay)
    time.sleep(delay)
    move_legs(GR1_HIPS, 50, delay)
    move_legs(GR2_HIPS, -50, delay)

    time.sleep(delay)







## Main program


# # Move each servo
# for servo in range(1, 19):
#     board.bus_servo_set_position(1, [[servo, 600]])
#     time.sleep(0.2)
#     board.bus_servo_set_position(1, [[servo, 500]])

# time.sleep(2)

# # # Turn left

for i in range(4):
    turn_left()

# time.sleep(2)

# # # # Turn right
# turn_right()

# time.sleep(2)

# # Sonar
# for _ in range(1000):
#     s = sonar.Sonar()
#     print(s.getDistance())
#     time.sleep(0.01)

# time.sleep(2)

# Behavior 1:
# Robot moves forward until the sonar detects an object in front of it

# for i in ankles[:3]:
#     board.bus_servo_set_position(1, [[i, 300]])
# for i in ankles[3:]:
#     board.bus_servo_set_position(1, [[i, 700]])



# while s.getDistance() > 300:
#     walk()
# reset()

# time.sleep(2)

# reset()


# # Behavior 2:
# # Robot turns until the sonar no longer detects an object

# while sonar() < 1000:
#     turn_left()

# # Behavior 3: Jump DOESN'T WORK unfortunately
# lift_legs(ankles, 100, 2)
# lift_legs(knees, 100, 2)
# time.sleep(4)
# lift_legs(ankles, -100, 0.1)
# lift_legs(knees, -200, 0.1)

# time.sleep(5)
# reset()

# Behavior 2:
# reset()

# distances = []
# min_distance = 5000
# turn = 0

# for i in range(4):
    
#     print(s.getDistance())
#     if s.getDistance() < min_distance:
#         min_distance = s.getDistance()
#         turn = i

#     turn_left()

# print(min_distance, turn)

# for i in range(turn):
#     turn_left()

# reset()