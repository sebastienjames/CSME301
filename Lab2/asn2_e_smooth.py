import sys
import time
import signal
import threading
import math
import os
import ros_robot_controller_sdk as rrc
import sonar
import map_301

import sympy as sp

board = rrc.Board()
start = True

def Stop(signum, frame):
    global start
    start = False
    print('stopping...')

signal.signal(signal.SIGINT, Stop)

# defining servos

class ServoGroup:
     
     def __init__(self, name, zero_pos, min_pos, max_pos):
        #just for printing if needed
        self.name = name
        #relative to the chassy
        self.zero_pos = zero_pos
        #avoid collisions with these limits
        self.min_pos = min_pos
        self.max_pos = max_pos
     
     def deg_to_pos(self, degree):

        #avoid all these calculations in the obvious case
        if degree == 0.00:
            pos = self.zero_pos
            return pos

        #this could be adjusted but god willing it doesn't have to be
        #degree_range = 250.00
      #  pos_range = 1000.00
     
        #how many ticks to a degree? the answer is 4. probably
        factor = 4.22
        #pos_range/degree_range
        
        #this just checks if the motor is flipped (so changing positions is opposite effect)
        flip = self.max_pos < self.min_pos
        #change is what we add to the zero degree position
        change = factor*degree
        #if motor is flipped, change should be negative
        if flip:
            change = change*-1.00
#position is zero degree position plus change in position needed to shift that many degrees from zero
#so essentially you can put the zero degree position anywhere on the solenoid's actual rotation
#this allows you to consider the angle relative to say, the floor or the chassy, by calculating at which position the motors are zero degrees relative to that object
#if you kept zero_pos as 500, you are considering zero degrees to be the middle position of the solenoid itself
        pos = self.zero_pos + int(change)

        #check for illegal (colliding) positions and break if they occur
        if self.max_pos > self.zero_pos:
            if pos > self.max_pos or pos < self.min_pos:
                # print("position illegal?", pos, "for", self.name)
               # raise Exception
               pass
        else:
         #   self.max_pos > self.zero_pos:
            if pos > self.max_pos or pos < self.min_pos:
                # print("position illegal?", pos, "for", self.name)
               # raise Exception
               pass

        # print ("pos is ", pos, "degree was", degree)
        return pos

#just to organize
class Servo:
    def __init__(self, group, name, id):
        self.name = name
        self.group = group
        self.id = id

#the zero positions are pretty well agonized over
#the collision limits are tested but could stand to retest them if time tbh
r2 = ServoGroup("right joint 2", 500, 80, 940)
r3 = ServoGroup("right joint 3", 300, 950, 0)
l2 = ServoGroup("left joint 2", 500, 970, 60)
l3 = ServoGroup("left joint 3", 700, 40, 1000)
#r1 = ServoGroup("right joint 1", zero, min, max)
#l1 = ServoGroup("left joint 1", zero, min, max)

D1 = 1
E1 = 4
F1 = 7
C1 = 10
B1 = 13
A1 = 16

#A1 = Servo(r1, "a1",16)
A2 = Servo(r2, "a2", 17)
A3 = Servo(r3, "a3", 18)
#B1 = Servo(r1, "b1",13)
B2 = Servo(r2, "b2", 14)
B3 = Servo(r3, "b3", 15)
#C1 = Servo(r1, "c1", 10)
C2 = Servo(r2, "c2", 11)
C3 = Servo(r3, "c3", 12)
#D1 = Servo(l1, "d1", 1)
D2 = Servo(l2, "d2", 2)
D3 = Servo(l3, "d3", 3)
#E1 = Servo(l1, "e1",4)
E2 = Servo(l2, "e2", 5)
E3 = Servo(l3, "e3", 6)
#F1 = Servo(l1, "f1", 7)
F2 = Servo(l2, "f2", 8)
F3 = Servo(l3, "f3", 9) 



def set_degree(servo, degree, speed):
    
     # print("speed", speed)
     # print("degree", degree)
      pos = servo.group.deg_to_pos(degree)
     # print("pos", pos)
     # print("id", servo.id)
      board.bus_servo_set_position(speed, [[servo.id,pos]])

def resetPos():
    board.bus_servo_set_position(0.25, [[A1,500]])
    board.bus_servo_set_position(0.25, [[B1,500]])
    board.bus_servo_set_position(0.25, [[C1,500]])
    board.bus_servo_set_position(0.25, [[D1,500]])
    board.bus_servo_set_position(0.25, [[E1,500]])
    board.bus_servo_set_position(0.25, [[F1,500]])
    board.bus_servo_set_position(0.25, [[D2.id,300]])
    board.bus_servo_set_position(0.25, [[E2.id,300]])
    board.bus_servo_set_position(0.25, [[F2.id,300]])
    board.bus_servo_set_position(0.25, [[A2.id,700]])
    board.bus_servo_set_position(0.25, [[B2.id,700]])
    board.bus_servo_set_position(0.25, [[C2.id,700]])
    board.bus_servo_set_position(0.25, [[D3.id,200]])
    board.bus_servo_set_position(0.25, [[E3.id,200]])
    board.bus_servo_set_position(0.25, [[F3.id,200]])
    board.bus_servo_set_position(0.25, [[A3.id,800]])
    board.bus_servo_set_position(0.25, [[B3.id,800]])
    board.bus_servo_set_position(0.25, [[C3.id,800]])

def crabReady():
    board.bus_servo_set_position(0.25, [[A1,325]])
    board.bus_servo_set_position(0.25, [[B1,500]])
    board.bus_servo_set_position(0.25, [[C1,685]])
    board.bus_servo_set_position(0.25, [[D1,325]])
    board.bus_servo_set_position(0.25, [[E1,500]])
    board.bus_servo_set_position(0.25, [[F1,685]])
    board.bus_servo_set_position(0.25, [[D2.id,300]])
    board.bus_servo_set_position(0.25, [[E2.id,300]])
    board.bus_servo_set_position(0.25, [[F2.id,300]])
    board.bus_servo_set_position(0.25, [[A2.id,700]])
    board.bus_servo_set_position(0.25, [[B2.id,700]])
    board.bus_servo_set_position(0.25, [[C2.id,700]])
    board.bus_servo_set_position(0.25, [[D3.id,200]])
    board.bus_servo_set_position(0.25, [[E3.id,200]])
    board.bus_servo_set_position(0.25, [[F3.id,200]])
    board.bus_servo_set_position(0.25, [[A3.id,800]])
    board.bus_servo_set_position(0.25, [[B3.id,800]])
    board.bus_servo_set_position(0.25, [[C3.id,800]])

def testPos():
    board.bus_servo_set_position(0.25, [[A1,500]])
    board.bus_servo_set_position(0.25, [[B1,500]])
    board.bus_servo_set_position(0.25, [[C1,500]])
    board.bus_servo_set_position(0.25, [[D1,500]])
    board.bus_servo_set_position(0.25, [[E1,500]])
    board.bus_servo_set_position(0.25, [[F1,500]])
    board.bus_servo_set_position(0.25, [[D2.id,500]])
    board.bus_servo_set_position(0.25, [[E2.id,500]])
    board.bus_servo_set_position(0.25, [[F2.id,500]])
    board.bus_servo_set_position(0.25, [[A2.id,500]])
    board.bus_servo_set_position(0.25, [[B2.id,500]])
    board.bus_servo_set_position(0.25, [[C2.id,500]])
    board.bus_servo_set_position(0.25, [[D3.id,500]])
    board.bus_servo_set_position(0.25, [[E3.id,500]])
    board.bus_servo_set_position(0.25, [[F3.id,500]])
    board.bus_servo_set_position(0.25, [[A3.id,500]])
    board.bus_servo_set_position(0.25, [[B3.id,500]])
    board.bus_servo_set_position(0.25, [[C3.id,500]])



# variables in cm
step_limit = 16 #changeable
stride = 12 #changeable
body_height = 13.1 #changeable # 7 good
step_height = 5 #changeable
body = 13.45
segment1 = 7.5
segment2 = 13.7

# Turning degrees
turn2degree = 0 # 70.123
turn3degree = -90 # -160.123
allTickPos = 323 # 235 # Use this to tune the 45 turn
up3pos = 50 # 0 will click
up3posINV = 1000 - up3pos

# law of cosines
def angle (a, b, c):
    return math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

def findangle(a, b, c):
    angA = angle(b,c,a)
    angB = angle(c,a,b)
    angC = angle(a,b,c)
    # assert angA + angB + angC == 180.0
    return [angA, angB, angC]

def footPos1(i_body, i_height, i_limit, i_stride, i_stepheight):
    # constraints: height, distance from midline body length, minimum step length, stride length, lift height
    # driven dimensions: jointangle1, jointangle2

    # calculate distance between anchor and foot location
    anchor = [i_body, i_height]
    foot = [i_limit + i_stride, 0]
    c = math.dist(anchor, foot)
    #print(anchor, foot, c)

    #calculate angles for isolated leg triangle
    [rawA, rawB, rawC] = findangle(segment1, segment2, c)
    #print(rawA,rawB,rawC)

    # joint 2 is relative
    j2 = -(180-rawC)

    # joint 1 is fixed
    theta1 = math.degrees(math.asin((foot[1]-anchor[1])/c))
    #print(theta1)
    j1 = theta1+rawB

    return [j1,j2]

def footPos2(i_body, i_height, i_limit, i_stride, i_stepheight):
    # constraints: height, distance from midline body length, minimum step length, stride length, lift height
    # driven dimensions: jointangle1, jointangle2

    # calculate distance between anchor and foot location
    anchor = [i_body, i_height]
    foot = [i_limit + i_stride/2, i_stepheight]
    c = math.dist(anchor, foot)
    #print(anchor, foot, c)

    #calculate angles for isolated leg triangle
    [rawA, rawB, rawC] = findangle(segment1, segment2, c)
    #print(rawA+rawB+rawC)

    # joint 2 is relative
    j2 = -(180-rawC)

    # joint 1 is fixed
    theta1 = math.degrees(math.asin((foot[1]-anchor[1])/c))
    #print(theta1)
    j1 = theta1+rawB

    return [j1,j2]

def footPos3(i_body, i_height, i_limit, i_stride, i_stepheight):
    # constraints: height, distance from midline body length, minimum step length, stride length, lift height
    # driven dimensions: jointangle1, jointangle2

    # calculate distance between anchor and foot location
    anchor = [i_body, i_height]
    foot = [i_limit, 0]
    c = math.dist(anchor, foot)
    #print(anchor, foot, c)

    #calculate angles for isolated leg triangle
    #print(segment1,segment2,c)
    [rawA, rawB, rawC] = findangle(segment1, segment2, c)
    #print(rawA,rawB,rawC)

    # joint 2 is relative
    j2 = -(180-rawC)

    # joint 1 is fixed
    theta1 = math.degrees(math.asin((foot[1]-anchor[1])/c))
    #print(theta1)
    j1 = theta1+rawB

    return [j1,j2]

def footPos4(i_body, i_height, i_limit, i_stride, i_stepheight):
    # constraints: height, distance from midline body length, minimum step length, stride length, lift height
    # driven dimensions: jointangle1, jointangle2

    # calculate distance between anchor and foot location
    anchor = [i_body, i_height]
    foot = [i_limit + i_stride/2, 0]
    c = math.dist(anchor, foot)
    #print(anchor, foot, c)

    #calculate angles for isolated leg triangle
    [rawA, rawB, rawC] = findangle(segment1, segment2, c)
    #print(rawA+rawB+rawC)

    # joint 2 is relative
    j2 = -(180-rawC)

    # joint 1 is fixed
    theta1 = math.degrees(math.asin((foot[1]-anchor[1])/c))
    #print(theta1)
    j1 = theta1+rawB

    
    return [j1,j2]

def genAngles(i_body, i_height, i_limit, i_stride, i_stepheight):
    af1 = footPos3(i_body, i_height, i_limit, i_stride, i_stepheight)
    af2 = footPos4(i_body, i_height, i_limit, i_stride, i_stepheight)
    af3 = footPos1(i_body, i_height, i_limit, i_stride, i_stepheight)
    af4 = footPos2(i_body, i_height, i_limit, i_stride, i_stepheight)

    bf1 = footPos1(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf2 = footPos2(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf3 = footPos3(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf4 = footPos4(i_body, i_height, i_limit, i_stride, i_stepheight)

    cf1 = af1
    cf2 = af2
    cf3 = af3
    cf4 = af4

    df1 = footPos3(i_body, i_height, i_limit, i_stride, i_stepheight)
    df2 = footPos2(i_body, i_height, i_limit, i_stride, i_stepheight)
    df3 = footPos1(i_body, i_height, i_limit, i_stride, i_stepheight)
    df4 = footPos4(i_body, i_height, i_limit, i_stride, i_stepheight)

    ef1 = footPos1(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef2 = footPos4(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef3 = footPos3(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef4 = footPos2(i_body, i_height, i_limit, i_stride, i_stepheight)

    ff1 = df1
    ff2 = df2
    ff3 = df3
    ff4 = df4

    frame1angs = [af1,bf1,cf1,df1,ef1,ff1]
    frame2angs = [af2,bf2,cf2,df2,ef2,ff2]
    frame3angs = [af3,bf3,cf3,df3,ef3,ff3]
    frame4angs = [af4,bf4,cf4,df4,ef4,ff4]

    return[frame1angs,frame2angs,frame3angs,frame4angs]

def strideA6(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.25 #0.15
    delay = 0.5 #0.25
    # Frame 1
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    time.sleep(delay)
    # Frame 2
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(B2,angs[1][1][0],stepTime)
    set_degree(B3,angs[1][1][1],stepTime)
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(D2,angs[1][3][0],stepTime)
    set_degree(D3,angs[1][3][1],stepTime)
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)
    set_degree(F2,angs[1][5][0],stepTime)
    set_degree(F3,angs[1][5][1],stepTime)
    time.sleep(delay)
    # Frame 3
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)

    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)

    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)

    time.sleep(delay)
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)
    time.sleep(delay)

    # Frame 4
    set_degree(A2,angs[3][0][0],stepTime)
    set_degree(A3,angs[3][0][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    set_degree(C2,angs[3][2][0],stepTime)
    set_degree(C3,angs[3][2][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    set_degree(E2,angs[3][4][0],stepTime)
    set_degree(E3,angs[3][4][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    time.sleep(delay)


def strideA(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.15 #0.15
    delay = 0.25 #0.25
    # Frame 1
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    # Frame 2
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(B2,angs[1][1][0],stepTime)
    set_degree(B3,angs[1][1][1],stepTime)
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(D2,angs[1][3][0],stepTime)
    set_degree(D3,angs[1][3][1],stepTime)
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)
    set_degree(F2,angs[1][5][0],stepTime)
    set_degree(F3,angs[1][5][1],stepTime)
    time.sleep(delay)
    # Frame 3
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)
    time.sleep(delay)
    # Frame 4
    set_degree(A2,angs[3][0][0],stepTime)
    set_degree(A3,angs[3][0][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    set_degree(C2,angs[3][2][0],stepTime)
    set_degree(C3,angs[3][2][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    set_degree(E2,angs[3][4][0],stepTime)
    set_degree(E3,angs[3][4][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    time.sleep(delay)

def strideD(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.15 #0.15 #0.08
    delay = 0.25  # 0.25 #0.2
    # Frame 1
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)

    # Frame 4
    set_degree(A2,angs[3][0][0],stepTime)
    set_degree(A3,angs[3][0][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    set_degree(C2,angs[3][2][0],stepTime)
    set_degree(C3,angs[3][2][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    set_degree(E2,angs[3][4][0],stepTime)
    set_degree(E3,angs[3][4][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    time.sleep(delay)
    # Frame 3
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)
    time.sleep(delay)
    # Frame 2
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(B2,angs[1][1][0],stepTime)
    set_degree(B3,angs[1][1][1],stepTime)
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(D2,angs[1][3][0],stepTime)
    set_degree(D3,angs[1][3][1],stepTime)
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)
    set_degree(F2,angs[1][5][0],stepTime)
    set_degree(F3,angs[1][5][1],stepTime)
    time.sleep(delay)

def strideAW(distance,adjust):

    
    # adjust is an absolute value in centimeters
    
    

    stepTime = 0.25
    delay = 0.5

    Dtarget = 275
    Ftarget = 635
    Btarget = 450

    if (adjust >= 5):
        adjust = 5

    else:
        Dtarget += int((5 - adjust)*10)
        Ftarget += int((5 - adjust)*10)
        Btarget += int((5 - adjust)*10)

    # Geometric analysis for extra stride distance
    offset = math.sqrt(distance ** 2 + adjust ** 2) # hypotenuse for triangle of stride and adjust (a^2 + b^2)^(1/2), for error purposes should be set to 0 in every other case
    print("offset:", offset)
    # angles should generate with distance + offset, there is a chance that it could be an illegal angle? in that event the code could break but if it's a 5-12-13 triangle then we should be okay
    angs = genAngles(body,body_height,step_limit, offset, step_height)

    # Frame 1
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    time.sleep(delay)
    
    # Frame 2
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(B2,angs[1][1][0],stepTime)
    set_degree(B3,angs[1][1][1],stepTime)
    board.bus_servo_set_position(0.25, [[B1,Btarget]])
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(D2,angs[1][3][0],stepTime)
    set_degree(D3,angs[1][3][1],stepTime)
    board.bus_servo_set_position(0.25, [[D1,Dtarget]])
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)
    set_degree(F2,angs[1][5][0],stepTime)
    set_degree(F3,angs[1][5][1],stepTime)
    board.bus_servo_set_position(0.25, [[F1,Ftarget]])
    time.sleep(delay)

    # Frame 3
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)
    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)
    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)
    time.sleep(delay)
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)    
    time.sleep(delay)

    # Frame 4    
    set_degree(A2,angs[3][0][0],stepTime)
    set_degree(A3,angs[3][0][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    board.bus_servo_set_position(0.25, [[B1,500]])
    set_degree(C2,angs[3][2][0],stepTime)
    set_degree(C3,angs[3][2][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    board.bus_servo_set_position(0.25, [[D1,325]])
    set_degree(E2,angs[3][4][0],stepTime)
    set_degree(E3,angs[3][4][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    board.bus_servo_set_position(0.25, [[F1,685]])    
    time.sleep(delay)

def strideAS(distance, adjust):



    stepTime = 0.25
    delay = 0.5

    Dtarget = 390
    Ftarget = 750
    Btarget = 565

    if (adjust >= 5):
        adjust = 5

    else:
        Dtarget -= int((5 - adjust)*13)
        Ftarget -= int((5 - adjust)*13)
        Btarget -= int((5 - adjust)*13)

    # Geometric analysis for extra stride distance
    offset = math.sqrt(distance ** 2 + adjust ** 2) # hypotenuse for triangle of stride and adjust (a^2 + b^2)^(1/2), for error purposes should be set to 0 in every other case
    print("offset:", offset)
    # angles should generate with distance + offset, there is a chance that it could be an illegal angle? in that event the code could break but if it's a 5-12-13 triangle then we should be okay
    angs = genAngles(body,body_height,step_limit, offset, step_height)

    # Frame 1
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    time.sleep(delay)
    
    # Frame 2
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(B2,angs[1][1][0],stepTime)
    set_degree(B3,angs[1][1][1],stepTime)
    board.bus_servo_set_position(0.25, [[B1,Btarget]])
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(D2,angs[1][3][0],stepTime)
    set_degree(D3,angs[1][3][1],stepTime)
    board.bus_servo_set_position(0.25, [[D1,Dtarget]])
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)
    set_degree(F2,angs[1][5][0],stepTime)
    set_degree(F3,angs[1][5][1],stepTime)
    board.bus_servo_set_position(0.25, [[F1,Ftarget]])
    time.sleep(delay)

    # Frame 3
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)
    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)
    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)
    time.sleep(delay)
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)    
    time.sleep(delay)

    # Frame 4    
    set_degree(A2,angs[3][0][0],stepTime)
    set_degree(A3,angs[3][0][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    board.bus_servo_set_position(0.25, [[B1,500]])
    set_degree(C2,angs[3][2][0],stepTime)
    set_degree(C3,angs[3][2][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    board.bus_servo_set_position(0.25, [[D1,325]])
    set_degree(E2,angs[3][4][0],stepTime)
    set_degree(E3,angs[3][4][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    board.bus_servo_set_position(0.25, [[F1,685]])    
    time.sleep(delay)


def turnPose():
    stepTime = 0.25
    # All 1 fan
    board.bus_servo_set_position(stepTime, [[A1,500]])
    board.bus_servo_set_position(stepTime, [[B1,500]])
    board.bus_servo_set_position(stepTime, [[C1,500]])
    board.bus_servo_set_position(stepTime, [[D1,500]])
    board.bus_servo_set_position(stepTime, [[E1,500]])
    board.bus_servo_set_position(stepTime, [[F1,500]])
    # All 2 up
    set_degree(A2, turn2degree, stepTime)
    set_degree(B2, turn2degree, stepTime)
    set_degree(C2, turn2degree, stepTime)
    set_degree(D2, turn2degree, stepTime)
    set_degree(E2, turn2degree, stepTime)
    set_degree(F2, turn2degree, stepTime)
    # All 3 down
    set_degree(A3, turn3degree, stepTime)
    set_degree(B3, turn3degree, stepTime)
    set_degree(C3, turn3degree, stepTime)
    set_degree(D3, turn3degree, stepTime)
    set_degree(E3, turn3degree, stepTime)
    set_degree(F3, turn3degree, stepTime)

def crabPose():
    angs = genAngles(13.45, 13.1, 16, 12, 0)
    stepTime = 0.25
    # Crab joint 1
    board.bus_servo_set_position(stepTime, [[A1,325]])
    board.bus_servo_set_position(stepTime, [[B1,500]])
    board.bus_servo_set_position(stepTime, [[C1,685]])
    board.bus_servo_set_position(stepTime, [[D1,325]])
    board.bus_servo_set_position(stepTime, [[E1,500]])
    board.bus_servo_set_position(stepTime, [[F1,685]])
    # Crab joints 2 & 3 using generated angles
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(A3,angs[1][0][1],stepTime)
    set_degree(C2,angs[1][2][0],stepTime)
    set_degree(C3,angs[1][2][1],stepTime)
    set_degree(B2,angs[3][1][0],stepTime)
    set_degree(B3,angs[3][1][1],stepTime)
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(D3,angs[3][3][1],stepTime)
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(F3,angs[3][5][1],stepTime)
    set_degree(E2,angs[1][4][0],stepTime)
    set_degree(E3,angs[1][4][1],stepTime)

def ACEPlace():
    stepTime = 0.25
    # Joint 2
    set_degree(A2, turn2degree, stepTime)
    set_degree(C2, turn2degree, stepTime)
    set_degree(E2, turn2degree, stepTime)
    # Joint 3
    set_degree(A3, turn3degree, stepTime)
    set_degree(C3, turn3degree, stepTime)
    set_degree(E3, turn3degree, stepTime)

def BDFPlace():
    stepTime = 0.25
    # Joint 2
    set_degree(B2, turn2degree, stepTime)
    set_degree(D2, turn2degree, stepTime)
    set_degree(F2, turn2degree, stepTime)
    # Joint 3
    set_degree(B3, turn3degree, stepTime)
    set_degree(D3, turn3degree, stepTime)
    set_degree(F3, turn3degree, stepTime)

def ACElift():
    stepTime = 0.25
    # Joint 2
    set_degree(A2, 90, stepTime)
    set_degree(C2, 90, stepTime)
    set_degree(E2, 90, stepTime)
    # Joint 3 (if needed)
    board.bus_servo_set_position(stepTime, [[E3.id,up3pos]])
    board.bus_servo_set_position(stepTime, [[A3.id,up3posINV]])
    board.bus_servo_set_position(stepTime, [[C3.id,up3posINV]])

def BDFlift():
    stepTime = 0.25
    # Joint 2
    set_degree(B2, 90, stepTime)
    set_degree(D2, 90, stepTime)
    set_degree(F2, 90, stepTime)
    # Joint 3 (if needed)
    board.bus_servo_set_position(stepTime, [[D3.id,up3pos]])
    board.bus_servo_set_position(stepTime, [[F3.id,up3pos]])
    board.bus_servo_set_position(stepTime, [[B3.id,up3posINV]])

def ACE1turn(direction):
    stepTime = 0.5
    if direction == 0:
        # Turn
        tickPos = allTickPos
        board.bus_servo_set_position(stepTime, [[A1,tickPos]])
        board.bus_servo_set_position(stepTime, [[C1,tickPos]])
        board.bus_servo_set_position(stepTime, [[E1,tickPos]])
    elif direction == 1:
        # Neutral
        board.bus_servo_set_position(stepTime, [[A1,500]])
        board.bus_servo_set_position(stepTime, [[C1,500]])
        board.bus_servo_set_position(stepTime, [[E1,500]])         
    else:
        # Counter turn
        tickPos = 1000 - allTickPos
        board.bus_servo_set_position(stepTime, [[A1,tickPos]])
        board.bus_servo_set_position(stepTime, [[C1,tickPos]])
        board.bus_servo_set_position(stepTime, [[E1,tickPos]])

def BDF1turn(direction):
    stepTime = 0.5
    if direction == 0:
        # Turn
        tickPos = allTickPos
        board.bus_servo_set_position(stepTime, [[B1,tickPos]])
        board.bus_servo_set_position(stepTime, [[D1,tickPos]])
        board.bus_servo_set_position(stepTime, [[F1,tickPos]])
    elif direction == 1:
        # Neutral
        board.bus_servo_set_position(stepTime, [[B1,500]])
        board.bus_servo_set_position(stepTime, [[D1,500]])
        board.bus_servo_set_position(stepTime, [[F1,500]])
    else:
        # Counter turn
        tickPos = 1000 - allTickPos
        board.bus_servo_set_position(stepTime, [[B1,tickPos]])
        board.bus_servo_set_position(stepTime, [[D1,tickPos]])
        board.bus_servo_set_position(stepTime, [[F1,tickPos]])

def Qsequence():
    # Begin sequencing
    BDFlift()
    time.sleep(0.4)
    BDF1turn(2)
    time.sleep(0.75)
    BDFPlace()
    time.sleep(0.4)
    ACElift()
    time.sleep(0.4)
    BDF1turn(0)
    time.sleep(0.75)
    ACEPlace()
    time.sleep(0.4)

def Esequence():
    # Begin sequencing
    BDFlift()
    time.sleep(0.4)
    BDF1turn(0)
    time.sleep(0.75)
    BDFPlace()
    time.sleep(0.4)
    ACElift()
    time.sleep(0.4)
    BDF1turn(2)
    time.sleep(0.75)
    ACEPlace()
    time.sleep(0.4)

def left90():
    delay = 0.4
    # Verify turnPose
    turnPose()
    time.sleep(delay)
    Qsequence()
    Qsequence()
    BDFlift()
    time.sleep(delay)
    BDF1turn(1)
    time.sleep(0.75)
    BDFPlace()

def right90():
    delay = 0.4
    # Verify turnPose
    turnPose()
    time.sleep(delay)
    Esequence()
    Esequence()
    BDFlift()
    time.sleep(delay)
    BDF1turn(1)
    time.sleep(0.75)
    BDFPlace()

def turn180():
    delay = 0.4
    # Verify turnPose
    turnPose()
    time.sleep(delay)
    Esequence()
    Esequence()
    Esequence()
    Esequence()
    BDFlift()
    time.sleep(delay)
    BDF1turn(1)
    time.sleep(0.75)
    BDFPlace()

def shiftCrabTurn():
    # Leg Order A,D,F,C ???? Maybe A, D, C, F??
    stepTime = 0.15
    delay = 0.25

    # Verify crabPose()
    crabPose()

    # A up
    set_degree(A2, 90, stepTime)
    time.sleep(delay)
    set_degree(A3, turn3degree, stepTime)
    board.bus_servo_set_position(stepTime, [[A1,500]])
    time.sleep(delay)

    # A down, D up
    set_degree(A2, turn2degree, stepTime)
    set_degree(D2, 90,stepTime)
    time.sleep(delay)
    set_degree(D3, turn3degree, stepTime)
    board.bus_servo_set_position(stepTime, [[D1,500]])
    time.sleep(delay)

    # D down, F up
    set_degree(D2, turn2degree, stepTime)
    set_degree(F2, 90,stepTime)
    time.sleep(delay)
    set_degree(F3, turn3degree, stepTime)
    board.bus_servo_set_position(stepTime, [[F1,500]])
    time.sleep(delay)

    # F down, C up
    set_degree(F2, turn2degree,stepTime)
    set_degree(C2, 90,stepTime)
    time.sleep(delay)
    set_degree(C3, turn3degree,stepTime)
    board.bus_servo_set_position(stepTime, [[C1,500]])
    time.sleep(delay)

    # C down
    set_degree(C2, turn2degree,stepTime)
    time.sleep(delay)

    # Verify turnPose()
    turnPose()

def shiftTurnCrab():
    # Leg Order A,D,F,C ???? Maybe A, D, C, F??

    angs = genAngles(body, 13.1, 16, 12, step_height)

    stepTime = 0.15
    delay = 0.25

    # Verify turnPose()
    turnPose()

    # A up
    set_degree(A2,20,stepTime)
    time.sleep(delay)
    set_degree(A3,angs[1][0][1],stepTime)
    board.bus_servo_set_position(stepTime, [[A1,325]])
    time.sleep(delay)

    # A down, D up
    set_degree(A2,angs[1][0][0],stepTime)
    set_degree(D2,20,stepTime)
    time.sleep(delay)
    set_degree(D3,angs[3][3][1],stepTime)
    board.bus_servo_set_position(stepTime, [[D1,325]])
    time.sleep(delay)

    # D down, F up
    set_degree(D2,angs[3][3][0],stepTime)
    set_degree(F2,20,stepTime)
    time.sleep(delay)
    set_degree(F3,angs[3][5][1],stepTime)
    board.bus_servo_set_position(stepTime, [[F1,685]])
    time.sleep(delay)

    # F down, C up
    set_degree(F2,angs[3][5][0],stepTime)
    set_degree(C2,20,stepTime)
    time.sleep(delay)
    set_degree(C3,angs[1][2][1],stepTime)
    board.bus_servo_set_position(stepTime, [[C1,685]])
    time.sleep(delay)

    # C down
    set_degree(C2,angs[1][2][0],stepTime)
    time.sleep(delay)

    # Verify crabPose()
    crabPose()



def crabWalk(s, target, allowance, distance):
    #read sensor
  #  print("reading")
    dist = s.getDistance()
    while dist == 5000:
        print("5000 reading :/")
        dist = s.getDistance()
    #choose behavior
    error = abs(dist - target)
    adjust = (abs(allowance - error))/10
    #if we are within allowance
    if (error < allowance):
        strideA6(distance)
    #we are too close

    elif (dist - target < 0):
      
        strideAS(distance, adjust)
    #dist - target > 0 - we are too far
    else:
        strideAW(distance, adjust)
#initialize and setup


#make the sonar
s = sonar.Sonar()

# Need to merge crabWalk() which is the Proportional control with walk() to implement wall following
# Consider gaps in walls causing weird sensor readings and unnecessary Proportional adjustment
def walk():
    crabReady()

    # for i in range(8):
    if not start:
        ("resetting and stopping")
        crabReady()
        
        time.sleep(1)
        print('breaking loop')
        return

    if s.getDistance() > 350:
        strideA6(stride)
    else:
        print("Reading", s.getDistance())
        crabWalk(s, 233, 10, stride)
    
    if not start:
        ("resetting and stopping")
        crabReady()
        
        time.sleep(1)
        print('breaking loop')
        return

    if s.getDistance() > 350:
        strideA6(stride)
    else:
        print("Reading", s.getDistance())
        crabWalk(s, 233, 10, stride)
    
    if not start:
        ("resetting and stopping")
        crabReady()
        
        time.sleep(1)
        print('breaking loop')
        return

    if s.getDistance() > 350:
        strideA6(stride)
    else:
        print("Reading", s.getDistance())
        crabWalk(s, 233, 10, stride-4)
    time.sleep(0.3)
    crabPose()    


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
    print("Going from", curr, "to", to)
    if curr - to == 0:
        print("Staying")
        return
    if curr - to == 1 or curr - to == -3:
        print("Left turn")
        shiftCrabTurn()
        left90()
        shiftTurnCrab()
        return
    if curr - to == -1 or curr - to == 3:
        print("Right turn")
        shiftCrabTurn()
        right90()
        shiftTurnCrab()
    else:
        print("Around")
        shiftCrabTurn()
        turn180()
        shiftTurnCrab()


## Main program

def main():
    your_map = map_301.CSME301Map()

    your_map.printObstacleMap()
    your_map.costmap_size_col
    x, y, d = map(int, input("Start coords x y d:   ").split(" "))
    goal_x, goal_y, goal_d = map(int, input("Goal coords x y d:   ").split(" "))

    
    
    instructions = breadth_first_search(your_map, (x, y), (goal_x, goal_y))
    print(instructions)
    start = instructions[0]
    current_direction = d

    for c in instructions[1:]:
        if not start:
            ("resetting and stopping")
            crabReady()
            print('breaking loop')
            return
        print("Current direction", current_direction)
        if start[0] > c[0]: # UP
            print("Going up", c, 1)
            
            go_direction(current_direction, 1)
            current_direction = 1
            walk()
        
        elif start[0] < c[0]: #DOWN
            print("Going down", c, 3)
            go_direction(current_direction, 3)
            current_direction = 3
            walk()
            
        elif start[1] > c[1]: # LEFT
            print("Going left", c, 4)
            go_direction(current_direction, 4)
            
            current_direction = 4
            walk()
        else:
            print("Going right", c, 2)
            go_direction(current_direction, 2)
            current_direction = 2
            walk()
            

        start = c

    go_direction(current_direction, goal_d)
        


if __name__ == "__main__":
    board.bus_servo_set_position(0.25, [[21, 1000-875]])
    main()
    
    crabPose()
    # board.bus_servo_set_position(0.25, [[21, 875]]) # Forward
    # board.bus_servo_set_position(0.25, [[21, 1000-875]]) # Backward
    # board.bus_servo_set_position(0.25, [[21, 500]]) # Right

