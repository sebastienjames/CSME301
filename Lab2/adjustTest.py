import sys
import time
import signal
import threading
import math
import os
import ros_robot_controller_sdk as rrc
import sonar

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
                print("position illegal?", pos, "for", self.name)
               # raise Exception
        else:
         #   self.max_pos > self.zero_pos:
            if pos > self.max_pos or pos < self.min_pos:
                print("position illegal?", pos, "for", self.name)
               # raise Exception

        print ("pos is ", pos, "degree was", degree)
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
step_limit = 15 #changeable
stride = 13 #changeable
body_height = 7 #changeable # 7 good
step_height = 2.5 #changeable
body = 13.45
segment1 = 7.5
segment2 = 13.7

# law of cosines
def angle (a, b, c):
    return math.degrees(math.acos((c**2 - b**2 - a**2)/(-2.0 * a * b)))

def findangle(a, b, c):
    angA = angle(b,c,a)
    angB = angle(c,a,b)
    angC = angle(a,b,c)
    # assert angA + angB + angC == 180.0
    return [angA, angB, angC]

def frame1step(i_body, i_height, i_limit, i_stride, i_stepheight):
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


def frame2step(i_body, i_height, i_limit, i_stride, i_stepheight):
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


def frame3step(i_body, i_height, i_limit, i_stride, i_stepheight):
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


def frame4step(i_body, i_height, i_limit, i_stride, i_stepheight):
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
    af1 = frame3step(i_body, i_height, i_limit, i_stride, i_stepheight)
    af2 = frame4step(i_body, i_height, i_limit, i_stride, i_stepheight)
    af3 = frame1step(i_body, i_height, i_limit, i_stride, i_stepheight)
    af4 = frame2step(i_body, i_height, i_limit, i_stride, i_stepheight)

    bf1 = frame1step(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf2 = frame2step(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf3 = frame3step(i_body, i_height, i_limit, i_stride, i_stepheight)
    bf4 = frame4step(i_body, i_height, i_limit, i_stride, i_stepheight)

    cf1 = af1
    cf2 = af2
    cf3 = af3
    cf4 = af4

    df1 = frame3step(i_body, i_height, i_limit, i_stride, i_stepheight)
    df2 = frame2step(i_body, i_height, i_limit, i_stride, i_stepheight)
    df3 = frame1step(i_body, i_height, i_limit, i_stride, i_stepheight)
    df4 = frame4step(i_body, i_height, i_limit, i_stride, i_stepheight)

    ef1 = frame1step(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef2 = frame4step(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef3 = frame3step(i_body, i_height, i_limit, i_stride, i_stepheight)
    ef4 = frame2step(i_body, i_height, i_limit, i_stride, i_stepheight)

    ff1 = df1
    ff2 = df2
    ff3 = df3
    ff4 = df4

    frame1angs = [af1,bf1,cf1,df1,ef1,ff1]
    frame2angs = [af2,bf2,cf2,df2,ef2,ff2]
    frame3angs = [af3,bf3,cf3,df3,ef3,ff3]
    frame4angs = [af4,bf4,cf4,df4,ef4,ff4]

    return[frame1angs,frame2angs,frame3angs,frame4angs]


def strideA(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.15
    delay = 0.25
    print('frame1')
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
    print('frame2')
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
    print('frame3')
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
    print('frame4')
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
    print('frame1')
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

    print('frame4')
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
    print('frame3')
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
    print('frame2')
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
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.15
    delay = 0.25

    Dtarget = 275
    Ftarget = 635
    Btarget = 450

    if (adjust >= 5):
        adjust = 5

    else:
        Dtarget += int((5 - adjust)*10)
        Ftarget += int((5 - adjust)*10)
        Btarget += int((5 - adjust)*10)

       
    

    print('frame1')
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
    
    print('frame2')
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

    print('frame3')
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
    
    print('frame4')
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
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = 0.15
    delay = 0.25

    Dtarget = 390
    Ftarget = 750
    Btarget = 565

    if (adjust >= 5):
        adjust = 5

    else:
        Dtarget -= int((5 - adjust)*13)
        Ftarget -= int((5 - adjust)*13)
        Btarget -= int((5 - adjust)*13)

       
    print('frame1')
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
    
    print('frame2')
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

    print('frame3')
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
    
    print('frame4')
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



# def set_degree(servo, degree, speed):
def degTest(servo):
    print("resetting")
  #  resetPos()
    time.sleep(3)
   # print("going through -90 to 90 degrees")
    deg = -90
    print("degree", deg)
    set_degree(servo,deg,1)
    time.sleep(3)
    deg = -45
    print("degree", deg)
    set_degree(servo,deg,1)
    time.sleep(3)
    deg = 0
    print("degree", deg)
    set_degree(servo,deg,1)
    time.sleep(3)
    deg = 45
    print("degree", deg)
    set_degree(servo,deg,1)

def degTest2(servos):
    print("resetting")
  #  resetPos()
    time.sleep(3)
   # print("going through -90 to 90 degrees")
    deg = -90
    print("degree", deg)
    for servo in servos:
        set_degree(servo,deg,1)
    time.sleep(3)
    deg = -45
    print("degree", deg)
    for servo in servos:
        set_degree(servo,deg,1)
    time.sleep(3)
    deg = 0
    print("degree", deg)
    for servo in servos:
        set_degree(servo,deg,1)
    time.sleep(3)
    deg = 45
    print("degree", deg)
    for servo in servos:
        set_degree(servo,deg,1)
    

def Joint3out():
    board.bus_servo_set_position(0.25, [[D3.id,350]])
    board.bus_servo_set_position(0.25, [[E3.id,350]])
    board.bus_servo_set_position(0.25, [[F3.id,350]])
    board.bus_servo_set_position(0.25, [[A3.id,650]])
    board.bus_servo_set_position(0.25, [[B3.id,650]])
    board.bus_servo_set_position(0.25, [[C3.id,650]])

def Joint3in():
    board.bus_servo_set_position(0.25, [[D3.id,200]])
    board.bus_servo_set_position(0.25, [[E3.id,200]])
    board.bus_servo_set_position(0.25, [[F3.id,200]])
    board.bus_servo_set_position(0.25, [[A3.id,800]])
    board.bus_servo_set_position(0.25, [[B3.id,800]])
    board.bus_servo_set_position(0.25, [[C3.id,800]])

def BDFDown():
    board.bus_servo_set_position(0.25, [[D2.id,500]])
    board.bus_servo_set_position(0.25, [[F2.id,500]])
    board.bus_servo_set_position(0.25, [[B2.id,500]])

def ACEDown():
    board.bus_servo_set_position(0.25, [[A2.id,500]])
    board.bus_servo_set_position(0.25, [[C2.id,500]])
    board.bus_servo_set_position(0.25, [[E2.id,500]])

def BDFUp():
    board.bus_servo_set_position(0.25, [[D2.id,100]])
    board.bus_servo_set_position(0.25, [[F2.id,100]])
    board.bus_servo_set_position(0.25, [[B2.id,900]])

def ACEUp():
    board.bus_servo_set_position(0.25, [[E2.id,100]])
    board.bus_servo_set_position(0.25, [[A2.id,900]])
    board.bus_servo_set_position(0.25, [[C2.id,900]])

def BDFNeutral():
    board.bus_servo_set_position(0.25, [[B1,500]])
    board.bus_servo_set_position(0.25, [[D1,500]])
    board.bus_servo_set_position(0.25, [[F1,500]])

def ACENeutral():
    board.bus_servo_set_position(0.25, [[A1,500]])
    board.bus_servo_set_position(0.25, [[C1,500]])
    board.bus_servo_set_position(0.25, [[E1,500]])


def BDFCturn():
    board.bus_servo_set_position(0.25, [[B1,672]])
    board.bus_servo_set_position(0.25, [[D1,672]])
    board.bus_servo_set_position(0.25, [[F1,672]])

def ACEturn():
    board.bus_servo_set_position(0.25, [[A1,328]])
    board.bus_servo_set_position(0.25, [[C1,328]])
    board.bus_servo_set_position(0.25, [[E1,328]])

def ACECturn():
    board.bus_servo_set_position(0.25, [[A1,672]])
    board.bus_servo_set_position(0.25, [[C1,672]])
    board.bus_servo_set_position(0.25, [[E1,672]])

def BDFturn():
    board.bus_servo_set_position(0.25, [[B1,328]])
    board.bus_servo_set_position(0.25, [[D1,328]])
    board.bus_servo_set_position(0.25, [[F1,328]])

def Qsequence():
    BDFUp()
    time.sleep(0.5)
    BDFCturn()
    time.sleep(0.5)
    BDFDown()
    time.sleep(0.5)
    ACEUp()
    time.sleep(0.5)
    BDFturn()
    time.sleep(0.5)
    ACEDown()
    time.sleep(0.5)

def left90():
    ACEDown()
    BDFDown()
    time.sleep(0.5)
    Joint3out()
    time.sleep(0.5)
    Qsequence()
    Qsequence()
    BDFUp()
    time.sleep(0.5)
    BDFNeutral()
    time.sleep(0.5)
    BDFDown()

def left45():
    ACEDown()
    BDFDown()
    time.sleep(0.5)
    Joint3out()
    time.sleep(0.5)
    Qsequence()
    BDFUp()
    time.sleep(0.5)
    BDFNeutral()
    time.sleep(0.5)
    BDFDown()

def right90():
    ACEDown()
    BDFDown()
    time.sleep(0.5)
    Joint3out()
    time.sleep(0.5)
    Esequence()
    Esequence()
    BDFUp()
    time.sleep(0.5)
    BDFNeutral()
    time.sleep(0.5)
    BDFDown()

def right45():
    ACEDown()
    BDFDown()
    time.sleep(0.5)
    Joint3out()
    time.sleep(0.5)
    Esequence()
    BDFUp()
    time.sleep(0.5)
    BDFNeutral()
    time.sleep(0.5)
    BDFDown()

def Esequence():
    BDFUp()
    time.sleep(0.5)
    BDFturn()
    time.sleep(0.5)
    BDFDown()
    time.sleep(0.5)
    ACEUp()
    time.sleep(0.5)
    BDFCturn()
    time.sleep(0.5)
    ACEDown()
    time.sleep(0.5)


def countWaves(sen):
    #i have to be honest i chose this number at near random
    precision = 50
    #record first measurement
    last = sen.getDistance()
    waves = 0
    #keeps track of if wave is up or down
    waveUp = False
    time.sleep(0.5)
    for i in range(9):
        print("Last:", last)
        curr = sen.getDistance()
        print("Curr:", curr)
        if abs(last - curr) > precision:
            if waveUp:
                 waves += 1
                 waveUp = False
            else: 
                waveUp = True
        last = curr
        print("wave count =", waves)
        time.sleep(0.5)
    print("waves is", waves)
    return waves

def waveControl(s):
    print("reading waves")
    waves = countWaves(s)
    print("doing stuff")
    if waves == 1:
        #turn right
        print("turning right")
        right90()
    if waves == 2:
        #turn left
        print("turning left")
        left90()
    if waves == 3:
        #turn around
        #doing it to the left. why not?
        print("turning around")
        left90()
        left90()


def crabWalk(s, target, allowance):
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
        strideA(10)
    #we are too close

    elif (dist - target < 0):
      
        strideAS(10, adjust)
    #dist - target > 0 - we are too far
    else:
        strideAW(10, adjust)
#initialize and setup

#first thing we do is get her in a neat pose
resetPos()

#make the sonar
s = sonar.Sonar()
s.setRGBMode(0)
s.setPixelColor(1, (0, 0, 255))
s.setPixelColor(0,(0,0,255))
s.show()
time.sleep(0.1)
s.startSymphony()

#we may need to be crab ready
crabReady()
#resetPos()

#behaviors loop
while True:
    crabWalk(s, 220, 45)

    #strideA(15)
    #time.sleep(2)

    #crab testing
   # print("test crab")
    #(NOTE: should crab ready be called before BSequenceTest? like for it to behave properly)
   # BSequenceTest()
    #print("crab tested")

   # degree control testing
   # print("test B3")
    #degTest(B3)
    #degTest2((A2, B2, C2, D2, E2, F2))
    #degTest2((A3, B3, C3, D3, E3, F3))
   # print("test A2")
   # degTest(A2)
   # print("test B2")
  #  degTest(B2)
   # print("test C2")
  #  degTest(C2)
   # print("test D2")
   # degTest(D2)
   # print("test E2")
  #  degTest(E2)
  #  print("test F2")
   # degTest(F2)

    #ultrasonic testing
   # waveControl(s)

    print("looping")
   
   #kill
    if not start:
        ("resetting and stopping")
        crabReady()
        time.sleep(1)
        print('breaking loop')
        break

