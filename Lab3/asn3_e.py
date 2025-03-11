import sys
import time
import signal
import threading
import math
import os
import ros_robot_controller_sdk as rrc
import sympy as sp

import numpy as np
import pandas as pd
import seaborn as sns


data = pd.read_csv("/Users/sebastienjames/Documents/CS301/Lab3/data.csv")
data.dropna(inplace=True)

data = data[data["Step limit (15-(30-stride)cm)"] < 20]
data = data[data["Stride (1 - 30-steplimit) cm"] < 14]
# data = data[data["Stride Count (1 - )"] == 5]
# data = data[data["Step limit (15-(30-stride)cm)"] == 17]

data["dist_per_step"] = data["Distance (cm)"] / data["Stride Count (1 - )"]
# print(data)

sns.scatterplot(data=data, x="Distance (cm)", y="Battery (volts)", hue="Step limit (15-(30-stride)cm)")
# print(data)
def get_distance(goal, volt):
    newdata = data[["Distance (cm)", "Battery (volts)"]][:]
    sum_values = newdata.sum(axis=0)
    newdata = newdata / sum_values

    newdata["Distance (cm)"] -= goal / sum_values["Distance (cm)"]
    newdata["Distance (cm)"] *= newdata["Distance (cm)"]
    newdata["Battery (volts)"] -= volt / sum_values["Battery (volts)"]
    newdata["Battery (volts)"] *= newdata["Battery (volts)"]

    newdata["sum"] = 2*newdata["Distance (cm)"] + newdata["Battery (volts)"]
    newdata["sum"] = np.sqrt(newdata["sum"])

    return newdata

goal = 32
volt = 11.5

data["weight"] = 1 / get_distance(goal, volt)["sum"]
data["weight"]= data["weight"] / data["weight"].sum()

data = data.sort_values("weight", ascending=False)

# print(data.head(5))
n = 3

data["Step limit (15-(30-stride)cm)"] *= data["weight"]
data["Stride (1 - 30-steplimit) cm"] *= data["weight"]
data["Stride Count (1 - )"] *= data["weight"]

data = data.head(n)
print(data)

weight = data["weight"].sum()




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
      pos = servo.group.deg_to_pos(degree)
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

print("Step Limit:", )
print("Stride:", )
print("Stride Count:", )


# variables in cm
step_limit = data["Step limit (15-(30-stride)cm)"].sum() / weight #changeable MIN 15
stride = data["Stride (1 - 30-steplimit) cm"].sum() / weight #changeable ????? MAX 30-step_limit
a = data["Stride Count (1 - )"].sum() / weight
stride_count = int(a)
stride_remainder = a - stride_count

assert (step_limit + stride) <= 30

body = 13.45
segment1 = 7.5
segment2 = 13.7
body_height = 12 # Fixed 12
step_height = 3 # Fixed 3
globalStepTime = 0.25 # Fixed
globalDelay = 0.4 # Fixed

# POSES

def crabPose():
    angs = genAngles(body, body_height, step_limit, stride, 0)
    stepTime = globalStepTime
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

# ACTIONS

def strideA(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = globalStepTime #0.15
    delay = globalDelay #0.25
    # Frame 1
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    # Frame 1.5
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
    # Frame 3.5
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

def strideD(distance):
    angs = genAngles(body,body_height,step_limit, distance, step_height)
    stepTime = globalStepTime #0.15
    delay = globalDelay #0.25
    # Frame 1
    set_degree(A2,angs[0][0][0],stepTime)
    set_degree(A3,angs[0][0][1],stepTime)
    set_degree(C2,angs[0][2][0],stepTime)
    set_degree(C3,angs[0][2][1],stepTime)
    set_degree(E2,angs[0][4][0],stepTime)
    set_degree(E3,angs[0][4][1],stepTime)
    time.sleep(delay)
    # Frame 1.5
    set_degree(B2,angs[0][1][0],stepTime)
    set_degree(B3,angs[0][1][1],stepTime)
    set_degree(D2,angs[0][3][0],stepTime)
    set_degree(D3,angs[0][3][1],stepTime)
    set_degree(F2,angs[0][5][0],stepTime)
    set_degree(F3,angs[0][5][1],stepTime)
    time.sleep(delay)
    # Frame 2
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
    set_degree(B2,angs[2][1][0],stepTime)
    set_degree(B3,angs[2][1][1],stepTime)
    set_degree(D2,angs[2][3][0],stepTime)
    set_degree(D3,angs[2][3][1],stepTime)
    set_degree(F2,angs[2][5][0],stepTime)
    set_degree(F3,angs[2][5][1],stepTime)
    time.sleep(delay)
    # Frame 3.5
    set_degree(A2,angs[2][0][0],stepTime)
    set_degree(A3,angs[2][0][1],stepTime)
    set_degree(C2,angs[2][2][0],stepTime)
    set_degree(C3,angs[2][2][1],stepTime)
    set_degree(E2,angs[2][4][0],stepTime)
    set_degree(E3,angs[2][4][1],stepTime)
    time.sleep(delay)
    # Frame 4
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


## Main program

def main():
    crabPose()
    print('starting', step_limit, stride, stride_count+stride_remainder)
    time.sleep(2)
    for i in range(stride_count):
        strideA(stride)
    time.sleep(0.5)
    if stride_remainder > 0:
        strideA(stride * stride_remainder)
    crabPose()

if __name__ == "__main__":

    crabPose()
    
    main()
#kill
    if not start:
        ("resetting and stopping")
        crabPose()
        
        time.sleep(1)
        print('breaking loop')
        

