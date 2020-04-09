# -*- coding: utf-8 -*-
"""
Created on Sat Apr  4 20:45:43 2020

@author: User
"""

import math
import serial
import time
import cv2
import numpy as np

RB_LN = 80
DEBUG = False

class robot:
    is_gripped = False
    # stores serial port
    def __init__(self, port):
        self.cap = cv2.VideoCapture(0)
        
        self.old_joints = [90, 90, 90, 90]
        
        if DEBUG: 
            print("DEBUG ON! port set to: " + port)
        else:
            self.ser = serial.Serial(port=port)
            # zero out
            time.sleep(1.5)
            self.ser.write(b'ZZZZ')
            time.sleep(1)
            
    def close(self):
        self.cap.release()
        self.ser.close()
    
    def getgripper(self):
        if (self.is_gripped): return 40
        else: return 90
    
    # send joints to arduino
    def update(self, joints):
        assert len(joints) == 4
        
        sendbytes = bytearray()
        sendbytes.append(int(joints[0]))
        sendbytes.append(int(joints[1]))
        sendbytes.append(int(joints[2]))
        sendbytes.append(int(joints[3]))
        
        if DEBUG: print(sendbytes)
        else: self.ser.write(sendbytes)
        
        time.sleep(0.01)

def find2d(xy_pos):
    # y calibration offset 
    xy_pos[1] = xy_pos[1] * 2 / 3
    # static X offset
    xy_pos[0] += 25
    # hacky offset to make robot go straigt up/down
    #TODO: do this properly
    if (xy_pos[0] < 0):
        xy_pos[0] -= -1.263 + (0.932 * xy_pos[1]) + (-0.018 * (xy_pos[1] * xy_pos[1]))
    else:
        xy_pos[0] -= 0.3 * ((0.932 * xy_pos[1]) + (-0.018 * (xy_pos[1] * xy_pos[1])))
    # origin = servo centerline
    # find hypotenuse of triangle X,Y,n
    #hyp = math.sqrt((xy_pos[0] * xy_pos[0]) + (xy_pos[1] * xy_pos[1]))
    hyp = xy_pos[0]
    
    # since both links of the robot are the same
    # length, both angles adjecent the hyp will be equal
    a1_a2 = math.degrees(math.acos(((hyp * hyp)) / (2 * hyp * RB_LN)))
    
    a3 = 180 - (2 * a1_a2)
    
    # we now know the relative angles both joints
    # now we just find what angle hyp is at, relative 
    # to the robot, and add that to our angles
    ang_offset = math.degrees(math.asin(xy_pos[1] / hyp))
    
    a1_a2 += ang_offset
    a3 += ang_offset
    
    return [a1_a2, a3]

def find3d(xyz, j4):
    # make internal copy of xyz so as to not fuck it up
    int_xyz = xyz.copy()
    # z position prescale
    int_xyz[2] = int_xyz[2] * 1.6
    
    hyp = math.sqrt((int_xyz[1] * int_xyz[1]) + (-int_xyz[0] * -int_xyz[0]))
    # find our rotation based on the x/y position
    j1 = math.degrees(math.acos(int_xyz[1] / hyp))
        
    # offset y by amount needed to compensate for x
    int_xyz[1] += int_xyz[1] - (int_xyz[1] * math.cos(math.radians(j1)))
    print(xyz)
    
    # robot 0 = 90deg
    if (int_xyz[0] < 0):
        j1 = 90 - j1
    else:
        j1 = 90 + j1
    
    # find j2, j3
    j2, j3 = find2d([int_xyz[1], int_xyz[2]])
    
    return [j1, j2, j3, j4]
    
def adjpos(joints):
    # calibrate against robot's servos
    newjoints = []
    
    # J1 isn't quite centered
    newjoints.append(joints[0] + 11)
    # J2 is inverted
    newjoints.append(180 - joints[1])
    # subtract J2/J3 interaction
    newjoints.append((90 - (180 - joints[1])) + joints[2])
    newjoints.append(joints[3])
    
    return newjoints

def vision_find(cam):
    _, frame = cam.read()
    # doulbe snapshot camera to prevent leftovers 
    time.sleep(0.01)
    _, frame = cam.read()
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 100, 200)
    
    dot = cv2.imread('img/dot.jpg', 0)
    w, h = dot.shape[::-1]
    
    #cv2.imshow('Source', edges)
    #input()
    
    # search for dots
    res = cv2.matchTemplate(edges, dot, cv2.TM_CCORR_NORMED)
    threshold = 0.5
    loc = np.where(res >= threshold)
    
    matches = []
    
    #https://stackoverflow.com/questions/21829469/
    mask = np.zeros(edges.shape[:2], np.uint8)
    
    # draw rects over dots and eliminate duplicates
    for pt in zip(*loc[::-1]):
        cv2.rectangle(edges, pt, (pt[0] + w, pt[1] + h), (255,255,255), 2)
        
        if mask[int(pt[1] + h/2), int(pt[0] + w/2)] != 255:
            mask[pt[1]:pt[1]+h, pt[0]:pt[0]+w] = 255
            matches.append(pt)
            
    die_pos = []
    x = 0
    y = 0
    
    # find dice centerpoint by averaging the dots
    if (len(matches)):
        for pt in matches:
            x += pt[0]
            y += pt[1]
        
        die_pos.append(x / len(matches))
        die_pos.append(y / len(matches))
        print("die pos = " + str(die_pos))
        
    #cv2.imshow('Detected', edges)
    
    # return the number on die face and its position
    return [len(matches), die_pos]

def vision_adjoffset(visresult, zoffset):
    # convert from vision frame to robot frame
    #TODO: make an easy way to touch up vision calibration
    
    # from looking down at plane from above and behind robot:
    # X is zero at far right and ~599 at far left
    # Y is zero futhest from robot and 441 closest to robot
    # center ~=[280, 220]
    # X: camera 0 = robot -25, camera 591 = robot 34
    # Y: camera 0 = robot 141, camera 441 = robot 105
    # note: robot can't reach beyond 135 without a math error!
    adj_x = mapnum(visresult[1][0], 0, 591, -25, 34)
    adj_y = mapnum(visresult[1][1], 0, 441, 141, 105)
    
    # add hand offset
    return [adj_x, adj_y - 9, zoffset]
    
def move(rob, pos, dwell=1.0, gripper=40):
    #run ik solver
    joints = find3d(pos, gripper)
    rob.update(adjpos(joints))
    
    # waits for servo angle delta * seconds/degree
    time.sleep((max([abs(rob.old_joints[0] - joints[0]), \
                     abs(rob.old_joints[1] - joints[1])]) * 0.02) * dwell)
    #time.sleep(1)
    # store old joints for time calc
    rob.old_joints = joints
    
def mapnum(x, in_min, in_max, out_min, out_max):
    # analog to arduino map() function
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

if __name__ == "__main__":
    rb = robot(input("port: "))
    oldjoints = []
    
    while (True):
        try: 
            # MAIN ROBOT PROGRAM
            # get vision data
            vres = vision_find(rb.cap)
            print("vision data: " + str(vres))
            
            # found die
            if (vres[0] > 0):
                vpos = vision_adjoffset(vres, 0)
                
                # -x offset to scoop up the die
                vpos[0] -= 8
                move(rb, vpos)
                
                # add -z offset - slow travel
                for i in range(0, 12):
                    vpos[2] -= 2
                    move(rb, vpos)
                
                # scoop up die close gripper
                for i in range(0, 10):
                    vpos[0] += 1
                    move(rb, vpos)
                
                move(rb, vpos, gripper=120)
                time.sleep(0.5)
                
                # subtract z offset
                vpos[2] += 30
                #move(rb, vpos, gripper=120)
                
                # move to roll position
                move(rb, [10, 107, 10], gripper=120)
                time.sleep(0.5)
                move(rb, [10, 107, 10])
                time.sleep(1)
            
        except ValueError as e:
            print(e)
            # move to home
            move(rb, [0, 105, 40])
