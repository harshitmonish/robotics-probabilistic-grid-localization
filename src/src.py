#!/usr/bin/env python
import numpy as np
import sys
import matplotlib.pyplot as plt
import math
import os
from std_msgs.msg import String
import rospy
import cv2
import threading
from time import sleep
import subprocess
from queue import Queue

map_ = np.zeros((20, 20))
Right = [[0, 0], [0, 1], [0, 2]]
Left = [[0, 0], [0, -1], [0, -2]]
Down = [[0, 0], [-1, 0], [-2, 0]]
Up  = [[0, 0], [1, 0], [2, 0]]
initial_x = 10
initial_y = 10
map_[initial_x][initial_y] = 1
states = 8 # taking 45 degree discretized
#probs = [[[0 for i in range(states)] for i in range(20)] for i in range(20)]
prob_trans = [0.2, 0.6, 0.2] # transition probability for the possible actions i.e no move, 1unit move, 2 unit move
prob_obs = [0.1, 0.2, 0.4, 0.2, 0.1] 
obs_x = [[-2, 0], [-1, 0], [0, 0], [1, 0], [2, 0]]
obs_y = [[0, -2], [0, -1], [0, 0], [0, 1], [0, 2]]
count = 1
queue = Queue()
def show():
    global count
    #global cv
    while True:
        mp = queue.get()
        sleep(1)
        if mp:
            img = cv2.imread("./map_plot.png", cv2.IMREAD_COLOR)
            print("Displaying fig")
            count+=1
            window_name = "image_"+str(count)
            cv2.imshow(window_name, img)
            cv2.waitKey(2000)
            cv2.destroyWindow(window_name)
            #sleep(2)
        queue.task_done()

def plot(cmap_, first):
    #plt.imshow(cmap_, cmap='viridis', origin='lower')
    plt.imshow(cmap_)
    plt.xlabel('x')
    plt.ylabel('y')
    #plt.colorbar()
    if first == 1:
        plt.title("Initial Map")
    else:
        plt.title("Intermediate Map")
    print("Saving fig")
    plt.savefig("./map_plot.png")
    queue.put(1)

def observation_prob(direc, int_val, new_map):
    global map_
    print("Observation model")
    for i in range(20):
        for j in range(20):
            prev_val = map_[i][j]
            map_[i][j] = 0
            t1 = []
            t2 = []
            for k in range(5):
                new_i = i+direc[k][0]
                new_j = j+direc[k][1]
                t2.append([new_i, new_j])
                t1.append(prev_val*prob_obs[k])
            tot = 0
            for k in range(5):
                ni = t2[k][0]
                nj = t2[k][1]
                pb = 0.0

                #check if outside of map
                if ni > 19 or ni < 0 or nj > 19 or nj < 0:
                    #evaluate likewise for each value of k the possibility of wall
                    if k == 1:
                        pb = t1[1]+t1[2]+t1[3]+t1[4]
                        if pb != 0:
                            new_map[t2[k-1][0], t2[k-1][1]]+=pb
                    elif k == 2:
                        pb = t1[1]+t1[2]+t1[3]
                        if pb != 0:
                            new_map[t2[k-2][0], t2[k-2][1]]+=pb
                    elif k == 3:
                        pb = t1[1]+t1[2]
                        if pb != 0:
                            new_map[t2[k-3][0], t2[k-3][1]]+=pb
                    elif k == 4:
                        pb = t1[4]
                        if pb != 0:
                            new_map[t2[k-4][0], t2[k-4][1]]+=pb

                else:
                    #if inside map
                    pb = t1[k]
                    if pb != 0.0:
                        new_map[t2[k][0], t2[k][1]]+=pb
                tot+=pb
            #new_map = new_map / tot
    map_ = np.add(map_,new_map)
    plot(map_, 0)


def prediction_prob(direc, int_val, new_map):
    print("prediciton model")
    global map_
    for iter in range(int_val):
        for i in range(20):
            for j in range(20):
                prev_val = map_[i][j]
                map_[i][j] = 0
                t1 = []
                t2 = []
                for k in range(3):
                    new_i = i+direc[k][0]
                    new_j = j+direc[k][1]
                    t2.append([new_i, new_j])
                    t1.append(prev_val*prob_trans[k])
                tot = 0
                for k in range(3):
                    ni = t2[k][0]
                    nj = t2[k][1]

                    #check if outside of map
                    if ni > 19 or ni < 0 or nj > 19 or nj < 0:
                        #evaluate likewise for each value of k the possibility of wall
                        if k == 1:
                            pb = t1[1]+t1[2]
                            if pb != 0:
                                new_map[t2[k-1][0], t2[k-1][1]]+=pb
                        elif k == 2:
                            pb = t1[2]
                            if pb != 0:
                                new_map[t2[k-2][0], t2[k-2][1]]+=pb
                    else:
                        #if inside map
                        pb = t1[k]
                        if pb != 0.0:
                            new_map[t2[k][0], t2[k][1]]+=pb
                    tot+=pb
                #new_map = new_map / tot
        map_ = np.add(map_,new_map)
        plot(map_, 0)

def callback(msg):
    print("Callback: %s"%msg.data)
    data = msg.data.upper()
    direction = data[0]
    int_val = int(data[1])
    dirc = None
    if direction == 'R' or direction == 'L' or direction == 'U' or direction == 'D':
        pred = 1
    elif direction == 'X' or direction == 'Y':
        pred = 0
    else:
        print("Error!! Invalid command type given, kindly provide either R, L, U, D or X, Y with int values only")
        return
    new_map = np.zeros((20, 20))
    if pred:
        #prediction
        if direction == 'R':
            direc = Right
        elif direction == 'L':
            direc = Left
        elif direction == 'U':
            direc = Up
        elif direction == 'D':
            direc = Down
        prediction_prob(direc, int_val, new_map)
        

    else:
        #observation
        if direction == 'X':
            direc = obs_x
        else:
            direc = obs_y
        observation_prob(direc, int_val, new_map)

def main():
    rospy.init_node('lab6')
    print("Plotting Initial Map")
    plot(map_, 1)
    thread = threading.Thread(target=show)
    thread.start()
    print("Initializing Nodes ")
    sub1 = rospy.Subscriber('robot', String, callback, queue_size=10)
    pub1 = rospy.Publisher('robot', String, queue_size=10)
    r = rospy.Rate(2)
    rospy.spin()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
