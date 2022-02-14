#!/usr/bin/env python3


import copy

import matplotlib.pyplot as plt

from heapq import heappush, heappop

import numpy

n = 3

row = [1,0,-1,0]
col = [0,-1,0,1]

class priorityQueue:

    def __init__(self):
        self.heap = []


    def push(self,k):
        heappush(self.heap, k)

    def pop(self):
        return heappop(self.heap)


    def empty(self):
        if not self.heap:
            return True
        else:
            return False

class node:
    def __init__(self, parent, occ_mat, robot_pose, cost, level, robot_action):

        self.parent = parent
        
        self.occ_mat = occ_mat

        self.robot_pose = robot_pose

        self.robot_action = robot_action

        self.cost = cost

        self.level = level


    def __lt__(self,nxt):
        return self.level+self.cost*0.5 < nxt.level+nxt.cost*0.5


def calculateCost(occ_mat,final) -> float:

    tot_dist = 0
    for i in range(len(occ_mat)):
        for j in range(len(occ_mat[0])):
            if (occ_mat[i][j]):
                tot_dist += abs(occ_mat[i][j]-final[i][j])
    return tot_dist

def newNode(occ_mat, robot_pose, new_robot_pose,
                level, parent, final, action) -> node:

    new_mat = copy.deepcopy(occ_mat)

    x1 = robot_pose[0]
    y1 = robot_pose[1]
    x2 = new_robot_pose[0]
    y2 = new_robot_pose[1]

    if (new_mat[x2][y2]):
        step_x = x2-x1
        step_y = y2-y1
        if (isSafe(x2+step_x, y2+step_y,len(parent.occ_mat),len(parent.occ_mat[0]))):
            new_mat[x2+step_x][y2+step_y] += new_mat[x2][y2]
            new_mat[x2][y2] = 0
        else:
            return

    
    cost = calculateCost(new_mat, final)

    new_node = node(parent, new_mat, new_robot_pose,
                    cost+level, level, action)

    return new_node


def printMatrix(mat):
    for i in range(len(mat)):
        for j in range(len(mat[0])):
            print("%d " % (mat[i][j]), end = " ")
        print()

def isSafe(x, y, x_max,y_max):
    return x>=0 and x < x_max  and y >= 0 and y< y_max

def robotPath(root):

    if root == None:
        return []

    return robotPath(root.parent) + [root.robot_action]


def solve(initial, robot_pose, final):

    pq = priorityQueue()

    cost = calculateCost(initial, final)

    root = node(None, initial, robot_pose, cost, 0, [])

    pq.push(root)

    while not pq.empty():
        minimum = pq.pop()

        if minimum.cost - minimum.level <= 0.1:

            return robotPath(minimum)

        for i in range(n+1):
            new_robot_pose = [minimum.robot_pose[0] + row[i], minimum.robot_pose[1] + col[i]]
            # print(new_robot_pose)
            if isSafe(new_robot_pose[0], new_robot_pose[1],len(minimum.occ_mat),len(minimum.occ_mat[0])):

                child = newNode(minimum.occ_mat,minimum.robot_pose,new_robot_pose,minimum.level+1,minimum,final, [row[i],col[i]])
                if child:
                    pq.push(child)


def main():
    initial = numpy.zeros((8,8))


    initial[3][4] = 1
    initial[3][3] = 1
    initial[3][2] = 1
    initial[4][2] = 1


    final = numpy.zeros((8,8))

    final[3][4] = 1
    final[2][3] = 2
    final[3][2] = 1

    robot_pose = [5,5]


    robot_path = solve(initial,robot_pose,final)
    print(robot_path)





if __name__ == "__main__":
    main()