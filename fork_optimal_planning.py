#!/usr/bin/env python3

from cmath import inf
import copy
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import numpy


class centerMass:
    def __init__(self):
        dx = 0
        dy = 0
 

class forkPlannings:
    def __init__(self,occGrid):
        self.occGrid = occGrid
        self.cM = centerMass()
        self.setCM()

    def setCM(self):
        for i in range(len(self.occGrid)):
            totalMass = 0
            for j in range(len(self.occGrid[0])):
                    self.cM.dx += i*self.occGrid[i][j]
                    self.cM.dy += j*self.occGrid[i][j]
                    totalMass += self.occGrid[i][j]
            self.cM.dx /= totalMass
            self.cM.dy /= totalMass


class node:
    def __init__(self, parent, occ_mat, robot_pose, cost, level, robot_action):

        self.parent = parent
        
        self.occ_mat = occ_mat

        self.robot_pose = robot_pose

        self.robot_action = robot_action

        self.cost = cost

        self.level = level


    def __lt__(self,nxt):
        return self.level < nxt.level


def minimalDistanceFromRobot(occ_mat, robot_pose): 
    minDistance = inf
    for i in range(len(occ_mat)):
        for j in range(len(occ_mat[0])):
            if occ_mat[i][j] >= 1:
                distance = numpy.sqrt((i-robot_pose[0])**2 + (j-robot_pose[1])**2)
            if distance < minDistance:
                minDistance = distance
    return minDistance

def main():
    pass


if __name__ == '__main__':
    main()