#!/usr/bin/env python3

from cmath import inf
import copy
import matplotlib.pyplot as plt
from heapq import heappush, heappop
import numpy
from node import *
from fork_push import *



n = 3

motion = numpy.array([[1,0],[0,1],[0,-1],[-1,0]])




import numpy


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

def forkPush(poseCurr,poseTar,motion,occMap):
    if availableMotion(poseCurr,motion,occMap) and motionCheck(poseCurr,poseTar,motion):
        pose = poseCurr.copy()
        steps = 0
        while(not numpy.all(((poseTar-pose).dot(motion))==0)):
            occMap[pose[0]+motion[0]][pose[1]+motion[1]] += occMap[pose[0]][pose[1]]
            occMap[pose[0]][pose[1]] = 0
            pose += motion
            steps +=1
        return occMap,steps
    else:
        return None, None


def motionCheck(poseCurr,poseTar,motion):
    return (poseTar-poseCurr).dot(motion) >= 0
    
def availableMotion(poseCurr,motion,occMap):
    return not (occMap[int(poseCurr[0]-motion[0])][int(poseCurr[1]-motion[1])]) and \
            isSafe(poseCurr[0]-motion[0],poseCurr[1]-motion[1],len(occMap),len(occMap[0]))

def calculateDistance(poseCurr,poseTar):
    return abs(poseCurr[0]-poseTar[0]) + abs(poseCurr[1]-poseTar[1])


class node:
    def __init__(self, parent, occMap, robotPose, MaterialPose, cost, level, robotAction, materialTarget):

        self.parent = parent
        
        self.occMap = occMap

        self.robotPose = robotPose

        self.materialPose = MaterialPose

        self.robotAction = robotAction

        self.cost = cost

        self.materialTarget = materialTarget

        self.level = level


    def __lt__(self,nxt):
        return self.level < nxt.level

def newNode(occMap,TargetMap, materialPose, MaterialTarget,
                level, parent, action):

    newMap = copy.deepcopy(occMap)

    newMap, steps = forkPush(materialPose, MaterialTarget, action, newMap)
    if not steps:
        return
    else:
        materialPose = materialPose + steps*action
        robotPose =  materialPose - action
        new_node = node(parent, newMap, robotPose,materialPose,
                    int(calculateDistance(materialPose, MaterialTarget)), parent.level + steps, steps*action, MaterialTarget)

        return new_node


def isSafe(x, y, x_max,y_max):
    return x>=0 and x < x_max  and y >= 0 and y< y_max



def calculateCost(occMap,final):
    totDist = 0
    for i in range(len(occMap)):
        for j in range(len(occMap[0])):
            if (occMap[i][j]):
                totDist += int(abs(occMap[i][j]-final[i][j]))
    return int(totDist)


def robotPath(root):

    if root == None:
        return []

    return robotPath(root.parent) + [root.robotAction]


def solve(occMap, robotPose, targetMap, materialPose, materialTarget):

    pq = priorityQueue()

    cost = calculateDistance(materialPose, materialTarget)

    root = node(None, occMap, robotPose,materialPose, cost, 0, [],materialTarget)

    pq.push(root)

    while not pq.empty():
        minimum = pq.pop()
        print(minimum.cost)
        if minimum.cost == 0:
            return minimum

        for i in range(n+1):
            child = newNode(minimum.occMap,targetMap, minimum.materialPose, minimum.materialTarget ,minimum.level, minimum, motion[i])

            if child:
                pq.push(child)



def main():
    occMap=numpy.zeros([5,5])
    occMap[1][1] = 1
    robotPose = numpy.array([0,0])
    TargetMap = numpy.zeros([5,5])
    TargetMap[3][3] = 1
    MaterialPose = numpy.array([1,1])
    MaterialTarget = numpy.array([3,3])
    print(solve(occMap,robotPose, TargetMap, MaterialPose, MaterialTarget))



if __name__ == '__main__':
    main()