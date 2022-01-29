#!/usr/bin/env python3


import copy

import matplotlib.pyplot as plt

from heapq import heappush, heappop

from numpy import empty

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
    def __init__(self, parent, occ_mat, empty_tile_pos, cost, level):

        self.parent = parent
        
        self.occ_mat = occ_mat

        self.empty_tile_pos = empty_tile_pos

        self.cost = cost

        self.level = level


    def __lt__(self,nxt):
        return self.cost < nxt.cost


def calculateCost(occ_mat,final) -> float:

    tot_dist = 0
    for i in range(len(occ_mat)):
        for j in range(len(occ_mat[0])):
            if (occ_mat[i][j]):
                
                tot_dist += occ_mat[i][j]*((i-final[0])**2+(j-final[1])**2)**(1/2)
    return tot_dist


def newNode(occ_mat, empty_tile_pos, new_empty_tile_pos,
                level, parent, final) -> node:

    new_mat = copy.deepcopy(occ_mat)

    x1 = empty_tile_pos[0]
    y1 = empty_tile_pos[1]
    x2 = new_empty_tile_pos[0]
    y2 = new_empty_tile_pos[1]

    if (new_mat[x2][y2]):
        step_x = x2-x1
        step_y = y2-y1
        new_mat[x2+step_x][y2+step_y] += new_mat[x2][y2]
        new_mat[x2][y2] = 0

    
    cost = calculateCost(new_mat, final)

    new_node = node(parent, new_mat, new_empty_tile_pos,
                    cost+level, level)

    return new_node


def printMatrix(mat):
    for i in range(len(mat)):
        for j in range(len(mat[0])):
            print("%d " % (mat[i][j]), end = " ")
        print()

def isSafe(x, y, x_max,y_max,final):
    return x>=0 and x < x_max  and y >= 0 and y< y_max and (not (x == final[0] and y == final[1]))

def printPath(root):

    if root == None:
        return

    printPath(root.parent)
    printMatrix(root.occ_mat)
    print(root.empty_tile_pos)
    print()


def solve(initial, empty_tile_pos, final):

    pq = priorityQueue()

    cost = calculateCost(initial, final)

    root = node(None, initial, empty_tile_pos, cost, 0)

    pq.push(root)

    while not pq.empty():
        minimum = pq.pop()

        if int(minimum.cost)-minimum.level == 0:

            printPath(minimum)
            return

        for i in range(n+1):
            print(row[i],col[i])
            new_tile_pos = [minimum.empty_tile_pos[0] + row[i], minimum.empty_tile_pos[1] + col[i]]
            print(new_tile_pos)
            if isSafe(new_tile_pos[0], new_tile_pos[1],len(minimum.occ_mat),len(minimum.occ_mat[0]),final):

                child = newNode(minimum.occ_mat,minimum.empty_tile_pos,new_tile_pos,minimum.level+1,minimum,final)

                pq.push(child)


def main():
    initial = [[0,0,0,0,0,0],[0,0,0,0,0,0],[0,0,1,0,0,0],[0,0,1,1,0,0],[0,0,0,0,0,0],[0,0,0,0,0,0]]

    sx = -5.0  # [m]
    sy = -5.0  # [m]
    gx = 50.0  # [m]
    gy = 50.0  # [m]
    grid_size = 2.0  # [m]
    robot_radius = 1.0  # [m]

    # set obstacle positions
    ox, oy = [], []
    for i in range(-10, 60):
        ox.append(i)
        oy.append(-10.0)
    for i in range(-10, 60):
        ox.append(60.0)
        oy.append(i)
    for i in range(-10, 61):
        ox.append(i)
        oy.append(60.0)
    for i in range(-10, 61):
        ox.append(-10.0)
        oy.append(i)
    for i in range(-10, 40):
        ox.append(20.0)
        oy.append(i)
    for i in range(0, 40):
        ox.append(40.0)
        oy.append(60.0 - i)




    final = [3,3]

    empty_tile_pos = [0,0]

    plt.plot(empty_tile_pos[0], empty_tile_pos[1], "og")
    plt.plot([2,3], [2,2], "xb")
    plt.grid(True)
    plt.axis("equal")
    plt.show()

    solve(initial,empty_tile_pos,final)





if __name__ == "__main__":
    main()