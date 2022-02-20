
import copy



class node:
    def __init__(self, parent, occMap, robotPose, cost, level, robotAction, targetCell):

        self.parent = parent
        
        self.occMap = occMap

        self.robotPose = robotPose

        self.robotAction = robotAction

        self.cost = cost

        self.target = targetCell

        self.level = level


    def __lt__(self,nxt):
        return self.cost < nxt.cost

def newNode(occMap, robot_pose, new_robot_pose,
                level, parent, final, action) -> node:

    new_mat = copy.deepcopy(occMap)

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


def isSafe(x, y, x_max,y_max):
    return x>=0 and x < x_max  and y >= 0 and y< y_max
    
