import numpy

def forkPush(poseCurr,poseTar,motion,occMap):
    if availableMotion(poseCurr,motion,occMap) and motionCheck(poseCurr,poseTar,motion):
        Pose = poseCurr[:]
        steps = 0
        while(not numpy.all(((poseTar-Pose).dot(motion))==0)):
            occMap[poseCurr[0]+motion[0]][poseCurr[1]+motion[1]] += occMap[Pose[0]][Pose[1]]
            occMap[Pose[0]][Pose[1]] = 0
            Pose += motion
            steps +=1
        return occMap, steps
    else:
        return None


def motionCheck(poseCurr,poseTar,motion):
    return (poseTar-poseCurr).dot(motion) >= 0
    
def availableMotion(poseCurr,motion,occMap):
    return not (occMap[int(poseCurr[0]-motion[0])][int(poseCurr[1]-motion[1])])

def calculateDistance(poseCurr,poseTar):
    return abs(poseCurr[0]-poseTar[0]) + abs(poseCurr[1]-poseTar[1])






def main():
    
    occMap = numpy.zeros((5,5))
    occMap[0][0] = 1
    occMap[1][3] = 1
    occMap[2][2] = 1
    occMap[3][3] = 1
    poseCurr = numpy.array([1,3])
    poseTar = numpy.array([2,1])
    motion = numpy.array([0,-1])
    print(forkPush(poseCurr,poseTar,motion,occMap))

if __name__ == '__main__':
    main()