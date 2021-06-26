
#import matplotlib.pyplot as plt
import numpy as np
#import random
import copy
'''create the field'''

#field = []
#targets = np.array([[0,0]]*12)
#obstacles = np.array([[0,0]]*4)
#while len(field) < 16:
#    ran = [random.randint(1,5),random.randint(1,5)]
#    if ran not in field and ran != [5,1] and ran != [4,1] and ran != [3,1]:
#        field.append(ran)
#for i in range(12):
#    targets[i] = field[i]
#for i in range(4):
#    obstacles[i] = field[15-i]
#print("12 targets are：")
#print(targets)
#print('\r')
#print("4 obstacles are：")
#print(obstacles)

def plan(targets,obstacles):
    '''route learning fun'''

    def Lroute(tar, obs, tend):
        s = []
        t = 0
        s.append(tar[0])
        weight = 99
        while (t < (len(tar) - 1)):
            l = len(s) - 1
            Location = copy.copy(s[l])
            '''judge the weight'''
            mov = np.array([0.0] * 4)
            Destination = tar[t + 1]
            RelativeP = Destination - Location
            '''tend weight'''
            if tend == 1:
                mov[0] += 1
                mov[2] += -1
            elif tend == 2:
                mov[0] += -1
                mov[2] += 1
            elif tend == 0:
                mov[0] += (Location[0] - 3)
                mov[2] -= (Location[0] - 3)
            '''target weight'''
            if RelativeP[0] > 0:
                mov[2] += weight
                mov[0] -= weight
            elif RelativeP[0] < 0:
                mov[0] += weight
                mov[2] -= weight
            if RelativeP[1] > 0:
                mov[1] += weight
                mov[3] -= weight
            elif RelativeP[1] < 0:
                mov[3] += weight
                mov[1] -= weight
            '''border weight'''
            if Location[0] == 1:
                mov[0] -= 10000
            if Location[0] == 5:
                mov[2] -= 10000
            if Location[1] == 1:
                mov[3] -= 10000
            if Location[1] == 5:
                mov[1] -= 10000
            '''obstacle weight'''
            if (obs == (Location + [1, 0])).all(1).any():
                mov[2] -= 10000
            if (obs == (Location + [-1, 0])).all(1).any():
                mov[0] -= 10000
            if (obs == (Location + [0, 1])).all(1).any():
                mov[1] -= 10000
            if (obs == (Location + [0, -1])).all(1).any():
                mov[3] -= 10000
            '''dont loop'''
            for i in range(l):
                if (s[l - 1 - i] == (Location + [1, 0])).all():
                    mov[2] -= 50
                if (s[l - 1 - i] == (Location + [-1, 0])).all():
                    mov[0] -= 50
                if (s[l - 1 - i] == (Location + [0, 1])).all():
                    mov[1] -= 50
                if (s[l - 1 - i] == (Location + [0, -1])).all():
                    mov[3] -= 50
            '''choose movement'''
            if max(mov) < -800:
                s.append(Location)
            movement = np.argmax(mov)
            if movement == 0:
                s.append(Location + [-1, 0])
            if movement == 1:
                s.append(Location + [0, 1])
            if movement == 2:
                s.append(Location + [1, 0])
            if movement == 3:
                s.append(Location + [0, -1])
            '''arrive at a target?'''
            if (Destination == s[l + 1]).all():
                t += 1

        return (s)

    initialL = np.array([5, 1])
    presentL = initialL
    Torder = np.array([[0, 0]] * 12)
    targetsL = np.array([[0, 0]] * 12)
    for i in range(12):
        targetsL[i] = copy.copy(targets[i])
    for t in range(12):
        relativeL = targetsL - presentL - [0.1, -0.1]
        relativeD = []
        for x in range(12):
            relativeD.append(np.linalg.norm(relativeL[x]))
        target = relativeD.index(min(relativeD))
        Torder[t] = targetsL[target]
        presentL = copy.copy(targetsL[target])
        targetsL[target] = [10, 10]
    print(Torder)
    Rorder = np.array([[0, 0]] * 14)
    Rorder[0] = initialL
    Rorder[13] = initialL
    for i in range(12):
        Rorder[i + 1] = Torder[i]
    Route0 = Lroute(Rorder, obstacles, 0)
    print(Route0)
    Torder1 = np.array([[0, 0]] * 14)
    Torder1[13] = initialL
    indexT = [0]
    for i in range(12):
        for j in range(len(Route0)):
            if (Route0[j] == Torder[i]).all():
                indexT.append(j)
                break
    indexT.append(len(Route0))
    indexT = sorted(indexT)
    print(indexT)
    for i in range(13):
        Torder1[i] = Route0[indexT[i]]
    print(Torder1)
    RO = np.array([[0, 0]] * 2)
    Route11 = []
    Route1 = []
    Route1.append(initialL)
    for i in range(13):
        RO[0] = Torder1[i]
        RO[1] = Torder1[i + 1]
        Route11 = Lroute(RO, obstacles, 0)
        for j in range(len(Route11) - 1):
            Route1.append(Route11[j + 1])
    print(Route1)
    RR = np.array([[0, 0]] * len(Route1))
    for i in range(len(RR)):
        RR[i] = Route1[i]
    return(RR)

#RR1 = plan(targets,obstacles)
#print(RR1)
#plt.figure(1)
#y_values=list(range(6))
#x_values=list(range(6))
#plt.plot(obstacles.T[0],obstacles.T[1],'y*')
#plt.plot(targets.T[0],targets.T[1],'bo')
#plt.plot(RR1.T[0],RR1.T[1],'b-')
#plt.show()
