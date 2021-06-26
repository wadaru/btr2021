#import matplotlib.pyplot as plt
import numpy as np
#import random
import copy

def plan(targets,obstacles):
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
                mov[1] += weight * 1.1
                mov[3] -= weight * 1.1
            elif RelativeP[1] < 0:
                mov[3] += weight * 1.1
                mov[1] -= weight * 1.1
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

    def ana(targets, obstacles):
        '''targets allocation'''
        core = [np.mean(targets.T[0]), np.mean(targets.T[1])]
        targetsR1 = np.array([[-1]] * 4)
        targetsR2 = np.array([[-1]] * 4)
        targetsR3 = np.array([[-1]] * 4)
        targetsM1 = np.array([[0, 0]] * 4)
        targetsM2 = np.array([[0, 0]] * 4)
        targetsM3 = np.array([[0, 0]] * 4)
        step = np.array([[500] * 12] * 12)

        TMov = np.array([[0, 0]] * 2)
        for i in range(12):
            for j in range(12):
                if i != j:
                    TMov[0] = targets[i]
                    TMov[1] = targets[j]
                    l = Lroute(TMov, obstacles, 0)
                    step[i][j] = len(l) - 1
        stepO = copy.copy(step)
        k1 = []
        for i in range(12):
            k1.append((targets[i][0] - core[0]) - (targets[i][1] - core[1]))
        targetsR1[0] = k1.index(min(k1))
        for i in range(12):
            step[k1.index(min(k1))][i] = 500

        def thk(ss, targetsR, ss1, Tar):
            k = 1
            while k < 4:
                stepd = np.array([[0]] * 12)
                stepl = np.array([[600]] * 12)
                stepw = np.array([[0.0]] * 12)
                for i in range(12):
                    for j in range(k):
                        stepd[i] += ss1[i][targetsR[j]]
                        stepl[i] = min([stepl[i], ss1[i][targetsR[j]]])
                    if ss[i][0] > 100:
                        stepw[i] = 1000
                    else:
                        stepw[i] = stepl[i] * 3 + ((stepd[i] - 10 * Tar[i][1]) / 100)
                kk = np.argmin(stepw)
                targetsR[k] = kk
                for i in range(12):
                    ss[kk][i] = 500
                k += 1
            return ()

        thk(step, targetsR1, stepO, targets)
        print(targetsR1)
        k2 = []
        for i in range(12):
            if i in targetsR1:
                k2.append(-1)
            else:
                k2.append(targets[i][0] + targets[i][1])

        for i in range(4):
            targetsR2[i] = k2.index(max(k2))
            k2[k2.index(max(k2))] = 0

        print(targetsR2)
        p = 0
        for i in range(12):
            if (i not in targetsR1) and (i not in targetsR2):
                targetsR3[p] = i
                p += 1

        for i in range(4):
            targetsM1[i] = targets[targetsR1[i]]
            targetsM2[i] = targets[targetsR2[i]]
            targetsM3[i] = targets[targetsR3[i]]
        Tar1 = np.array([[0, 0]] * 6)
        Tar1[0] = [3, 1]
        Tar1[5] = [3, 1]
        Tar2 = np.array([[0, 0]] * 6)
        Tar2[0] = [4, 1]
        Tar2[5] = [4, 1]
        Tar3 = np.array([[0, 0]] * 6)
        Tar3[0] = [5, 1]
        Tar3[5] = [5, 1]
        for i in range(4):
            TD = []
            for x in range(4):
                TD.append(np.linalg.norm(targetsM1[x] - [5, 1]))
            tar = TD.index(min(TD))
            Tar1[i + 1] = targetsM1[tar]
            targetsM1[tar] = [1, 10]
        for i in range(4):
            TD = []
            for x in range(4):
                TD.append(np.linalg.norm(targetsM2[x] - [1, 1]))
            tar = TD.index(min(TD))
            Tar2[i + 1] = targetsM2[tar]
            targetsM2[tar] = [10, 10]
        for i in range(4):
            TD = []
            for x in range(4):
                TD.append(np.linalg.norm(targetsM3[x] - [1, 2]))
            tar = TD.index(min(TD))
            Tar3[i + 1] = targetsM3[tar]
            targetsM3[tar] = [10, 10]
        return (Tar1, Tar2, Tar3)

    targetsM1, targetsM2, targetsM3 = ana(targets, obstacles)
    '''MOV M1'''
    RO1 = np.array([[0, 0]] * 2)
    RouteM1 = []
    Route1 = []
    Route1.append([3, 1])
    for i in range(5):
        RO1[0] = targetsM1[i]
        RO1[1] = targetsM1[i + 1]
        RouteM1 = Lroute(RO1, obstacles, 1)
        for j in range(len(RouteM1) - 1):
            Route1.append(RouteM1[j + 1])
    RR1 = np.array([[0, 0]] * len(Route1))
    for i in range(len(RR1)):
        RR1[i] = Route1[i]
    print(RR1)
    '''MOV M2'''
    RO2 = np.array([[0, 0]] * 2)
    RouteM2 = []
    Route2 = []
    Route2.append([4, 1])
    for i in range(5):
        RO2[0] = targetsM2[i]
        RO2[1] = targetsM2[i + 1]
        RouteM2 = Lroute(RO2, obstacles, 0)
        for j in range(len(RouteM2) - 1):
            Route2.append(RouteM2[j + 1])
    RR2 = np.array([[0, 0]] * len(Route2))
    for i in range(len(RR2)):
        RR2[i] = Route2[i]
    print(RR2)
    '''MOV M3'''
    RO3 = np.array([[0, 0]] * 2)
    RouteM3 = []
    Route3 = []
    Route3.append([5, 1])
    for i in range(5):
        RO3[0] = targetsM3[i]
        RO3[1] = targetsM3[i + 1]
        RouteM3 = Lroute(RO3, obstacles, 2)
        for j in range(len(RouteM3) - 1):
            Route3.append(RouteM3[j + 1])
    RR3 = np.array([[0, 0]] * len(Route3))
    for i in range(len(RR3)):
        RR3[i] = Route3[i]
    print(RR3)
    return (RR1,RR2,RR3)

'''create the field'''
#field = []
#insert = np.array([[5,1],[4,1],[3,1]])
#targets = np.array([[0,0]]*12)
#obstacles = np.array([[0,0]]*4)
#while len(field) < 16:
#    ran = [random.randint(1,5),random.randint(1,5)]
#    if ran not in field and ran != [5,1] and ran!=[4,1] and ran!=[3,1]:
#        field.append(ran)
#for i in range(12):
#    targets[i] = field[i]
#for i in range(4):
#    obstacles[i] = field[15-i]
#print ("12 targets are：")
#print (targets)
#print('\r')
#print ("4 obstacles are：")
#print (obstacles)

'''for i in range(12):
        step[k2.index(max(k2))][i] = 500
    thk(step,targetsR2,stepO,targets)
    '''


#RR1,RR2,RR3,targetsM1,targetsM2,targetsM3=plan(targets,obstacles)


'''
plt.figure(1)
plt.subplot(221)
y_values=list(range(6))
x_values=list(range(6))
plt.plot(obstacles.T[0],obstacles.T[1],'k*')
plt.plot(targetsM1.T[0],targetsM1.T[1],'bo')
plt.plot(targetsM2.T[0],targetsM2.T[1],'ro')
plt.plot(targetsM3.T[0],targetsM3.T[1],'go')
plt.plot(RR1.T[0],RR1.T[1],'c-')
plt.subplot(222)
y_values=list(range(6))
x_values=list(range(6))
plt.plot(obstacles.T[0],obstacles.T[1],'k*')
plt.plot(targetsM1.T[0],targetsM1.T[1],'bo')
plt.plot(targetsM2.T[0],targetsM2.T[1],'ro')
plt.plot(targetsM3.T[0],targetsM3.T[1],'go')
plt.plot(RR2.T[0],RR2.T[1],'y-')
plt.subplot(223)
y_values=list(range(6))
x_values=list(range(6))
plt.plot(obstacles.T[0],obstacles.T[1],'k*')
plt.plot(targetsM1.T[0],targetsM1.T[1],'bo')
plt.plot(targetsM2.T[0],targetsM2.T[1],'ro')
plt.plot(targetsM3.T[0],targetsM3.T[1],'go')
plt.plot(RR3.T[0],RR3.T[1],'m-')
plt.show()
'''
