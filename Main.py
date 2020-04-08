#!/usr/bin/env python3

#TODO: make a way to check thershold

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from kalman import KalmanFilter
from tracker import Tracks
import time

data = []
threshold = 10
notTrackingLimit = 10
minPoints = 60

with open('radar_dump.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')

    for row in csv_reader:
        data.append([float(x) for x in row if x])

Lx = []
Ly = []
Lz = []

X = []
Y = []
Z = []

tracking = 0
tracks = []
notTracking = []

for measurements in data:
    count = 0
    # if measurements:
    #     Lx.append(measurements[0])
    #     Ly.append(measurements[1])
    #     Lz.append(measurements[2])

    measurements = np.array(measurements)
    if len(tracks) == 0:
        while len(tracks) < int(len(measurements) / 3):
            track = Tracks(measurements[count:count+3], tracking)
            tracking += 1
            tracks.append(track)
            count += 3
            notTracking.append(0)
    
    N = len(tracks)
    M = int(len(measurements))
    cost = []
    assignment = [-1]*N
    for i in range(N):
        diff = []
        for j in range(0, M, 3):
            diff.append(np.linalg.norm(tracks[i].prediction - (measurements[j: j+3]).reshape(-1,3)))
        cost.append(diff)
    
    for i in range(len(cost)):
        if cost[i]:
            index = np.argmin(cost[i])
            if tracks[i].CheckMeasurement(cost[i][index]):
                assignment[i] = index

    if len(tracks) < int(M /3):
        for i in range(int(M/3)):
            if i not in assignment:
                track = Tracks(measurements[3*i:3*i+3], tracking)
                tracking += 1
                tracks.append(track)
                count += 3
                notTracking.append(0)

    for i in range(len(assignment)):
        if assignment[i] == -1:
            tracks[i].predictNoDetect()
            tracks[i].trace.append(tracks[i].prediction)
            notTracking[i] += 1
            
        else:
            x = assignment[i]
            print(tracks[i].prediction, i)
            tracks[i].predict(measurements[x*3 : x*3 + 3])
            tracks[i].trace.append(tracks[i].prediction)
            print(tracks[i].prediction, i)

    popcount = 0
    while i < (len(assignment) - popcount):
        if notTracking[i] > notTrackingLimit:
            track = tracks.pop(i)
            notTracking.pop(i)
            assignment.pop(i)
            popcount += 1
            tracking -= 1
        
            if len(track.trace) > minPoints:
                tempX = []
                tempY = []
                tempZ = []
                for ele in track.trace:
                    tempX.append(ele.item(0))
                    tempY.append(ele.item(1))
                    tempZ.append(ele.item(2))
                X.append(tempX)
                Y.append(tempY)
                Z.append(tempZ)
            
        i += 1


# count1 = 0
# for ele in tracks[0].trace:
#     print(count1, list(ele))
#     count1 += 1
#     Lx.append(ele.item(0))
#     Ly.append(ele.item(1))
#     Lz.append(ele.item(2))

#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.scatter(Lx, Ly, Lz, c = 'r', marker = 'o')

#     ax.set_xlabel('X')
#     ax.set_ylabel('Y')
#     ax.set_zlabel('Z')

#     plt.show(block=False)
#     plt.pause(0.1)
#     time.sleep(0.1)
#     plt.close()

print("found %ld flight paths", len(X))

for i in range(len(X)):
    fig = plt.figure(i+1)
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(X[i], Y[i], Z[i], c = 'r', marker = 'o')

plt.show()