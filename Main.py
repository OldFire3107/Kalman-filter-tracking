#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from kalman import KalmanFilter
from tracker import *

data = []
threshold = 1000

with open('radar_dump.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')

    for row in csv_reader:
        data.append([float(x) for x in row if x])

Lx = []
Ly = []
Lz = []

tracking = 0
tracks = []

for measurements in data[1390:1600]:
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
            if cost[i][index] < threshold:
                assignment[i] = index

    for i in range(len(assignment)):
        if assignment[i] == -1:
            tracks[i].predictNoDetect()
        else:
            x = assignment[i]
            print(tracks[i].prediction, i)
            tracks[i].predict(measurements[x*3 : x*3 + 3])
            tracks[i].trace.append(tracks[i].prediction)
            print(tracks[i].prediction, i)
count = 0

for ele in tracks[1].trace:
    print(count, list(ele))
    count += 1
    Lx.append(ele.item(0))
    Ly.append(ele.item(1))
    Lz.append(ele.item(2))

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
ax.scatter(Lx, Ly, Lz, c = 'r', marker = 'o',)

ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

plt.show()