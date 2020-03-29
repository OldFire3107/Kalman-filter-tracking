#!/usr/bin/env python3

import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from kalman import KalmanFilter
from tracker import Tracker

with open('radar_dump.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')

    Lx = []
    Ly = []
    Lz = []

    for row in csv_reader:
        if row[0]:
            Lx.append(float(row[0]))
            Ly.append(float(row[1]))
            Lz.append(float(row[2])) 

    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(Lx[100:700], Ly[100:700], Lz[100:700], c = 'r', marker = 'o',)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    plt.show()