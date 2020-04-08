import numpy as np 
from kalman import KalmanFilter
from scipy.optimize import linear_sum_assignment
from collections import deque

class Tracks(object):

    def __init__(self, detection, trackID):

        super(Tracks, self).__init__()
        self.KF = KalmanFilter()
        self.KF.predict()
        self.KF.correct(np.matrix(detection).reshape(3, 1))
        self.trace = deque()
        self.prediction = detection.reshape(1,3)
        self.trackID = trackID
        self.skipped = 0

    def predict(self, detection):
        self.prediction = np.array(self.KF.predict()).reshape(1,3)
        self.KF.correct(np.matrix(detection).reshape(3,1))

    def predictNoDetect(self):
        self.prediction = np.array(self.KF.predict()).reshape(1,3)
        self.KF.update()
    
    def CheckMeasurement(self, cost):
        return self.KF.measurementProbability(cost)
