import numpy as np

class KalmanFilter(object):

	def __init__(self, dt = 1, stateVariance = 0.1, measurementVariance = 0.01):

		super(KalmanFilter, self).__init__()
		self.stateVariance = stateVariance
		self.measurementVariance = measurementVariance
		self.dt = dt
		self.initModel()

	def initModel(self):
		self.A = np.matrix([
			[1., 0., 0., self.dt, 0., 0.],
			[0., 1., 0., 0., self.dt, 0.],
			[0., 0., 1., 0., 0., self.dt],
			[0., 0., 0., 1., 0., 0.],
			[0., 0., 0., 0., 1., 0.],
			[0., 0., 0., 0., 0., 1.]
		])

		self.H = np.matrix([
			[1., 0., 0., 0., 0., 0.],
			[0., 1., 0., 0., 0., 0.],
			[0., 0., 1., 0., 0., 0.]
		])

		self.P = np.matrix(self.stateVariance*np.identity(self.A.shape[0]))
		self.R = np.matrix(self.measurementVariance*np.identity(self.H.shape[0]))

		self.erroCov = self.P
		self.state = np.matrix([
			[0.],
			[0.],
			[0.],
			[0.],
			[0.],
			[0.]
		])

	def predict(self):
		self.predictedState = self.A * self.state
		self.predictedErrorCov = self.A * self.erroCov * self.A.T

		temp = np.asarray(self.predictedState)
		return temp[0], temp[1], temp[2]

	def correct(self, currentMeasurement):
		S = self.H * self.predictedErrorCov * self.H.T + self.R
		self.KalmanGain = self.predictedErrorCov * self.H.T * np.linalg.pinv(S)
		self.state = self.predictedState + self.KalmanGain * (currentMeasurement - (self.H * self.predictedState))
		self.erroCov = (np.identity(self.P.shape[0]) -  self.KalmanGain * self.H) * self.predictedErrorCov

	def update(self):
		self.state = self.predictedState
		self.erroCov = self.predictedErrorCov

	def measurementProbability(self, cost): # For now it returns only whther it can be counted or not.
		R = 1000 * np.matrix([	
			self.erroCov.item(0),
			self.erroCov.item(7),
			self.erroCov.item(14)
		])
		Rcost = np.linalg.norm(R)

		if cost < Rcost:
			return True
		
		return False
