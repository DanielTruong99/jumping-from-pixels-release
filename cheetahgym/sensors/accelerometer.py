import numpy as np

class Accelerometer:
	def __init__(self):
		self.linear_accel = np.zeros(3)
		self.rpy = np.zeros(3)

	def update(self):
		raise NotImplementedError

	def get_measurement(self):
		return self.linear_accel, self.rpy