import numpy as np

class BodyStateEstimator:
	def __init__(self, initial_pos=None, initial_rpy=None):
		if initial_pos is None:
			self.pos = np.array([0, 0, 0.3])
		else:
			self.pos = initial_pos
		if initial_rpy is None:
			self.rpy = np.zeros(3)
		else:
			self.rpy = initial_rpy
		self.vel = np.zeros(3)

	def update(self, accel, rpy, dt):
		raise NotImplementedError

	def get_state(self):
		return self.pos, self.vel, self.rpy