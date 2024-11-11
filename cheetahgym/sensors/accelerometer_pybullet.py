from cheetahgym.sensors.accelerometer import Accelerometer
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion

import pybullet as p
import numpy as np

class AccelerometerPB(Accelerometer):
	def __init__(	self, 	
					robot_id = None, 
					physicsClientId = 0,
					accel_noise = 0,
					rpy_noise = 0 	):
		self.robot_id = robot_id
		self.physicsClientId = physicsClientId
		self.accel_noise = accel_noise
		self.rpy_noise = rpy_noise
		self.vel_prev = np.zeros(3)
		super().__init__()

	def update(self, dt = 0.002):
		pose = p.getBasePositionAndOrientation(self.robot_id, physicsClientId=self.physicsClientId)
		vel = p.getBaseVelocity(self.robot_id, physicsClientId=self.physicsClientId)

		rpy = p.getEulerFromQuaternion(pose[1])
		rot = np.array(p.getMatrixFromQuaternion(pose[1])).reshape((3, 3))
		# quat = [pose[1][3], pose[1][0], pose[1][1], pose[1][2]]
		# rpy = get_rpy_from_quaternion(quat)
		# rot = get_rotation_matrix_from_rpy(rpy)
		# vel_body_frame = rot.dot(vel[0])

		# accel_true_body_frame = (vel_body_frame - self.vel_prev)/dt
		# rpy_true = get_rpy_from_quaternion(quat)

		# self.linear_accel = accel_true_body_frame + self.accel_noise * np.random.randn(3)
		# self.rpy = rpy_true + self.rpy_noise * np.random.randn(3)

		# self.vel_prev = vel_body_frame

		linear_vel_world = np.array(vel[0])

		accel_world = (linear_vel_world - self.vel_prev) / dt

		self.linear_accel = rot.dot(accel_world) + self.accel_noise * np.random.randn(3)
		self.rpy = rpy + self.rpy_noise * np.random.randn(3)
		self.vel_prev = linear_vel_world