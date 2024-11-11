import numpy as np

class cameraParameters:
	def __init__(self, width=224, height=224, 
					   x=0, y=0, z=0, 
					   roll=0, pitch=np.pi/6, yaw=0, 
					   fov=45.0, aspect=1.0, 
					   nearVal=0.1, farVal=3.1,
					   cam_pose_std=0, cam_rpy_std=0):
		# defaults
		self.width = width
		self.height = height
		self.pose = np.array([x, y, z])
		self.rpy = np.array([roll, pitch, yaw])
		self.fov=fov
		self.aspect=aspect
		self.nearVal=nearVal
		self.farVal=farVal

		# randomization params
		self.cam_pose_std = cam_pose_std
		self.cam_rpy_std = cam_rpy_std