import numpy as np

class ActuatorModel:
	'''
	Based on https://dspace.mit.edu/handle/1721.1/118671
	'''
	def __init__(self, use_joint_friction=False):

		# mini cheetah parameters
		self.gear_ratio = np.array([6, 6, 9.33, 6, 6, 9.33,
									6, 6, 9.33, 6, 6, 9.33])
		self.kt_motor = 0.05
		self.R_motor = 0.173
		self.V_battery = 24
		self.joint_damping = 0.01
		self.dry_friction = 0.2
		self.tau_max = 3.0
		self._tau_ff = np.zeros(12)
		self._kp = np.ones(12) * 10
		self._kd = np.ones(12) * 0.1

		self.use_joint_friction = use_joint_friction

	def apply_torque_limit(self, tau, qd):
		linear_decay_start, linear_decay_end = 60, 200 # for positive work only

		tau_max = np.maximum(np.minimum(3, 3 - 3 * (np.abs(qd * self.gear_ratio) - linear_decay_start) / (linear_decay_end - linear_decay_start)), 0)

		tau_max[np.multiply(tau, qd) < 0] = 3 # for negative work, full torque achieved

		tau = np.clip(tau, -tau_max, tau_max)

		return tau


	def update_cmd(self, q_des, qd_des, tau_ff, kp, kd):
		self._q_des = q_des
		self._qd_des = qd_des
		self._tau_ff = tau_ff
		self._kp = kp
		self._kd = kd

	def compute_low_level_cmd(self, q, qd):
		return self._kp * (self._q_des - q) + self._kd * (self._qd_des - qd) + self._tau_ff

	def get_torque(self, tau_des, qd):
		tau_des_motor = tau_des / self.gear_ratio # motor torque
		i_des = tau_des_motor / (self.kt_motor * 1.5) # i = tau / KT
		bemf = qd * self.gear_ratio * self.kt_motor * 2. # back emf
		v_des = i_des * self.R_motor + bemf # v = I*R + emf
		v_actual = np.clip(v_des, -self.V_battery, self.V_battery) # limit to battery voltage
		tau_act_motor = 1.5 * self.kt_motor * (v_actual - bemf) / self.R_motor
		
		#tau_act = self.gear_ratio * np.clip(tau_act_motor, -self.tau_max, self.tau_max)
		tau_act = self.gear_ratio * self.apply_torque_limit(tau_act_motor, qd)

		if self.use_joint_friction:
			tau_act = tau_act - self.joint_damping * qd - self.dry_friction * np.sign(qd)

		return tau_act

