import numpy as np

class LowLevelState:
    def __init__(self):
        self.body_pos = np.zeros(3)
        self.body_rpy = np.zeros(3)
        self.body_linear_vel = np.zeros(3)
        self.body_angular_vel = np.zeros(3)
        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)

        self.vec = np.zeros(36)

    def from_vec(self, vec):
        self.body_pos = vec[0:3]
        self.body_rpy = vec[3:6]
        self.joint_pos = vec[6:18]
        self.body_linear_vel = vec[18:21]
        self.body_angular_vel = vec[21:24]
        self.joint_vel = vec[24:36]

    def to_vec(self):
        self.vec[0:3] = self.body_pos
        self.vec[3:6] = self.body_rpy
        self.vec[6:18] = self.joint_pos
        self.vec[18:21] = self.body_linear_vel
        self.vec[21:24] = self.body_angular_vel
        self.vec[24:36] = self.joint_vel
        return self.vec

class LowLevelCmd:
    def __init__(self):
        self.p_targets = np.zeros(19)
        self.v_targets = np.zeros(18)
        self.p_gains = np.zeros(18)
        self.v_gains = np.zeros(18)
        self.ff_torque = np.zeros(18)

        self.vec = np.zeros(92)

    def from_vec(self, vec):
        self.p_targets = vec[0:19]
        self.v_targets = vec[19:37]
        self.p_gains = vec[37:55]
        self.v_gains = vec[55:73]
        self.ff_torque = vec[73:91]

    def to_vec(self):
        self.vec[0:19] = self.p_targets
        self.vec[19:37] = self.v_targets 
        self.vec[37:55] = self.p_gains
        self.vec[55:73] = self.v_gains
        self.vec[73:91] = self.ff_torque 
        return self.vec