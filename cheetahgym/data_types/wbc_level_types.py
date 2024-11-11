import numpy as np

class WBCLevelState:
    def __init__(self):
        self.body_pos = np.zeros(3)
        self.body_rpy = np.zeros(3)
        self.body_linear_vel = np.zeros(3)
        self.body_angular_vel = np.zeros(3)
        self.body_linear_accel = np.zeros(3)
        self.joint_pos = np.zeros(3)
        self.joint_vel = np.zeros(3)

    def from_vec(self, vec):
        self.body_pos = vec[0:3]
        self.body_rpy = vec[3:6]
        self.joint_pos = vec[6:18]
        self.body_linear_vel = vec[18:21]
        self.body_angular_vel = vec[21:24]
        self.joint_vel = vec[24:36]

    def to_vec(self):
        vec = np.zeros(36)
        vec[0:3] = self.body_pos
        vec[3:6] = self.body_rpy
        vec[6:18] = self.joint_pos
        vec[18:21] = self.body_linear_vel
        vec[21:24] = self.body_angular_vel
        vec[24:36] = self.joint_vel
        return vec


class WBCLevelCmd:
    def __init__(self):
        # trajectory generator
        self.pBody_des = np.zeros(3)
        self.vBody_des = np.zeros(3)
        self.aBody_des = np.zeros(3)
        self.pBody_RPY_des = np.zeros(3)
        self.vBody_Ori_des = np.zeros(3)
        self.pFoot_des = np.zeros(12)
        self.vFoot_des = np.zeros(12)
        self.aFoot_des = np.zeros(12)
        self.contact_state = np.zeros(4)
        # mpc's work
        self.Fr_des = np.zeros(12)

    def from_vec(self, vec):
        self.pBody_des = vec[0:3]
        self.vBody_des = vec[3:6]
        self.aBody_des = vec[6:9]
        self.pBody_RPY_des = vec[9:12]
        self.vBody_Ori_des = vec[12:15]
        self.pFoot_des = vec[15:27]
        self.vFoot_des = vec[27:39]
        self.aFoot_des = vec[39:51]
        self.Fr_des= vec[51:63]
        # mpc's work
        self.contact_state = vec[63:67].astype(int)

    def to_vec(self):
        vec = np.zeros(28)
        vec[0:3] = self.pBody_des
        vec[3:6] = self.vBody_des
        vec[6:9] = self.aBody_des
        vec[9:12] = self.pBody_RPY_des
        vec[12:15] = self.vBody_Ori_des
        vec[15:27] = self.pFoot_des
        vec[27:39] = self.vFoot_des
        vec[39:51] = self.aFoot_des
        vec[51:63] = self.Fr_des
        vec[63:67] = self.contact_state
        return vec