import numpy as np

def build_mpc_table_from_params(offsets, durations, cycleLength):
    mpc_table = np.zeros((4 * cycleLength, 1))
    for i in range(len(offsets)):
        start = int(offsets[i])
        end = int(offsets[i] + durations[i])
        if end < cycleLength:
            mpc_table[start*4+i:end*4+i:4, 0] = 1
        else:
            mpc_table[start*4+i::4, 0] = 1
            #if self.cfg.use_gait_cycling and self.cfg.use_gait_wrapping_obs:
            mpc_table[i:(end % cycleLength)*4+i:4, 0] = 1
    return mpc_table



class MPCLevelState:
    def __init__(self):
        self.body_pos = np.zeros(3)
        self.body_rpy = np.zeros(3)
        self.body_linear_vel = np.zeros(3)
        self.body_angular_vel = np.zeros(3)
        self.body_linear_accel = np.zeros(3)
        self.joint_pos = np.zeros(3)
        self.joint_vel = np.zeros(3)

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

class MPCLevelCmd:
    def __init__(self):
        self.vel_cmd = np.zeros(3)
        self.vel_rpy_cmd = np.zeros(3)
        self.fp_rel_cmd = np.zeros(8)
        self.fh_rel_cmd = np.zeros(4)
        self.footswing_height = 0.
        self.offsets_smoothed = np.zeros(4).astype(int)
        self.durations_smoothed = np.zeros(4).astype(int)
        self.mpc_table_update = np.zeros((40, 1))
        self.vel_table_update = np.zeros((30, 1))
        self.vel_rpy_table_update = np.zeros((30, 1))
        self.iterations_table_update = np.zeros((10, 1))
        self.planningHorizon = 10
        self.adaptationHorizon = 10
        self.adaptationSteps = 10
        self.iterationsBetweenMPC = 0

        self.vec = np.zeros(28)

    def set_offsets(self, offsets):
        self.offsets_smoothed = offsets
        self.mpc_table_update = build_mpc_table_from_params(self.offsets_smoothed, self.durations_smoothed, cycleLength=10)

    def set_durations(self, durations):
        self.durations_smoothed = durations
        self.mpc_table_update = build_mpc_table_from_params(self.offsets_smoothed, self.durations_smoothed, cycleLength=10)

    def set_vel(self, vel):
        self.vel_cmd = vel
        self.vel_table_update = np.array([self.vel_cmd] * self.adaptationSteps).reshape(-1, 1)

    def set_vel_rpy(self, vel_rpy):
        self.vel_rpy_cmd = vel_rpy
        self.vel_rpy_table_update = np.array([self.vel_rpy_cmd] * self.adaptationSteps).reshape(-1, 1)

    def set_iterations(self, iterations):
        self.iterations = iterations
        self.iterations_table_update = np.array([self.iterationsBetweenMPC] * self.adaptationSteps).reshape(-1, 1)



    def from_vec(self, vec):
        self.vel_cmd = vec[0:3]
        self.vel_rpy_cmd = vec[3:6]
        self.fp_rel_cmd = vec[6:14]
        self.fh_rel_cmd = vec[14:18]
        self.footswing_height = vec[18]
        self.offsets_smoothed = vec[19:23].astype(int)
        self.durations_smoothed = vec[23:27].astype(int)
        self.iterationsBetweenMPC = int(vec[27])
        self.mpc_table_update = build_mpc_table_from_params(self.offsets_smoothed, self.durations_smoothed, cycleLength=10)
        self.vel_table_update = np.array([self.vel_cmd] * self.adaptationSteps).reshape(-1, 1)
        self.vel_rpy_table_update = np.array([self.vel_rpy_cmd] * self.adaptationSteps).reshape(-1, 1)
        self.iterations_table_update = np.ones(self.adaptationSteps) * self.iterationsBetweenMPC

    def to_vec(self):
        self.vec[0:3] = self.vel_cmd
        self.vec[3:6] = self.vel_rpy_cmd
        self.vec[6:14] = self.fp_rel_cmd
        self.vec[14:18] = self.fh_rel_cmd
        self.vec[18] = self.footswing_height
        self.vec[19:23] = self.offsets_smoothed
        self.vec[23:27] = self.durations_smoothed
        self.vec[27] = self.iterationsBetweenMPC
        return self.vec