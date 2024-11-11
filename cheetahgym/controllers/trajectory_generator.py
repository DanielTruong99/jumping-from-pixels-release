import numpy as np
from tabulate import tabulate
import copy

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion
from cheetahgym.controllers.foot_swing_trajectory import FootSwingTrajectory
from cheetahgym.data_types.wbc_level_types import WBCLevelCmd

import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
    pass



class TrajectoryGenerator:
    def __init__(self, planning_horizon=10, iterationsBetweenMPC=17, dt=0.002):
        self.planning_horizon = planning_horizon
        self.iterationsBetweenMPC = iterationsBetweenMPC
        self.dt = dt

        self.iteration_prev_update = 0
        self.current_iteration = 0
        self.rpy_int = [0, 0]
        self.footswing_trajectories = [FootSwingTrajectory() for i in range(4)]


    def initialize_static_gait(self, offsets, durations, iters, vel, vel_rpy, initial_pose):
        self.pose_table, self.rpy_table, self.contact_table  = np.zeros((3, self.planning_horizon)), np.zeros((3, self.planning_horizon)), np.ones((4, self.planning_horizon))

        # contact plan
        for i in range(self.planning_horizon):
            for j in range(4):
                if (offsets[j]+durations[j] > i and i >= offsets[j]) or (offsets[j] + durations[j] > self.planning_horizon and i < offsets[i] + durations[j] - self.planning_horizon):
                    self.contact_table[j, i] = 1
                else:
                    self.contact_table[j, i] = 0
        
        # contact plan accounting
        self.contact_idx, self.liftoff_idx, self.next_contact_idx, self.next_liftoff_idx = np.zeros(4), np.zeros(4), offsets, offsets+durations

        self.swing_duration = (self.planning_horizon - durations) * self.iterationsBetweenMPC
        self.swing_start = durations * self.iterationsBetweenMPC
        self.contact_duration = durations * self.iterationsBetweenMPC
        self.contact_start = offsets

        self.swing_progress, self.contact_progress, self.contact_iteration, self.liftoff_iteration = np.zeros(4), np.zeros(4), np.zeros(4), np.zeros(4)

        self.next_contact_iteration = offsets * self.iterationsBetweenMPC
        self.next_liftoff_iteration = (offsets + durations) * self.iterationsBetweenMPC

        # body pose plan
        self.pose_table[:, 0] = [initial_pose[0], initial_pose[1], 0.29]
        self.rpy_table[:, 0] = 0
        self.iterations_table = self.iterationsBetweenMPC * np.ones((1, self.planning_horizon))
        self.vel_table = np.tile(vel.reshape((3, 1)), (1, self.planning_horizon))
        self.vel_rpy_table = np.tile(vel_rpy.reshape((3, 1)), (1, self.planning_horizon))
        for i in range(1, self.planning_horizon):
            self.pose_table[:, i] = self.pose_table[:, i-1] + vel * self.iterationsBetweenMPC * self.dt
            self.rpy_table[:, i] = self.rpy_table[:, i-1] + vel_rpy * self.iterationsBetweenMPC * self.dt

        # globally referenced timing variables
        self.iteration_prev_update = 0
        self.current_iteration = 0
        #self.iteration = 0

        # start standing
        self.contact_table = np.concatenate((np.ones((4, 1)), self.contact_table), axis=1)


    def update(self, offset, adaptation_steps, contact_table, vel_table, vel_rpy_table, iteration_table):
        # TODO: implement offset
        if self._due_for_mpc_update():
            # compute future pose
            pose_table_update, rpy_table_update = np.zeros_like(vel_table), np.zeros_like(vel_rpy_table)
            pose_table_update[:, 0] = self.pose_table[:, -1] + self.vel_table[:, -1] * self.iterationsBetweenMPC * self.dt
            rpy_table_update[:, 0] = self.rpy_table[:, -1] + self.vel_rpy_table[:, -1] * self.iterationsBetweenMPC * self.dt
            for i in range(1, adaptation_steps):
                pose_table_update[:, i] = pose_table_update[:, i-1] + vel_table[:, i-1] * self.iterationsBetweenMPC * self.dt
                rpy_table_update[:, i] = rpy_table_update[:, i-1] + vel_rpy_table[:, i-1] * self.iterationsBetweenMPC * self.dt

            # update
            self.pose_table = np.concatenate((self.pose_table, pose_table_update[:, :adaptation_steps]), axis=1)
            self.rpy_table = np.concatenate((self.rpy_table, rpy_table_update[:, :adaptation_steps]), axis=1)
            self.vel_table = np.concatenate((self.vel_table, vel_table[:, :adaptation_steps]), axis=1)
            self.vel_rpy_table = np.concatenate((self.vel_rpy_table, vel_rpy_table[:, :adaptation_steps]), axis=1)

            self.iterations_table = np.concatenate((self.iterations_table, iteration_table[:adaptation_steps]), axis=1)
            self.contact_table = np.concatenate((self.contact_table, contact_table[:, :adaptation_steps]), axis=1)

            #input((self.contact_table, contact_table))
            #print(self.contact_table)
  
    def step(self, foot_locations, low_level_state):
        # robot state is only used to set footswing trajectories

        if self._due_for_mpc_update():

            # step tables forward
            self.pose_table, self.rpy_table, self.vel_table, self.vel_rpy_table = self.pose_table[:, 1:], self.rpy_table[:, 1:], self.vel_table[:, 1:], self.vel_rpy_table[:, 1:]
            self.contact_table, self.iterations_table = self.contact_table[:, 1:], self.iterations_table[:, 1:]

            # update contact plan accounting
            for i in range(4):
                if self._foot_in_swing(i):
                    if self._just_broke_contact(i): 
                        self.liftoff_iteration[i] = self.current_iteration
                        next_contact_idx, next_liftoff_idx = self._scan_for_next_contact(i)
                        self.next_contact_iteration[i] = self.current_iteration + next_contact_idx * self.iterationsBetweenMPC
                        self.next_liftoff_iteration[i] = self.current_iteration + next_liftoff_idx * self.iterationsBetweenMPC
                        self.swing_duration[i] = self.next_contact_iteration[i] - self.liftoff_iteration[i]

                        self._update_footswing_trajectory(i, initial_position=foot_locations[i], low_level_state = low_level_state)

                    #self._swing[i] = self.next_contact_iteration[i] - self.liftoff_iteration[i]
                    #self._stance[i] = self.next_liftoff_iteration[i] - self.next_contact_iteration[i]
                    # self.swing_progress[i] = (self.current_iteration - self.liftoff_iteration[i]) / self.swing_duration[i]
                    # self.contact_progress[i] = (0.0 if (contact_progress[i] >= 1.0 or contact_progress[i] == 0) else 1.0)
                elif self._foot_in_contact(i):
                    if self._just_made_contact(i):
                        self.contact_iteration[i] = self.current_iteration
                        next_liftoff_idx, next_contact_idx = self._scan_for_next_liftoff(i)
                        self.next_liftoff_iteration[i] = self.current_iteration + next_liftoff_idx * self.iterationsBetweenMPC
                        self.next_contact_iteration[i] = self.current_iteration + next_contact_idx * self.iterationsBetweenMPC
                        self.contact_duration[i] = self.next_liftoff_iteration[i] - self.contact_iteration[i]
                    
                    #self._swing[i] = self.next_contact_iteration[i] - self.next_liftoff_iteration[i]
                    #self._stance[i] = self.self.next_liftoff_iteration[i] - self.contact_iteration[i]
                    # self.contact_progress[i] = (self.current_iteration - self.contact_iteration[i]) / self.contact_duration[i]
                    # self.swing_progress[i] = (0.0 if (swing_progress[i] >= 1.0 or swing_progress[i] == 0) else 1.0)

            self.iteration_prev_update = self.current_iteration

        #else:
        # update swing foot states
        for i in range(4):
            if self._foot_in_swing(i):
                self.swing_progress[i] = (self.current_iteration - self.liftoff_iteration[i]) / self.swing_duration[i]
                self.contact_progress[i] = (0.0 if (self.contact_progress[i] >= 1.0 or self.contact_progress[i] == 0) else 1.0)
            elif self._foot_in_contact(i):
                self.contact_progress[i] = (self.current_iteration - self.contact_iteration[i]) / self.contact_duration[i]
                self.swing_progress[i] = (0.0 if (self.swing_progress[i] >= 1.0 or self.swing_progress[i] == 0) else 1.0)


        self.current_iteration += 1

        #print(self.swing_duration)

    def to_wbc_cmd(self, low_level_state):
        self._compute_current_footswings()

        grounded_pos = np.array([low_level_state.body_pos[0], low_level_state.body_pos[1], 0])

        wbc_cmd = WBCLevelCmd()
        wbc_cmd.pBody_des = self.pose_table[:, 0]
        wbc_cmd.vBody_des = self.vel_table[:, 0]
        wbc_cmd.aBody_des = np.zeros(3)
        wbc_cmd.pBody_RPY_des = self.rpy_table[:, 0]
        wbc_cmd.vBody_Ori_des = self.vel_rpy_table[:, 0]
        wbc_cmd.pFoot_des = self._get_pfoot_des()# + np.tile(grounded_pos, (4))
        wbc_cmd.vFoot_des = self._get_vfoot_des()
        wbc_cmd.aFoot_des = self._get_afoot_des()
        wbc_cmd.contact_state = self.contact_table[:, 1]
        wbc_cmd.Fr_des = np.zeros(4) # to be computed by MPC
        
        return wbc_cmd

    def get_traj(self, low_level_state):
        USE_VEL_CONTROL = True
        COMP_RPY = True

        if COMP_RPY: # in the original code, this is turned off for pronking. we should consider keeping it off
            # heuristic integral-esque pitch and roll compensation
            if(abs(low_level_state.body_linear_vel[0]) > 0.2):
                self.rpy_int[1] += self.dt * (self.rpy_table[1, 0] - low_level_state.body_rpy[1]) / low_level_state.body_linear_vel[0]
            if(abs(low_level_state.body_linear_vel[1]) > 0.1):
                self.rpy_int[0] += self.dt * (self.rpy_table[0, 0] - low_level_state.body_rpy[0]) / low_level_state.body_linear_vel[1]
            
            rpy_comp = [low_level_state.body_linear_vel[1] * self.rpy_int[0], low_level_state.body_linear_vel[0] * self.rpy_int[1], low_level_state.body_rpy[2]]

        
        if USE_VEL_CONTROL: # not totally aligned yet
            trajAll = np.zeros((12, self.planning_horizon))
            start_pose = np.array([low_level_state.body_pos[0], low_level_state.body_pos[1], 0.29])
            #print(rpy_comp, start_pose, self.vel_rpy_table[:, 0], self.vel_table[:, 0])
            trajAll[:, 0] = np.concatenate((rpy_comp, start_pose, self.vel_rpy_table[:, 0], self.vel_table[:, 0]))
            for i in range(1, self.planning_horizon):
                trajAll[0:6, i] = trajAll[0:6, i-1] + trajAll[6:12, i-1] * self.dt * self.iterations_table[:, i-1]
        else:
            trajAll = np.concatenate((self.rpy_table[:, :self.planning_horizon], self.pose_table[:, :self.planning_horizon], self.vel_rpy_table[:, :self.planning_horizon], self.vel_table[:, :self.planning_horizon]))
        
        return trajAll.T.flatten()

    def get_mpc_table(self):
        return self.contact_table[:, 1:self.planning_horizon+1]

    def reset(self):
        self.iteration_prev_update = 0
        self.current_iteration = 0
        self.rpy_int = [0, 0]
        self.footswing_trajectories = [FootSwingTrajectory() for i in range(4)]
        
        self.initialize_static_gait(    offsets = np.array([0, 0, 0, 0]), 
                                        durations = np.array([10, 10, 10, 10]), 
                                        iters = 17, 
                                        vel = np.zeros(3), 
                                        vel_rpy = np.zeros(3), 
                                        initial_pose = np.array([0, 0, 0.29])
                                    )

    def _due_for_mpc_update(self):
        return self.current_iteration - self.iteration_prev_update >= self.iterationsBetweenMPC or self.current_iteration == 0

    def _just_broke_contact(self, foot):
        return self.contact_table[foot, 1] == 0 and self.contact_table[foot, 0] == 1

    def _just_made_contact(self, foot):
        return self.contact_table[foot, 1] == 1 and self.contact_table[foot, 0] == 0

    def _foot_in_swing(self, foot):
        return self.contact_table[foot, 1] == 0

    def _foot_in_contact(self, foot):
        return self.contact_table[foot, 1] == 1

    def _scan_for_next_contact(self, foot):
        # returns the relative start and end indices of the next contact.
        # TODO account for contact longer than the planned horizon
        contact_start, contact_duration = -1, -1
        for j in range(2, self.contact_table.shape[1]):
            if contact_duration == -1 and contact_start == -1 and self.contact_table[foot, j] == 1:
                contact_start = j - 1
            elif contact_start != -1 and self.contact_table[foot, j] == 0:
                contact_duration = j - 1 - contact_start
                return contact_start, contact_start + contact_duration
        return contact_start, -1

    def _scan_for_next_liftoff(self, foot):
        # returns the relative start and end indices of the next swing.
        # TODO account for swings longer than the planned horizon
        swing_start, swing_duration = -1, -1
        for j in range(2, self.contact_table.shape[1]):
            if swing_duration == -1 and swing_start == -1 and self.contact_table[foot, j] == 0:
                swing_start = j - 1
            elif swing_start != -1 and self.contact_table[foot, j] == 1:
                swing_duration = j - 1 - swing_start
                return swing_start, swing_start + swing_duration
        return swing_start, -1

    def _update_footswing_trajectory(self, foot, initial_position, low_level_state):
        
        self.footswing_trajectories[foot].setInitialPosition(initial_position) # liftoff position
        self.footswing_trajectories[foot].setHeight(0.06) # make adjustable later

        grounded_pos = np.array([low_level_state.body_pos[0], low_level_state.body_pos[1], 0])
        #print(grounded_pos)
        
        #pfoot_f = grounded_pos + 0.5 * self.vel_table[:, 0] * self.swing_duration[foot] * self.dt# + self.traj[-1].pos # raibert heuristic -- add angular component later!
        
        pRobotFrame = np.zeros(3)
        pRobotFrame[0] = -0.19 if foot in [2, 3] else 0.19
        pRobotFrame[1] = -0.114 if foot in [1, 3] else 0.114
        theta = -self.vel_rpy_table[2, 0] * 0.5 * self.contact_duration[foot] * self.dt
        pYawCorrected = np.dot(np.array([[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0], [0, 0, 1]]), pRobotFrame)
        Pf = low_level_state.body_pos + np.dot(get_rotation_matrix_from_rpy(low_level_state.body_rpy).T, ( low_level_state.body_linear_vel * self.swing_duration[foot] * self.dt + pYawCorrected))

        pfx_rel = 0.5 * low_level_state.body_linear_vel[0] * self.swing_duration[foot] * self.dt + \
                    0.03 * (low_level_state.body_linear_vel[0]-self.vel_table[0,0]) + \
                    0.5 * (low_level_state.body_pos[2]/9.81) * (low_level_state.body_linear_vel[1] * self.vel_rpy_table[2,0])
        pfy_rel = 0.5 * low_level_state.body_linear_vel[1] * self.swing_duration[foot] * self.dt + \
                    0.03 * (low_level_state.body_linear_vel[1]-self.vel_table[1,0]) + \
                    0.5 * (low_level_state.body_pos[2]/9.81) * (-low_level_state.body_linear_vel[0] * self.vel_rpy_table[2,0])


        Pf[0] += pfx_rel
        Pf[1] += pfy_rel
        Pf[2] = 0.
        # pfoot_f = grounded_pos + 0.5 * low_level_state.body_linear_vel * self.swing_duration[foot] * self.dt# + self.traj[-1].pos # raibert heuristic -- add angular component later!
        # pfoot_f[0] += (-0.2 if foot in [2, 3] else 0.15) # side offset (correct for yaw later)
        # pfoot_f[1] += (-0.12 if foot in [1, 3] else 0.12) # side offset (correct for yaw later)
        # pfoot_f[2] = 0
        self.footswing_trajectories[foot].setFinalPosition(Pf) # landing position

        #print("swingbounds", foot, initial_position, Pf)

    def _compute_current_footswings(self):
        for i in range(4):
            phase = self.swing_progress[i]
            swingTime = self.swing_duration[i] * self.dt
            self.footswing_trajectories[i].computeSwingTrajectoryBezier(phase, swingTime)

    def _get_pfoot_des(self):
        return np.concatenate([self.footswing_trajectories[i].getPosition() for i in range(4)])

    def _get_vfoot_des(self):
        return np.concatenate([self.footswing_trajectories[i].getVelocity() for i in range(4)])

    def _get_afoot_des(self):
        return np.concatenate([self.footswing_trajectories[i].getAcceleration() for i in range(4)])


    def plot_traj(self):

        foot_poses = np.zeros((20, 12))

        for j in range(20):
            for i in range(4):
                swingTime = self.swing_duration[i] * self.dt
                phase_int = self.swing_progress[i] + (i/20.)*(1-self.swing_progress[i])
                self.footswing_trajectories[i].computeSwingTrajectoryBezier(phase_int, swingTime)
            foot_poses[j, :] = self._get_pfoot_des()

        # z axis

        self.axs[0].clear()

        self.axs[0].plot(range(self.pose_table.shape[0]), self.pose_table[:, 2], 'k')
        colors = ['green', 'blue', 'red', 'orange']
        for f in range(4):
            self.axs[0].plot(range(foot_poses.shape[0]), foot_poses[:, f*3+2], '--', color=colors[f])
        self.axs[0].legend(["Body", "LF Foot", "RF Foot", "LR Foot", "RR Foot"], prop={'size': 14})

        # x axis

        self.axs[1].clear()

        self.axs[1].plot(range(body_poses.shape[0]), body_poses[:, 0], 'k')
        colors = ['green', 'blue', 'red', 'orange']
        for f in range(4):
            self.axs[1].plot(range(foot_poses.shape[0]), foot_poses[:, f*3+0], '--', color=colors[f])
        self.axs[1].legend(["Body", "LF Foot", "RF Foot", "LR Foot", "RR Foot"], prop={'size': 14})

        # y axis

        self.axs[2].clear()

        self.axs[2].plot(range(body_poses.shape[0]), body_poses[:, 1], 'k')
        colors = ['green', 'blue', 'red', 'orange']
        for f in range(4):
            self.axs[2].plot(range(foot_poses.shape[0]), foot_poses[:, f*3+1], '--', color=colors[f])
        self.axs[2].legend(["Body", "LF Foot", "RF Foot", "LR Foot", "RR Foot"], prop={'size': 14})



        plt.waitforbuttonpress()
      


if __name__ == "__main__":

    pass