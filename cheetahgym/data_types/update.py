import numpy as np
from tabulate import tabulate
import copy

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion
from cheetahgym.utils.foot_swing_trajectory import FootSwingTrajectory
from cheetahgym.data_types.wbc_level_types import WBCLevelCmd
from cheetahgym.systems.pybullet_system import PyBulletSystem

import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass


class OnestepTarget:

    def __init__(self, pos, rpy, vel, vel_rpy, pfoot, vfoot, afoot, contact_forces, contact_state):

        self.pos = pos
        self.rpy = rpy
        self.vel = vel
        self.vel_rpy = vel_rpy
        self.pfoot = pfoot
        self.vfoot = vfoot
        self.afoot = afoot
        self.contact_forces = contact_forces
        self.contact_state = contact_state

    def to_wbc_cmd(self):
        wbc_cmd = WBCLevelCmd()
        wbc_cmd.pBody_des = self.pos
        wbc_cmd.vBody_des = self.vel
        wbc_cmd.aBody_des = np.zeros(3)
        wbc_cmd.pBody_RPY_des = self.rpy
        wbc_cmd.vBody_Ori_des =self.vel_rpy
        wbc_cmd.pFoot_des = np.concatenate([self.pfoot[3*i:3*i+3] + [self.pos[0], self.pos[1], 0] for i in range(4)])
        wbc_cmd.vFoot_des = self.vfoot
        wbc_cmd.aFoot_des = self.afoot
        wbc_cmd.contact_state = self.contact_state
        wbc_cmd.Fr_des = self.contact_forces
        
        return wbc_cmd



class Trajectory:

    def __init__(self, dt):
        self.dt = dt
        self.traj = []
        self.traj_history = []

        plt.figure()

    def get_target(self):
        if len(self.traj) == 0:
            raise Exception("Trajectory queried but not yet populated!")
        return self.traj[0]

    def get_nstep_target(self, n):
        if len(self.traj) < n:
            raise Exception("Trajectory queried but not yet populated!")
        return self.traj[0:n]

    def step_forward(self):
        if len(self.traj) == 0:
            raise Exception("Trajectory queried but not yet populated!")
        self.traj_history += [self.traj[0]]
        return self.traj.pop(0)

    def step_forward_cycle(self):
        if len(self.traj) == 0:
            raise Exception("Trajectory queried but not yet populated!")
        self.traj_history += [self.traj[0]]
        traj_last = self.traj.pop(0)
        self.traj += [copy.deepcopy(traj_last)]
        self.traj[-1].pos = self.traj[-2].pos + self.traj[-2].vel * self.dt
        #for i in range(4):
          #print(self.traj[-2].pfoot[i*3:i*3+3])
          #input()
          #self.traj[-1].pfoot[i*3:i*3+3] = self.traj[-2].pfoot[i*3:i*3+3] + self.traj[-2].vel * self.dt
        
        return traj_last

    def set_defaults(self, num_steps, default_target):
        self.traj = [copy.deepcopy(default_target)] * num_steps


    def print_traj(self):
        tab_list = []
        for t in self.traj_history:
            tab_list += [[#t.pos, 
                          #t.rpy, 
                          #t.vel, 
                          #t.vel_rpy, 
                          t.pfoot[0:3], 
                          #t.vfoot, 
                          #t.afoot, 
                          t.contact_forces, 
                          t.contact_state
                        ]]

        print(tabulate(tab_list, headers=[#'pos', 
                                          #'rpy', 
                                          #'vel', 
                                          #'vel_rpy', 
                                          'pfoot', 
                                          #'vfoot', 
                                          #'afoot', 
                                          'contact_forces', 
                                          'contact_state'
                                          ], tablefmt='orgtbl'))

    def plot_traj(self):
        body_poses = np.array([t.pos for t in self.traj])
        foot_poses = np.array([t.pfoot for t in self.traj])

        print(body_poses.shape)

        plt.cla()

        plt.plot(range(body_poses.shape[0]), body_poses[:, 0], 'k')
        colors = ['green', 'blue', 'red', 'orange']
        for f in range(4):
            plt.plot(range(foot_poses.shape[0]), foot_poses[:, f*3+2], '--', color=colors[f])
        plt.legend(["Body", "LF Foot", "RF Foot", "LR Foot", "RR Foot"], prop={'size': 14})

        plt.waitforbuttonpress()

class RaibertBezierTrajectory(Trajectory):

    def __init__(self, dt, footswing_height):

        self.footswing_height = footswing_height
        self.footswing_trajectories = [FootSwingTrajectory() for i in range(4)]
        for i in range(4): 
            self.footswing_trajectories[i].setHeight(self.footswing_height)
        self.swing_times = np.zeros(4)
        self.stance_times = np.zeros(4)

        self.prev_contact_table = np.ones(4)
        self.times_since_liftoff = np.zeros(4)

        super().__init__(dt)

    def repopulate_footswing_trajectories(self):

        num_steps = self.contact_table.shape[0] * self.iterationsBetweenContact

        for i in range(4):
            pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
            self.footswing_trajectories[i].setInitialPosition(pfoot_i) # initial position
            self.footswing_trajectories[i].setFinalPosition(pfoot_i) # initial position
            self.swing_times = np.ones(4) * 5 * self.iterationsBetweenContact * self.dt

        foot_fully_computed = [0, 0, 0, 0]

        #input(self.contact_table)

        for step in range(0, num_steps):

            cur_time = self.dt * step
            contact_step = step // self.iterationsBetweenContact

            pfoot = np.zeros(12)
            vfoot = np.zeros(12)
            afoot = np.zeros(12)

            # manage bezier trajectories

            if step % self.iterationsBetweenContact == 0:

              # update contact states
              for i in range(4): # each foot

                  if (self.contact_table[contact_step-1, i] == 1 and self.contact_table[contact_step, i] == 0): # contact broken
                      self.swing_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder
                      pfoot_i = self.traj[step].pfoot[3*i:3*i+3]
                      self.footswing_trajectories[i].setInitialPosition(pfoot_i) # liftoff position
                      pfoot_f = self.traj[step].pos + self.traj[step].vel * self.swing_times[i] # raibert heuristic -- add angular component later!
                      #print("PFOOTF", pfoot_f)
                      #input()
                      pfoot_f[0] += (-0.2 if i in [2, 3] else 0.2) # side offset (correct for yaw later)
                      pfoot_f[1] += (-0.1 if i in [1, 3] else 0.1) # side offset (correct for yaw later)
                      pfoot_f[2] = 0

                      self.footswing_trajectories[i].setFinalPosition(pfoot_f)

                      foot_fully_computed[i] += 1

                      #print(pfoot_i, pfoot_f)

                  elif self.contact_table[contact_step-1, i] == 0 and self.contact_table[contact_step, i] == 1: # contact made
                      self.stance_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder

                      foot_fully_computed[i] += 1

            # query bezier trajectories

            for i in range(4): # each foot



                if foot_fully_computed[i] >= 2:
                    continue

                swingTime = 5 * self.iterationsBetweenContact * self.dt
                self.footswing_trajectories[i].computeSwingTrajectoryBezier(1 - self.swing_times[i] / swingTime, swingTime)
                pfoot[3*i:3*i+3] = self.footswing_trajectories[i].getPosition()
                vfoot[3*i:3*i+3] = self.footswing_trajectories[i].getVelocity()
                afoot[3*i:3*i+3] = self.footswing_trajectories[i].getAcceleration()

                #print(step, contact_table[step//self.iterationsBetweenContact, i], i, foot_fully_computed, 1 - self.swing_times[i] / (5 * self.iterationsBetweenContact * self.dt), pfoot[3*i:3*i+3])

                if self.contact_table[contact_step, i] == 1:
                    self.swing_times[i] -= self.dt
                else:
                    self.stance_times[i] -= self.dt

            #print(self.swing_times)

            self.traj[i].pfoot = pfoot
            self.traj[i].vfoot = vfoot
            self.traj[i].afoot = afoot

    def _get_time_since_liftoff(self, foot_idx):
        return self.times_since_liftoff[foot_idx]

    def _update_time_since_liftoff(self, current_contact_table):
        self.times_since_liftoff += 1
        for i in range(4):
          if current_contact_table[i]==0 and self.prev_contact_table[i]==1:
            self.times_since_liftoff[i] = 0
        self.prev_contact_table = current_contact_table[:]

        #print("TSL", self.times_since_liftoff)


    def set_constant_gait(self, contact_table, initial_target, iterationsBetweenContact=None):
        
        self.contact_table = contact_table

        if iterationsBetweenContact is not None:
          self.iterationsBetweenContact = iterationsBetweenContact

        num_steps = self.contact_table.shape[0] * self.iterationsBetweenContact

        # initialization

        self.traj = [initial_target]
        for i in range(4):
            pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
            self.footswing_trajectories[i].setInitialPosition(pfoot_i) # initial position
            self.footswing_trajectories[i].setFinalPosition(pfoot_i) # initial position
            self.swing_times = np.zeros(4)#np.ones(4) * 5 * self.iterationsBetweenContact * self.dt



        for step in range(0, num_steps):

            cur_time = self.dt * step
            contact_step = step // self.iterationsBetweenContact

            pfoot = np.zeros(12)
            vfoot = np.zeros(12)
            afoot = np.zeros(12)

            # manage bezier trajectories

            if step % self.iterationsBetweenContact == 0:

              # update contact states
              for i in range(4): # each foot

                  if (self.contact_table[contact_step-1, i] == 1 or step == 0) and self.contact_table[contact_step, i] == 0: # contact broken
                      self.swing_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder
                      pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
                      #print(self.traj[-1].pfoot)
                      #if len(self.traj) > 2: print(self.traj[-20].pfoot)
                      #input()
                      self.footswing_trajectories[i].setInitialPosition(pfoot_i) # liftoff position
                      pfoot_f = self.traj[-1].pos + self.traj[-1].vel * self.swing_times[i] # raibert heuristic -- add angular component later!
                      #print("PFOOTF", pfoot_f)
                      #input()
                      pfoot_f[0] += (-0.2 if i in [2, 3] else 0.15) # side offset (correct for yaw later)
                      pfoot_f[1] += (-0.12 if i in [1, 3] else 0.12) # side offset (correct for yaw later)
                      pfoot_f[2] = 0

                      #pfoot_f[0] += 0.05

                      self.footswing_trajectories[i].setFinalPosition(pfoot_f)

                      #print(pfoot_i, pfoot_f)

                  elif (self.contact_table[contact_step-1, i] == 0  or step == 0) and self.contact_table[contact_step, i] == 1: # contact made
                      self.stance_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder

            # query bezier trajectories
            #print(self.swing_times, self.stance_times)

            for i in range(4): # each foot
                swingTime = 5 * self.iterationsBetweenContact * self.dt
                if self.contact_table[contact_step, i] == 1: # in swing
                    self.footswing_trajectories[i].computeSwingTrajectoryBezier(1, swingTime)
                    self.stance_times[i] -= self.dt
                else: # in swing
                    self.footswing_trajectories[i].computeSwingTrajectoryBezier(1 - self.swing_times[i] / swingTime, swingTime)
                    self.swing_times[i] -= self.dt
                pfoot[3*i:3*i+3] = self.footswing_trajectories[i].getPosition()
                vfoot[3*i:3*i+3] = self.footswing_trajectories[i].getVelocity()
                afoot[3*i:3*i+3] = self.footswing_trajectories[i].getAcceleration()
                

            #print(self.swing_times)

            #input((pfoot[0:3], contact_table[step // self.iterationsBetweenContact, :]))


            target = OnestepTarget(
                            pos = initial_target.pos + initial_target.vel * cur_time,
                            rpy = initial_target.rpy + initial_target.vel_rpy * cur_time,
                            vel = initial_target.vel,
                            vel_rpy = initial_target.vel_rpy,
                            pfoot = pfoot,
                            vfoot = vfoot,
                            afoot = afoot,
                            contact_forces = np.zeros(4),
                            contact_state = contact_table[step // self.iterationsBetweenContact, :]
                        )


            self.traj += [target]

    def replan(self, contact_table, initial_target, iterationsBetweenContact=None):
        
        self.contact_table = contact_table

        self._update_time_since_liftoff(initial_target.contact_state)

        if iterationsBetweenContact is not None:
          self.iterationsBetweenContact = iterationsBetweenContact

        num_steps = self.contact_table.shape[0] * self.iterationsBetweenContact

        # initialization

        self.traj = [initial_target]
        for i in range(4):
            pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
            self.footswing_trajectories[i].setInitialPosition(pfoot_i) # initial position
            self.footswing_trajectories[i].setFinalPosition(pfoot_i) # initial position
            self.swing_times = np.zeros(4)#np.ones(4) * 5 * self.iterationsBetweenContact * self.dt



        for step in range(0, num_steps):

            cur_time = self.dt * step
            contact_step = step // self.iterationsBetweenContact

            pfoot = np.zeros(12)
            vfoot = np.zeros(12)
            afoot = np.zeros(12)

            # manage bezier trajectories

            if step % self.iterationsBetweenContact == 0:

              # update contact states
              for i in range(4): # each foot

                  if (self.contact_table[contact_step-1, i] == 1 or step == 0) and self.contact_table[contact_step, i] == 0: # contact broken
                      if step == 0:
                        self.swing_times[i] = (5 - self._get_time_since_liftoff(foot_idx=i)) * self.iterationsBetweenContact * self.dt
                      else:
                        self.swing_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder
                      pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
                      #print(self.traj[-1].pfoot)
                      #if len(self.traj) > 2: print(self.traj[-20].pfoot)
                      #input()
                      self.footswing_trajectories[i].setInitialPosition(pfoot_i) # liftoff position
                      pfoot_f = self.traj[-1].vel * self.swing_times[i]  #+ self.traj[-1].pos + # raibert heuristic -- add angular component later!
                      #print("PFOOTF", pfoot_f)
                      #input()
                      pfoot_f[0] += (-0.2 if i in [2, 3] else 0.15) # side offset (correct for yaw later)
                      pfoot_f[1] += (-0.11 if i in [1, 3] else 0.11) # side offset (correct for yaw later)
                      pfoot_f[2] = 0

                      #pfoot_f[0] += 0.05

                      self.footswing_trajectories[i].setFinalPosition(pfoot_f)

                      #print(pfoot_i, pfoot_f)

                  elif (self.contact_table[contact_step-1, i] == 0  or step == 0) and self.contact_table[contact_step, i] == 1: # contact made
                      self.stance_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder

            # query bezier trajectories
            #print(self.swing_times, self.stance_times)

            for i in range(4): # each foot
                swingTime = 5 * self.iterationsBetweenContact * self.dt
                if self.contact_table[contact_step, i] == 1: # in swing
                    self.footswing_trajectories[i].computeSwingTrajectoryBezier(1, swingTime)
                    self.stance_times[i] -= self.dt
                else: # in swing
                    self.footswing_trajectories[i].computeSwingTrajectoryBezier(1 - self.swing_times[i] / swingTime, swingTime)
                    self.swing_times[i] -= self.dt
                pfoot[3*i:3*i+3] = self.footswing_trajectories[i].getPosition()
                vfoot[3*i:3*i+3] = self.footswing_trajectories[i].getVelocity()
                afoot[3*i:3*i+3] = self.footswing_trajectories[i].getAcceleration()
                

            #print(self.swing_times)


            target = OnestepTarget(
                            pos = initial_target.pos + initial_target.vel * cur_time,
                            rpy = initial_target.rpy + initial_target.vel_rpy * cur_time,
                            vel = initial_target.vel,
                            vel_rpy = initial_target.vel_rpy,
                            pfoot = pfoot,
                            vfoot = vfoot,
                            afoot = afoot,
                            contact_forces = np.zeros(4),
                            contact_state = contact_table[step // self.iterationsBetweenContact, :]
                        )
            self.traj += [target]



    def replan_traj(self, initial_target, steps_so_far, iterationsBetweenContact=None):

        if iterationsBetweenContact is not None:
          self.iterationsBetweenContact = iterationsBetweenContact

        num_steps = self.contact_table.shape[0] * self.iterationsBetweenContact

        # initialization

        #self.traj = [initial_target]
        # for i in range(4):
        #     pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
        #     self.footswing_trajectories[i].setInitialPosition(pfoot_i) # initial position
        #     self.footswing_trajectories[i].setFinalPosition(pfoot_i) # initial position
        #     self.swing_times = np.zeros(4)#np.ones(4) * 5 * self.iterationsBetweenContact * self.dt



        for it in range(num_steps):

            step = (it + steps_so_far) % num_steps

            cur_time = self.dt * step
            # contact_step = step // self.iterationsBetweenContact

            # pfoot = np.zeros(12)
            # vfoot = np.zeros(12)
            # afoot = np.zeros(12)

            # # manage bezier trajectories

            # if step % self.iterationsBetweenContact == 0:

            #   # update contact states
            #   for i in range(4): # each foot

            #       if (self.contact_table[contact_step-1, i] == 1 or step == 0) and self.contact_table[contact_step, i] == 0: # contact broken
            #           self.swing_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder
            #           pfoot_i = self.traj[-1].pfoot[3*i:3*i+3]
            #           #print(self.traj[-1].pfoot)
            #           #if len(self.traj) > 2: print(self.traj[-20].pfoot)
            #           #input()
            #           self.footswing_trajectories[i].setInitialPosition(pfoot_i) # liftoff position
            #           pfoot_f = self.traj[-1].pos + self.traj[-1].vel * self.swing_times[i] # raibert heuristic -- add angular component later!
            #           #print("PFOOTF", pfoot_f)
            #           #input()
            #           pfoot_f[0] += (-0.2 if i in [2, 3] else 0.1) # side offset (correct for yaw later)
            #           pfoot_f[1] += (-0.1 if i in [1, 3] else 0.1) # side offset (correct for yaw later)
            #           pfoot_f[2] = 0

            #           pfoot_f[0] += 0.05

            #           self.footswing_trajectories[i].setFinalPosition(pfoot_f)

            #           #print(pfoot_i, pfoot_f)

            #       elif (self.contact_table[contact_step-1, i] == 0  or step == 0) and self.contact_table[contact_step, i] == 1: # contact made
            #           self.stance_times[i] = 5 * self.iterationsBetweenContact * self.dt # placeholder

            # # query bezier trajectories
            # #print(self.swing_times, self.stance_times)

            # for i in range(4): # each foot
            #     swingTime = 5 * self.iterationsBetweenContact * self.dt
            #     if self.contact_table[contact_step, i] == 1: # in swing
            #         self.footswing_trajectories[i].computeSwingTrajectoryBezier(1, swingTime)
            #         self.stance_times[i] -= self.dt
            #     else: # in swing
            #         self.footswing_trajectories[i].computeSwingTrajectoryBezier(1 - self.swing_times[i] / swingTime, swingTime)
            #         self.swing_times[i] -= self.dt
            #     pfoot[3*i:3*i+3] = self.footswing_trajectories[i].getPosition()
            #     vfoot[3*i:3*i+3] = self.footswing_trajectories[i].getVelocity()
            #     afoot[3*i:3*i+3] = self.footswing_trajectories[i].getAcceleration()
                

            # #print(self.swing_times)

            # #input((pfoot[0:3], contact_table[step // self.iterationsBetweenContact, :]))


            # target = OnestepTarget(
            #                 pos = initial_target.pos + initial_target.vel * cur_time,
            #                 rpy = initial_target.rpy + initial_target.vel_rpy * cur_time,
            #                 vel = initial_target.vel,
            #                 vel_rpy = initial_target.vel_rpy,
            #                 pfoot = pfoot,
            #                 vfoot = vfoot,
            #                 afoot = afoot,
            #                 contact_forces = np.zeros(4),
            #                 contact_state = contact_table[step // self.iterationsBetweenContact, :]
            #             )


            # self.traj += [target]

            #print(len(self.traj), it)
            self.traj[it].pos = initial_target.pos + initial_target.vel * cur_time



class MPCSupportedRaibertBezierTrajectory(RaibertBezierTrajectory):

    def __init__(self, dt, footswing_height, mpc_controller=None):

        super().__init__(dt, footswing_height)

        # either pass in an mpc controller object or instantiate a new one
        if mpc_controller is None:
          from cheetahgym.utils.mpc_wbc_bindings import RSCheetahControllerV1
          #from cheetahgym.utils.mpc_wbc_bindings_forpublicrepo import RSCheetahControllerV1
          import pathlib
          self.mpc_controller = RSCheetahControllerV1(robot_filename=str(pathlib.Path(__file__).parent.absolute())+"/../config/quadruped-parameters.yaml",
                                                      estimator_filename=str(pathlib.Path(__file__).parent.absolute())+"/../config/quadruped-estimator-parameters.yaml")
        else:
          self.mpc_controller = mpc_controller

        self.iterationsBetweenContact = self.mpc_controller.iterationsBetweenMPC


    def populate_contact_forces(self, iter, low_level_state, foot_locations):

        trajAll = self.build_trajAll(low_level_state)
        iters_list = np.ones(20) * self.iterationsBetweenContact
        wbc_level_cmd = self.traj[0].to_wbc_cmd()
        mpc_table = self.build_mpc_table()
        rot_w_b = inversion(get_rotation_matrix_from_rpy(low_level_state.body_rpy))
        
        Fr_des = self.mpc_controller.mpc_compute_forces_from_traj(low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations)

        for i in range(self.iterationsBetweenContact):
          #print(Fr_des[[2, 5, 8, 11]])
          self.traj[i].contact_forces = Fr_des


    def build_trajAll(self, low_level_ob):
        # compute body traj table
        xStart, yStart = low_level_ob.body_pos[0], low_level_ob.body_pos[1]#self.world_position_desired[0], self.world_position_desired[1]
        max_pos_error=0.1
        if(xStart - low_level_ob.body_pos[0] > max_pos_error): xStart = low_level_ob.body_pos[0] + max_pos_error;
        if(low_level_ob.body_pos[0] - xStart > max_pos_error): xStart = low_level_ob.body_pos[0] - max_pos_error;

        if(yStart - low_level_ob.body_pos[1] > max_pos_error): yStart = low_level_ob.body_pos[1] + max_pos_error;
        if(low_level_ob.body_pos[1] - yStart > max_pos_error): yStart = low_level_ob.body_pos[1] - max_pos_error;

        yaw_des = self.traj[0].rpy[2]
        _body_height = self.traj[0].pos[2]

        #if abs(low_level_ob.body_linear_vel[0]) > 0.2:
        #    self.rpy_int[0] += self.dt * (self.pBody_RPY_des[1] - low_level_ob.body_rpy[1]) / low_level_ob.body_linear_vel[0]
        #if abs(low_level_ob.body_linear_vel[1]) > 0.1:
        #    self.rpy_int[0] += self.dt * (self.pBody_RPY_des[0] - low_level_ob.body_rpy[0]) / low_level_ob.body_linear_vel[1]
        #self.rpy_int = [min(max(self.rpy_int[0], -0.25), 0.25), min(max(self.rpy_int[1], -0.25), 0.25)]
        #rpy_comp = [low_level_ob.body_linear_vel[0] * self.rpy_int[1], low_level_ob.body_linear_vel[1] * self.rpy_int[0]]

        trajInitial = [0, #rpy_comp[0],
                       0, #rpy_comp[1],
                       yaw_des,
                       xStart,
                       yStart,
                       _body_height,
                       0,
                       0,
                       self.traj[0].vel_rpy[2],
                       self.traj[0].vel[0],
                       self.traj[0].vel[1],
                       0]

        trajAll = np.zeros(12*36)
        for i in range(min(30, len(self.traj))): # horizonLength
            #print(i, len(self.traj), i*self.iterationsBetweenContact)
            trajAll[12*i:12*i+12] = trajInitial[:]
            if i == 0:
                trajAll[2] = low_level_ob.body_rpy[2]
            else:
                trajAll[12*i+3] = trajAll[12*(i-1)+3] + self.dt  * self.iterationsBetweenContact * self.traj[i * self.iterationsBetweenContact].vel[0]
                trajAll[12*i+4] = trajAll[12*(i-1)+4] + self.dt  * self.iterationsBetweenContact * self.traj[i * self.iterationsBetweenContact].vel[1]
                trajAll[12*i+2] = trajAll[12*(i-1)+2] + self.dt  * self.iterationsBetweenContact * self.traj[i * self.iterationsBetweenContact].vel_rpy[2]

            #if i < 3:
            #    print(i, trajAll[12*i:12*i+12])

        return trajAll

    def build_mpc_table(self):

      mpc_table = np.zeros((4, 30))

      for i in range(min(30, len(self.traj))):
          mpc_table[:, i] = self.traj[i * self.iterationsBetweenContact].contact_state

      #print("mpc table", mpc_table)
      #input()

      return mpc_table

class NLPSupportedRaibertBezierTrajectory(RaibertBezierTrajectory):

    def __init__(self, dt, footswing_height, mpc_controller=None):

        super().__init__(dt, footswing_height)

        # either pass in an mpc controller object or instantiate a new one
        if mpc_controller is None:
          from cheetahgym.utils.mpc_wbc_bindings import RSCheetahControllerV1
          import pathlib
          self.mpc_controller = RSCheetahControllerV1(robot_filename=str(pathlib.Path(__file__).parent.absolute())+"/../config/quadruped-parameters.yaml",
                                                      estimator_filename=str(pathlib.Path(__file__).parent.absolute())+"/../config/quadruped-estimator-parameters.yaml")
        else:
          self.mpc_controller = mpc_controller

        # instantiate the NLP primer
        from cheetahgym.controllers.nlp_trajopt_matlab import NLPOptimizerM
        self.nlp_optimizer = NLPOptimizerM()

        self.iterationsBetweenContact = self.mpc_controller.iterationsBetweenMPC


    def optimize_trajectory(self):

        opt_step = 5


        # vectorize trajectory
        traj_vec_ref = np.zeros((len(self.traj) // opt_step, 75))
        contact_table = np.zeros((len(self.traj) // opt_step, 4))
        for i in range(12):#len(reference_trajectory)):
            t = self.traj[i*opt_step]
            traj_vec_ref[i, :] = np.concatenate((t.pos, t.rpy, np.zeros(12), t.vel_rpy, t.vel, np.zeros(12), # q, qd
                                                 t.pos, t.vel, np.zeros(3), # r, rd, rdd
                                                 t.pfoot, np.zeros(12), # pfoot, f_foot
                                                 np.zeros(3), np.zeros(3))) # h, hd
            contact_table[i, :] = t.contact_state

        #print(traj_vec_ref.shape)
        
        traj_vec_opt = self.nlp_optimizer.optimize_reference(traj_vec_ref, contact_table)

        optimized_traj = []

        # export vector to WBC commands
        for i in range(traj_vec_opt.shape[1] // opt_step):
            target_i = OnestepTarget( pos = traj_vec_opt[0:3, i],
                                      rpy = traj_vec_opt[3:6, i],
                                      vel = traj_vec_opt[21:24, i],
                                      vel_rpy = traj_vec_opt[18:21, i],
                                      pfoot = traj_vec_opt[42:57, i],
                                      vfoot = np.zeros(12),
                                      afoot = np.zeros(12),
                                      contact_forces = traj_vec_opt[57:69, i],
                                      contact_state = self.traj[i*opt_step].contact_state
                                    )
            #print(i, i*opt_step, self.traj[i*opt_step].contact_state)
            optimized_traj += [target_i] * opt_step

        self.traj = optimized_traj

        #print(self.traj)

        #for i in range(self.iterationsBetweenContact):
        #  self.traj[i].contact_forces = Fr_des


if __name__ == "__main__":

    
    #traj = RaibertBezierTrajectory(dt=0.01, footswing_height=0.05)
    traj = MPCSupportedRaibertBezierTrajectory(dt=0.002, footswing_height=0.05)
    #traj = NLPSupportedRaibertBezierTrajectory(dt=0.002, footswing_height=0.05)
    
    initial_target = OnestepTarget(
                            pos = np.array([0, 0, 0.3]),
                            rpy = np.array([0, 0, 0]),
                            vel = np.array([0., 0, 0]),
                            vel_rpy = np.array([0, 0, 0]),
                            pfoot = np.array([0.1, 0.10, 0.0,
                                              0.1, -0.10, 0.0,
                                              -0.2, 0.10, 0.0,
                                              -0.2, -0.10, 0.0,]),
                            vfoot = np.zeros(12),
                            afoot = np.zeros(12),
                            contact_forces = np.zeros(4),
                            #contact_state = np.array([1, 0, 0, 1])
                            contact_state = np.array([1, 1, 1, 1])
                        )

    contact_table = np.array([[1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],

                            ])
    
    contact_table = np.array([[1, 1, 1, 1],
                              [1, 1, 1, 1],
                              [1, 1, 1, 1],
                              [1, 1, 1, 1],
                              [1, 1, 1, 1],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0],
                              [0, 0, 0, 0],
                            ])

    # contact_table = np.ones((10, 4))
    

    num_cycle = 3
    contact_table = np.concatenate([contact_table for i in range(num_cycle)])

    traj.set_constant_gait(contact_table, initial_target)

    if isinstance(traj, MPCSupportedRaibertBezierTrajectory):
      
      

      simulator = PyBulletSystem(gui=True, mpc_controller=traj.mpc_controller, fix_body=False)
      state = simulator.reset_system()
      simulator.add_heightmap_array(np.zeros((200,200)), body_pos=[0, 0], resolution=0.02)

      for i in range(10000):

        if i % 13 == 0:
          # replan
          contact_table = np.concatenate((contact_table[1:, :], contact_table[0:1, :]))
          new_target = OnestepTarget(
                            pos = np.array([state.body_pos[0], state.body_pos[1], 0.3]),
                            rpy = np.array([0, 0, 0]),
                            vel = np.array([0.0, 0, 0]),
                            vel_rpy = np.array([0, 0, 0]),
                            pfoot = np.array([0.1, 0.10, 0.0,
                                              0.1, -0.10, 0.0,
                                              -0.2, 0.10, 0.0,
                                              -0.2, -0.10, 0.0,]),
                            vfoot = np.zeros(12),
                            afoot = np.zeros(12),
                            contact_forces = np.zeros(4),
                            #contact_state = np.array([1, 0, 0, 1])
                            contact_state = contact_table[0, :]
                        )
          #traj.replan_traj(new_target, steps_so_far=i//13)
          traj.replan(contact_table, new_target)
          traj.populate_contact_forces(iter = 0,
                                       low_level_state = state,
                                       foot_locations = simulator.get_foot_positions())

          #traj.plot_traj()

          #traj.repopulate_footswing_trajectories()

        #if i % 130 == 0 and i > 1:
        #  traj.plot_traj()

        target = traj.get_target().to_wbc_cmd()


        #input(target.pFoot_des[0:3])

        state, pd_cmd = simulator.step_state_wbc(target, state)

        #if i % 13 == 0:
        traj.step_forward_cycle()
        
        #print(len(traj.traj))
        #print('pfd', target.pFoot_des[2])
        #print('contact forces', target.Fr_des)
        #input()

    elif isinstance(traj, NLPSupportedRaibertBezierTrajectory):

      simulator = PyBulletSystem(gui=True, mpc_controller=traj.mpc_controller)#, fix_body=True)
      state = simulator.reset_system()
      simulator.add_heightmap_array(np.zeros((200,200)), body_pos=[0, 0], resolution=0.02)

      #traj.plot_traj()

      traj.optimize_trajectory()

      for i in range(len(traj.traj)):

        #if i % 13 == 0:
        #  traj.populate_contact_forces(iter = 0,
        #                               low_level_state = state,
        #                               foot_locations = simulator.get_foot_positions())

        target = traj.get_target().to_wbc_cmd()

        input()

        

        state, pd_cmd = simulator.step_state_wbc(target, state)

        #if i % 13 == 0:

        traj.step_forward()
        #print(len(traj.traj))
        #print('pfd', target.pFoot_des[2])
        #print('contact forces', target.Fr_des)
        #input()


    traj.print_traj()
    traj.plot_traj()
