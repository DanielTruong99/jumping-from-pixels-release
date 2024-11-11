"""
Gym environment for mini cheetah gait parameters.

"""

#import cv2
#import lcm
import time
import numpy as np
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rpy_from_quaternion
from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_rotation_matrix_from_rpy, inversion
from gym import Env
from gym import spaces

from cheetahgym.utils.heightmaps import FileReader, MapGenerator

from cheetahgym.systems.system import System

from cheetahgym.envs.cheetah_base_env import CheetahBaseEnv

from cheetahgym.data_types.mpc_level_types import MPCLevelCmd, MPCLevelState
from cheetahgym.data_types.low_level_types import LowLevelCmd, LowLevelState

from cheetahgym.data_types.camera_parameters import cameraParameters

import cProfile


import json

from math import exp, isnan

import copy


import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass



class CheetahMPCEnv(CheetahBaseEnv):
    r"""Gait Planning environment
    This is an environment for selecting gait parameters for MPC/WBC actuation using the controller of [1].
    
    observation space = [ height                                                      n =  1, si =  0
                          z-axis in world frame expressed in body frame (R_b.row(2))  n =  3, si =  1
                          joint angles,                                               n = 12, si =  4
                          body Linear velocities,                                     n =  3, si = 16
                          body Angular velocities,                                    n =  3, si = 19
                          joint velocities,                                           n = 12, si = 22 ] total 34

    action space      = [ velocity command <continuous>,                              n =  3, si =  0
                          footswing height <continuous, positive>,                    n =  1, si =  3] total 4


    """

    def __init__(self, hmap_generator=None, cfg=None, gui=None, expert_cfg=None, mpc_controller_obj=None, lcm_publisher=None, test_mode=False):

        self.mpc_controller_obj = mpc_controller_obj

        super().__init__(hmap_generator=hmap_generator, cfg=cfg, gui=gui, expert_cfg=expert_cfg, lcm_publisher=lcm_publisher, test_mode=test_mode)

        self.mpc_level_cmd, self.mpc_level_state, self.prev_mpc_level_cmd = MPCLevelCmd(), MPCLevelState(), MPCLevelCmd()
        self.ts = 0
        self.pr = None

        self.kp_mean = np.array([3., 3., 1.])
        self.kd_mean = np.array([2., 1., 0.3])


    def _setup_action_space(self, cfg):
        if cfg.use_22D_actionspace:
            cont_action_dim, disc_action_dim = 14, 0
        else:
            cont_action_dim, disc_action_dim = 4, 0

        if cfg.binary_contact_actions:
            disc_action_dim += cfg.adaptation_steps * 4
        elif cfg.symmetry_contact_actions or cfg.pronk_actions or cfg.bound_actions:
            disc_action_dim += cfg.adaptation_steps
        else:
            if cfg.nonzero_gait_adaptation:
                disc_action_dim += 4
            if not (cfg.fixed_durations or cfg.trot_only or cfg.alt_trot_only) and cfg.nonzero_gait_adaptation:
                disc_action_dim += 4
        if cfg.frequency_adaptation:
            disc_action_dim += 1

        if cfg.use_continuous_actions_only:
            cont_action_dim += disc_action_dim
            disc_action_dim = 0

        if cfg.use_mpc_force_redisuals:
            cont_action_dim += 12

        action_dim = cont_action_dim + disc_action_dim

        return action_dim, cont_action_dim, disc_action_dim

    def _setup_observation_space(self, cfg, cont_action_dim, disc_action_dim):
        if cfg.binary_contact_actions:
            if cfg.observe_contact_history_scalar:
                ob_dim = cont_action_dim + 8
            else:
                ob_dim = cont_action_dim + cfg.contact_history_len * 4
        elif cfg.symmetry_contact_actions or cfg.pronk_actions or cfg.bound_actions:
            ob_dim = cont_action_dim + cfg.contact_history_len
        elif cfg.use_multihot_obs:
            ob_dim=cont_action_dim + 50
        else:
            ob_dim = cont_action_dim + 9
        if cfg.only_observe_body_state:
            ob_dim += 10
        elif cfg.observe_state:
            ob_dim += 34
        if cfg.observe_mpc_progress:
            ob_dim += 1
        if cfg.observe_gap_state:
            ob_dim += 2*cfg.num_observed_gaps
        if cfg.observe_command_vel:
            ob_dim += 3
        
        ob_mean = np.array([*np.zeros(ob_dim)])

        ob_std = np.array([*np.ones(ob_dim)])

        if cfg.only_observe_body_state:
            ob_mean[0:10] = np.array([0.22,  # average height
                                                     0.0, 0.0, 0.0,  # gravity axis 3
                                                     *np.zeros(6),  # body linear/angular velocity
                                                     ])


            ob_std[0:10] = np.array([0.12,  # average height
                                                    *np.ones(3) * 0.7,  # gravity axes angles
                                                    *np.ones(3) * 2.0,  # body linear velocity
                                                    *np.ones(3) * 4.0,  # body angular velocity
                                                     ])
        elif cfg.observe_state:
            ob_mean[0:34] = np.array([0.22,  # average height
                                                     0.0, 0.0, 0.0,  # gravity axis 3
                                                     *self.gc_init[-self.num_joints:],  # joint positions
                                                     *np.zeros(self.num_joints),
                                                     *np.zeros(6),  # body linear/angular velocity
                                                     ])


            ob_std[0:34] = np.array([0.12,  # average height
                                                    *np.ones(3) * 0.7,  # gravity axes angles
                                                    *np.ones(12),  # joint angles
                                                    *np.ones(12) * 10.0,  # joint velocities
                                                    *np.ones(3) * 2.0,  # body linear velocity
                                                    *np.ones(3) * 4.0,  # body angular velocity
                                                     ])

        return ob_dim, ob_mean, ob_std

    def _initialize_simulator(self, cfg):

        # mpc controller
        if self.mpc_controller_obj is not None:
            self.mpc_controller = self.mpc_controller_obj
        elif cfg.no_mpc_ctrl:
            self.mpc_controller = None
        else:
            #from cheetahgym.controllers.control_container import ControlContainer
            from cheetahgym.controllers.control_container_old import ControlContainer
            self.mpc_controller = ControlContainer(dt=cfg.control_dt, iterationsBetweenMPC=cfg.iterationsBetweenMPC)
            if cfg.simulator_name == "PYBULLET":
                self.mpc_controller.reverse_legs = True

        super()._initialize_simulator(cfg)

    def set_gains(self, kp_joint, kd_joint):
        if self.mpc_controller is not None:
            self.mpc_controller.set_gains(kp_joint, kd_joint)


    def reset(self, terrain_parameters=None):

        self.offsets_cmd, self.durations_cmd, self.footswing_height, self.vel_cmd = np.array([0, 0, 0, 0]), np.array([5, 5, 5, 5]), 0.05, np.array([
            0.0, 0., 0.])
        self.fh_rel_cmd = np.zeros(4)
        self.fp_rel_cmd = np.zeros(8)
        self.offsets_smoothed, self.durations_smoothed = np.array([0, 0, 0, 0]), np.array([5, 5, 5, 5])
        self.vel_rpy_cmd = np.zeros(3)
        self.contact_table = np.ones((self.cfg.adaptation_steps, 4))
        self.symmetry_table_history = np.zeros(self.cfg.contact_history_len)
        self.contact_table_history = np.ones((self.cfg.contact_history_len, 4))
        
        self.mpc_progress = 0
        self.mpc_rollout_len = 10
        self.mpc_table = np.zeros((4, self.mpc_rollout_len))
        self.iterationsBetweenMPC = self.iterationsBetweenMPC  # 13
        self.prev_mpc_level_cmd = MPCLevelCmd()
        self.contact_times = np.zeros(4)
        self.flight_times = np.zeros(4)

        self._set_mpc_table()
        
        self.stance_rate = np.zeros(5)
        self.failure = False

        self.residual_forces = None

        super().reset(terrain_parameters=terrain_parameters)
        return self.ob_scaled_vis

    def _set_mpc_table(self):
        self.mpc_table = np.zeros((4, 10))
        for i in range(len(self.offsets_cmd)):
            start = int(self.offsets_cmd[i])
            end = int(self.offsets_cmd[i] + self.durations_cmd[i])
            if end < self.mpc_rollout_len:
                    self.mpc_table[i, start:end] = 1
            else:
                self.mpc_table[i, start:] = 1
                if self.cfg.use_gait_cycling and self.cfg.use_gait_wrapping_obs:
                    self.mpc_table[i, :(end % self.mpc_rollout_len)] = 1

    def _compute_stance_rate(self):
        unique, counts = np.unique(np.sum(self.mpc_table, axis=0), return_counts=True)
        for i in range(5):
            self.stance_rate[i] += dict(zip(unique, counts)).get(i, 0)


    def update_cmd(self, action):
        if self.lcm_publisher is not None:
            self.lcm_publisher.broadcast_action(action, self.num_steps)

        if self.cfg.clip_mpc_actions:
            action[0:4] = np.clip(action[0:4], -self.cfg.clip_mpc_actions_magnitude, self.cfg.clip_mpc_actions_magnitude)
            if self.cfg.use_22D_actionspace:
                action[4:14] = np.clip(action[4:14], -self.cfg.clip_mpc_actions_magnitude, self.cfg.clip_mpc_actions_magnitude)

        if self.cfg.use_mpc_force_redisuals:
            traj_params = action[:4]
            force_params = action[4:16]
            timing_params = action[16:]
        if self.cfg.use_22D_actionspace:
            traj_params = action[:14]
            timing_params = action[14:]
        else:
            traj_params = action[:4]
            timing_params = action[4:]
       
        if self.cfg.act_with_accel:
            self.vel_cmd = self.vel_cmd + np.array([self.cfg.longitudinal_body_vel_range * traj_params[0], self.cfg.lateral_body_vel_range * traj_params[1], self.cfg.vertical_body_vel_range * traj_params[2]])
            self.vel_rpy_cmd = np.zeros(3)
        else:
            self.vel_cmd = np.array([self.cfg.longitudinal_body_vel_center, 0, 0]) + np.array([self.cfg.longitudinal_body_vel_range * traj_params[0], self.cfg.lateral_body_vel_range * traj_params[1], self.cfg.vertical_body_vel_range * traj_params[2]])
            self.vel_rpy_cmd = np.array([0, 0, traj_params[3] * 0.1])
        if self.cfg.use_22D_actionspace:       
            self.vel_rpy_cmd = np.array([traj_params[3] * 0.1, traj_params[4] * 0.1, traj_params[5] * 0.1])
            self.fp_rel_cmd= traj_params[6:14] * 0.02
        else:
            self.fp_rel_cmd = np.zeros(8)

        self.prev_offsets_cmd, self.prev_durations_cmd = self.offsets_cmd[:], self.durations_cmd[:]

        if self.cfg.use_mpc_force_redisuals:
            self.residual_forces = force_params * np.array([0.1, 0.1, 5.0, 0.1, 0.1, 5.0, 0.1, 0.1, 5.0, 0.1, 0.1, 5.0])

        
        if self.cfg.nonzero_gait_adaptation:
            if self.cfg.binary_contact_actions:
                self.contact_table = timing_params[0:self.cfg.adaptation_steps * 4].reshape(self.cfg.adaptation_steps, 4)
                #print(self.contact_table)
                #input()\
            elif self.cfg.symmetry_contact_actions:
                self.contact_type_table = timing_params[0:self.cfg.adaptation_steps]
                # conversion to contact table
                contact_labels = {0: [1, 1, 1, 1],
                                  1: [1, 0, 0, 1],
                                  2: [0, 1, 1, 0],
                                  3: [1, 0, 1, 0],
                                  4: [0, 1, 0, 1],
                                  5: [1, 1, 0, 0],
                                  6: [0, 0, 1, 1],
                                  7: [0, 0, 0, 0]}
                self.contact_table = np.array([contact_labels[label] for label in self.contact_type_table])
            elif self.cfg.bound_actions:
                self.contact_type_table = timing_params[0:self.cfg.adaptation_steps]
                # conversion to contact table
                contact_labels = {0: [1, 1, 0, 0],
                                  1: [0, 0, 1, 1],
                                  2: [1, 0, 0, 1],
                                  3: [0, 1, 1, 0],
                                  4: [0, 0, 0, 0]}
                self.contact_table = np.array([contact_labels[label] for label in self.contact_type_table])
            elif self.cfg.pronk_actions:
                self.contact_type_table = timing_params[0:self.cfg.adaptation_steps]
                # conversion to contact table
                contact_labels = {0: [1, 1, 1, 1],
                                  1: [0, 0, 0, 0]}
                self.contact_table = np.array([contact_labels[label] for label in self.contact_type_table])
            else:
                if self.cfg.use_continuous_actions_only:
                    self.offsets_cmd = (np.rint(np.array([timing_params[0], timing_params[1], timing_params[2], timing_params[3]]) * 5)) % 10 #(self.offsets_smoothed + np.rint(action[4:8] * 5)) % 10
                elif self.cfg.trot_only:
                    self.offsets_cmd = (np.rint(np.array([timing_params[0], timing_params[1], timing_params[1], timing_params[0]]))) % 10
                elif self.cfg.alt_trot_only:
                    # enforce a semi-trot
                    if(timing_params[1] > timing_params[0]):
                        if(timing_params[1] - timing_params[0] < 4):
                            if abs(timing_params[1] - 5) < abs(timing_params[0] - 5):
                                timing_params[1] = (timing_params[0] + 4) % 10
                            else:
                                timing_params[0] = (timing_params[1] - 4) % 10
                        elif(timing_params[0] + 10 - timing_params[1] < 4):
                            if abs(timing_params[1] - 5) > abs(timing_params[0] - 5):
                                timing_params[1] = (timing_params[0] + 4) % 10
                            else:
                                timing_params[0] = (timing_params[1] - 4) % 10
                    elif(timing_params[1] <= timing_params[0]):
                        if(timing_params[0] - timing_params[1] < 4):
                            if abs(timing_params[0] - 5) < abs(timing_params[1] - 5):
                                timing_params[0] = (timing_params[0] + 4) % 10
                            else:
                                timing_params[1] = (timing_params[0] - 4) % 10
                        elif(timing_params[1] + 10 - timing_params[0] < 4):
                            if abs(timing_params[0] - 5) > abs(timing_params[1] - 5):
                                timing_params[0] = (timing_params[1] + 4) % 10
                            else:
                                timing_params[1] = (timing_params[0] - 4) % 10 
                    
                    self.offsets_cmd = (np.rint(np.array([timing_params[0], timing_params[1], timing_params[1], timing_params[0]]))) % 10
                else:
                    self.offsets_cmd = (np.rint(np.array([timing_params[0], timing_params[1], timing_params[2], timing_params[3]]))) % 10 #(self.offsets_smoothed + np.rint(action[4:8] * 5)) % 10

                if self.cfg.fixed_durations or self.cfg.trot_only or self.cfg.alt_trot_only:
                    if self.cfg.modulate_durations_anyway:
                        self.durations_cmd = (np.array([timing_params[2], timing_params[3], timing_params[3], timing_params[2]]).astype(int)) % 4 + 3
                    else:
                        self.durations_cmd = np.array([5, 5, 5, 5]).astype(int) #np.array([5, 5, 5, 5]).astype(int)
                else:
                    if self.cfg.use_continuous_actions_only:
                        self.durations_cmd = np.rint(np.array([timing_params[4], timing_params[5], timing_params[6], timing_params[7]]) * 5) % 9 + 1 #(self.durations_smoothed + np.rint(action[8:12] * 5)) % 10
                    else:
                        self.durations_cmd = np.rint(np.array([timing_params[4], timing_params[5], timing_params[6], timing_params[7]])) #(self.durations_smoothed + np.rint(action[8:12] * 5)) % 10
                if self.cfg.frequency_adaptation:
                    if self.cfg.fixed_durations or self.cfg.trot_only or self.cfg.alt_trot_only:
                        if self.cfg.use_continuous_actions_only:
                            self.iterationsBetweenMPC = int(np.rint(timing_params[4] * 5) % 10 + 13)
                        else:
                            self.iterationsBetweenMPC = int((timing_params[4]) + 13)
                    else:
                        if self.cfg.use_continuous_actions_only:
                            self.iterationsBetweenMPC = int(np.rint(timing_params[8] * 5) % 10 + 13)
                        else:
                            self.iterationsBetweenMPC = int((timing_params[8]) + 13)
        else:
            if self.cfg.fixed_gait_type=="trotting":
                self.offsets_cmd = (np.array([3, 8, 8, 3]).astype(int) - self.mpc_progress) % 10
                self.durations_cmd = np.array([5, 5, 5, 5]).astype(int)
            elif self.cfg.fixed_gait_type=="pronking":
                self.offsets_cmd = (np.array([0, 0, 0, 0]).astype(int) - self.mpc_progress) % 10
                self.durations_cmd = np.array([5, 5, 5, 5]).astype(int)
            elif self.cfg.fixed_gait_type=="standing":
                self.offsets_cmd = (np.array([0, 0, 0, 0]).astype(int) - self.mpc_progress) % 10
                self.durations_cmd = np.array([10, 10, 10, 10]).astype(int)
            elif self.cfg.fixed_gait_type=="bounding":
                self.offsets_cmd = (np.array([8, 8, 3, 3]).astype(int) - self.mpc_progress) % 10
                self.durations_cmd = np.array([5, 5, 5, 5]).astype(int)
            if self.cfg.frequency_adaptation:
                if self.cfg.use_continuous_actions_only:
                    self.iterationsBetweenMPC = int(np.rint(timing_params[0] * 5) % 10 + 13)
                else:
                    self.iterationsBetweenMPC = int(timing_params[0] + 13)


        self.fh_rel_cmd = np.zeros(4)
        self.footswing_height = 0.06
        

        self.mpc_action = MPCLevelCmd()



    def simulate_step(self):

        self.mean_torque = 0.
        self.mean_pitch, self.mean_roll = 0., 0.
        self.max_roll, self.max_pitch, self.max_yaw = 0., 0., 0.
        self.d_foot_gap_mins = np.ones(4) * self.cfg.foot_clearance_limit

        self.prev_mpc_level_cmd = copy.deepcopy(self.mpc_level_cmd)

        action = self.prev_cmd

        self.offsets_smoothed, self.durations_smoothed = self.offsets_cmd, self.durations_cmd

        self.low_level_ob.from_vec(self.cheat_state)
        self.rot_w_b = inversion(get_rotation_matrix_from_rpy(self.est_state[3:6]))

        if self.cfg.use_old_ctrl:
            wbcsteps = int(self.iterationsBetweenMPC * self.cfg.adaptation_steps)
        elif self.cfg.simulator_name == "HARDWARE":
            if self.cfg.low_level_deploy or self.cfg.wbc_level_deploy:
                wbcsteps = self.mpc_controller.getIterationsToNextUpdate(self.cfg.adaptation_steps)
            else:
                wbcsteps = 1
        else:
            wbcsteps = self.mpc_controller.getIterationsToNextUpdate(self.cfg.adaptation_steps)

        if self.pr is not None:
            import pstats, io
            s = io.StringIO()
            p = pstats.Stats(self.pr, stream=s).sort_stats('tottime')
            p.print_stats(20)
            print(s.getvalue())
        self.ts = time.time()

        self.simulator.reset_logger()

        for j in range(wbcsteps):

            if (self.cfg.control_dt > j * self.cfg.control_dt >= 0):
                action = np.zeros(28)
                action[0:3] = self.vel_cmd
                action[3:6] = self.vel_rpy_cmd
                action[6:14] = self.fp_rel_cmd
                action[14:18] = self.fh_rel_cmd
                action[18] = self.footswing_height
                action[19:23] = self.offsets_smoothed
                action[23:27] = self.durations_smoothed
                action[27] = self.iterationsBetweenMPC

                self.prev_cmd = copy.deepcopy(action)

                # increment contact table history
                self.contact_table_history[0:-self.cfg.adaptation_steps, :] = self.contact_table_history[self.cfg.adaptation_steps:, :]
                self.contact_table_history[-self.cfg.adaptation_steps:, :] = self.contact_table

                if self.cfg.symmetry_contact_actions or self.cfg.pronk_actions or self.cfg.bound_actions:
                    self.symmetry_table_history[0:-self.cfg.adaptation_steps] = self.symmetry_table_history[self.cfg.adaptation_steps:]
                    self.symmetry_table_history[-self.cfg.adaptation_steps:] = self.contact_type_table
                if not self.cfg.use_old_ctrl and not self.cfg.no_mpc_ctrl:
                    self.mpc_controller.set_accept_update() # differentiates new update from regular control loop


            self.mpc_level_cmd.planningHorizon = self.cfg.planning_horizon
            self.mpc_level_cmd.adaptationSteps = self.cfg.adaptation_steps# * self.iterationsBetweenMPC
            self.mpc_level_cmd.adaptationHorizon = self.cfg.adaptation_horizon#self.cfg.adaptation_steps# * self.iterationsBetweenMPC

            self.mpc_level_cmd.from_vec(action)
            if self.cfg.binary_contact_actions or self.cfg.symmetry_contact_actions or self.cfg.pronk_actions or self.cfg.bound_actions:
                self.mpc_level_cmd.mpc_table_update = self.contact_table.flatten()

            self.mpc_level_state.from_vec(self.cheat_state)

            if self.cfg.print_time: high_level_step_time = time.time()

            contact_state = self.mpc_controller.getContactState() # for onboard state estimator during deployment

            if self.cfg.use_old_ctrl:
                self.low_level_ob, self.low_level_cmd = self.simulator.step_state_high_level(self.mpc_level_cmd, self.mpc_level_state, self.low_level_ob, self.rot_w_b, contact_state=contact_state)
            else:
                self.low_level_ob, self.low_level_cmd = self.simulator.step_state_high_level_tabular(self.mpc_level_cmd, self.mpc_level_state, self.low_level_ob, self.rot_w_b, residual_forces=self.residual_forces, contact_state=contact_state)

            # RUN STATE ESTIMATOR
            self.low_level_ob = self.run_state_estimator(self.low_level_ob)

            self.cheat_state = self.low_level_ob.to_vec()
            self.est_state = self.cheat_state[:]
            self.rot_w_b = inversion(get_rotation_matrix_from_rpy(self.est_state[3:6]))

            self.done = self.is_terminal_state()
            if self.done: 
                return

            body_rpy = self.cheat_state[3:6]
            self.mean_pitch += np.abs(body_rpy[1]) / (
                    self.iterationsBetweenMPC * self.cfg.mpc_steps_per_env_step)
            self.mean_roll += np.abs(body_rpy[0]) / (
                    self.iterationsBetweenMPC * self.cfg.mpc_steps_per_env_step)

            self.max_roll = max(self.max_roll, abs(body_rpy[0]))
            self.max_pitch = max(self.max_pitch, abs(body_rpy[1]))
            self.max_yaw = max(self.max_yaw, abs(body_rpy[2]))

            if self.cfg.use_old_ctrl and (self.cfg.simulator_name == "RAISIM" or self.cfg.simulator_name == "PYBULLET"):
                contact_state = self.simulator.get_contact_state()
                command_contact_state = self.mpc_table[:, (j // self.iterationsBetweenMPC)]
                foot_positions = self.simulator.get_foot_positions()

                if j % self.iterationsBetweenMPC == 0:
                    for idx in range(4):
                        foot_pos = foot_positions[idx]
                        if contact_state[idx] == 1:
                            # check linear distance to gap
                            for dist in range(30):
                                for ori in [-1, 1]:
                                    px, py = np.rint(self.heightmap_sensor.convert_abs_pos_to_hmap_pixel(foot_pos[0]+ori*dist * self.heightmap_sensor.hmap_cfg["resolution"], foot_pos[1])).astype(int)
                                    if 0 <= px < self.hmap.shape[0] and 0 <= py < self.hmap.shape[1] and \
                                       self.hmap[px,py] < -0.01:
                                        self.d_foot_gap_mins[idx] = min(dist * self.heightmap_sensor.hmap_cfg["resolution"], self.d_foot_gap_mins[idx])
                                        break

        if self.cfg.use_old_ctrl:
            self.mpc_progress = self.mpc_controller.nmpc.iterationCounter // self.mpc_controller.nmpc.iterationsBetweenMPC % 10
        elif self.simulator.mpc_progress > -1:
            self.mpc_progress = self.simulator.mpc_progress
        else:
            self.mpc_progress = (self.mpc_progress + self.cfg.adaptation_steps) % 10
        

    
    def build_ob_scaled_vis(self, cfg, ob_dim, ob_mean, ob_std, camera_params):

        self.update_ob_noise_process(cfg)
        
        if not cfg.use_vision:
            pass
        elif cfg.use_raw_depth_image:
            self.update_depth_camera_ob()
            camera_image = self.depth_camera_ob
            
        elif cfg.use_grayscale_image:
            self.update_depth_camera_ob()
            self.rgb_camera_ob = self.depth_camera_ob
            # convert to grayscale
            grayscale_ob = 0.299 * self.rgb_camera_ob[:, :, 0] + \
                           0.587 * self.rgb_camera_ob[:, :, 1] + \
                           0.114 * self.rgb_camera_ob[:, :, 2]
            camera_image = grayscale_ob
        else:
            self.heightmap_ob = self.get_heightmap_ob(cfg.im_x_shift,
                                                 cfg.im_y_shift,
                                                 cfg.im_height,
                                                 cfg.im_width,
                                                 cfg.im_x_resolution,
                                                 cfg.im_y_resolution)
            camera_image = self.heightmap_ob

        ob_scaled_vis = self.build_ob_scaled_vis_from_state(self.low_level_ob, camera_image, self.mpc_level_cmd, self.mpc_progress, cfg, ob_dim, ob_mean, ob_std, camera_params)

        return ob_scaled_vis

    def build_ob_scaled_vis_from_state(self, robot_state, camera_image, prev_action, mpc_progress, cfg=None, ob_dim=None, ob_mean=None, ob_std=None, camera_params=None):

        if cfg is None: cfg = self.cfg
        if ob_dim is None: ob_dim = self.ob_dim
        if ob_mean is None: ob_mean = self.ob_mean
        if ob_std is None: ob_std = self.ob_std
        if camera_params is None: camera_params = self.camera_params

        ob_double, ob_scaled = np.zeros(ob_dim), np.zeros(ob_dim)

        est_state = robot_state.to_vec()

        
        # vector state
        state_end = 0
        if cfg.only_observe_body_state:
            
            ob_double[0] = est_state[2] + cfg.height_std * self.height_noise

            self.rot_w_b = inversion(get_rotation_matrix_from_rpy(est_state[3:6]))
            ob_double[1:4] = est_state[3:6] + np.array([cfg.roll_std, cfg.pitch_std, cfg.yaw_std]) * self.rpy_noise # numpy is row-major while Eigen is column-major


            # body (linear and angular) velocities
            self.body_linear_vel = est_state[18:21]
            self.body_angular_vel = est_state[21:24]
            ob_double[4:7] = self.body_linear_vel + cfg.vel_std * self.vel_noise
            ob_double[7:10] = self.body_angular_vel + np.array([cfg.vel_roll_std, cfg.vel_pitch_std, cfg.vel_yaw_std]) * self.vel_rpy_noise
            state_end = 10

        elif cfg.observe_state:
            # body height
            ob_double[0] = est_state[2] + cfg.height_std * self.height_noise

            # body orientation
            self.rot_w_b = inversion(get_rotation_matrix_from_rpy(est_state[3:6]))
            ob_double[1:4] = est_state[3:6] + np.array([cfg.roll_std, cfg.pitch_std, cfg.yaw_std]) * self.rpy_noise  # numpy is row-major while Eigen is column-major

            # joint angles and velocities
            ob_double[4:16] = est_state[6:18] + cfg.joint_pos_std * self.joint_pos_noise
            ob_double[16:28] = est_state[24:36] + cfg.joint_vel_std * self.joint_vel_noise

            # body (linear and angular) velocities
            self.body_linear_vel = est_state[18:21]
            self.body_angular_vel = est_state[21:24]
            ob_double[28:31] = self.body_linear_vel + cfg.vel_std * self.vel_noise
            ob_double[31:34] = self.body_angular_vel + np.array([cfg.vel_roll_std, cfg.vel_pitch_std, cfg.vel_yaw_std]) * self.vel_rpy_noise
            state_end = 34


        # timing parameter
        if cfg.observe_mpc_progress:
            ob_double[state_end] = mpc_progress / 10.
            state_end += 1

        if cfg.observe_command_vel:
            ob_double[state_end:state_end+3] = self.vel_cmd
            state_end += 3

        if cfg.binary_contact_actions:
            if cfg.observe_contact_history_scalar:
                # how long were we in contact or flight?
                self.contact_times = np.zeros(4)
                self.flight_times = np.zeros(4)
                for f in range(4):
                    for i in range(cfg.contact_history_len-1, 0, -1):
                        if self.contact_table_history[i, f] == 1:
                            self.contact_times[f] += 1
                        else:
                            self.flight_times[f] += 1
                        if self.contact_table_history[i, f] != self.contact_table_history[i-1, f]:
                            break
                            
                ob_double[-8:-4] = self.contact_times
                ob_double[-4:] = self.flight_times
            else:
                ob_double[-cfg.contact_history_len * 4:] = self.contact_table_history.flatten()
                ob_double[-cfg.contact_history_len * 4-self.cont_action_dim:-cfg.contact_history_len * 4] = self.action[:self.cont_action_dim]
        elif cfg.symmetry_contact_actions or cfg.pronk_actions or cfg.bound_actions:
            ob_double[-cfg.contact_history_len:] = self.symmetry_table_history
            ob_double[-cfg.contact_history_len - self.cont_action_dim:-cfg.contact_history_len] = self.action[:self.cont_action_dim]
        elif cfg.use_multihot_obs:
            self._set_mpc_table()
            iters_encoding = np.zeros(10)
            iters_encoding[int(self.iterationsBetweenMPC-13)] = 1.
            ob_double[-50:-10] = self.mpc_table.flatten()
            ob_double[-10:] = iters_encoding
            ob_double[-50-self.cont_action_dim:-50] = self.action[0:self.cont_action_dim]

        else:
            ob_double[-9:-5] = prev_action.offsets_smoothed
            ob_double[-5:-1] = prev_action.durations_smoothed
            ob_double[-1] = prev_action.iterationsBetweenMPC - 13
            ob_double[-9-self.cont_action_dim:-9] = self.action[0:self.cont_action_dim]

        # scale observation
        ob_scaled = np.asarray((ob_double - ob_mean) / ob_std, dtype=np.float)

        if np.any(np.isnan(ob_scaled)):
            print("NaN in ob!! step {self.num_steps}")
            #print(ob_scaled)
            ob_scaled[:] = 0
            self.BAD_TERMINATION = True

        # vision state
        if not cfg.use_vision:
            ob_vis = 0
        elif cfg.use_raw_depth_image:
            ob_vis = camera_image
            
        elif cfg.use_grayscale_image:
            self.rgb_camera_ob = camera_image
            # convert to grayscale
            grayscale_ob = 0.299 * self.rgb_camera_ob[:, :, 0] + \
                           0.587 * self.rgb_camera_ob[:, :, 1] + \
                           0.114 * self.rgb_camera_ob[:, :, 2]
            ob_vis = grayscale_ob
        elif cfg.observe_gap_state:
            num_observe_gaps = cfg.num_observed_gaps
            gap_state = self.heightmap_sensor.get_gap_state(self.cheat_state[0:3], num_observe_gaps)
            ob_scaled[state_end:state_end+num_observe_gaps*2:] = gap_state
            ob_vis = 0
        else:
            ob_vis = camera_image

        ob_scaled_vis = {"ob": ob_vis, "state": ob_scaled}

        if self.lcm_publisher is not None:
            self.lcm_publisher.broadcast_policy_input(ob_scaled, self.num_steps)

        return ob_scaled_vis

    def update_reward(self):
        
        self.total_reward = 0.0

        
        self.gap_crossing_reward = 0.0
        if self.cfg.reward_gap_crossing:
            self.gap_crossing_reward = (self.heightmap_sensor.get_num_gaps_before(self.cheat_state[0] - 0.1) - self.heightmap_sensor.get_num_gaps_before(self.prev_x_loc - 0.1)) * self.cfg.gap_crossing_reward_coef
            self.total_reward += self.gap_crossing_reward

        forward_progress = self.cheat_state[0] - self.prev_x_loc
        self.forward_vel_reward = self.cfg.progress_reward_coef * forward_progress * (10 / self.cfg.adaptation_steps)
        self.prev_x_loc = self.cheat_state[0]

        self.yaw_penalty = -self.cfg.yaw_penalty_coef * self.max_yaw

        self.vel_ceiling_penalty = - self.cfg.vel_penalty_coef * max(0, np.linalg.norm(self.cheat_state[18:21]) - self.cfg.vel_ceiling)
    
        body_rpy = self.cheat_state[3:6]
        self.roll_penalty = -self.cfg.roll_penalty_coef * self.max_roll
        self.pitch_penalty = -self.cfg.pitch_penalty_coef * self.max_pitch
    
        self.offset_change_penalty = 0. #-0.01 * np.linalg.norm(np.minimum((10 - np.abs(self.prev_offsets_cmd - self.offsets_cmd)), np.abs(self.prev_offsets_cmd - self.offsets_cmd)) / 5.) ** 2 # penalize change in timings
        self.duration_change_penalty = 0. #-0.01 * np.linalg.norm((self.prev_durations_cmd - self.durations_cmd) / 5.) ** 2 # penalize change in durations
        self.terminal_penalty = 0.
        if self.done:
            self.terminal_penalty -= self.cfg.terminal_penalty_coef

        self.total_reward += self.forward_vel_reward + self.vel_ceiling_penalty + self.roll_penalty + self.pitch_penalty + self.yaw_penalty + self.terminal_penalty

        if self.cfg.penalize_contact_change:
            self.contact_change_penalty = 0
            if self.cfg.binary_contact_actions:
                for f in range(4):
                    clen = 0
                    lastswitch = 0
                    for i in range(self.cfg.contact_history_len - 1, 0, -1):
                        if self.contact_table_history[i, f] != self.contact_table_history[i-1, f]:
                            lastswitch = i
                            break

                    for i in range(lastswitch-1, 0, -1):
                        #print(i)
                        if self.contact_table_history[i, f] == self.contact_table_history[i-1, f]:
                            clen += 1
                        else:
                            if clen < 3:
                                self.contact_change_penalty -= self.cfg.contact_change_penalty_magnitude
                            break
            elif self.cfg.symmetry_contact_actions or self.cfg.pronk_actions or self.cfg.bound_actions:
                for i in range(self.cfg.contact_history_len - 1, 0, -1):
                    if self.symmetry_table_history[i] != self.symmetry_table_history[i-1]:
                        self.contact_change_penalty -= self.cfg.contact_change_penalty_magnitude
            else:
                self.prev_contacts = self.prev_mpc_level_cmd.mpc_table_update[self.prev_mpc_level_cmd.adaptationSteps*4 - 4:self.prev_mpc_level_cmd.adaptationSteps*4]
                
                for f in range(4):
                    if self.prev_contacts[f] != self.mpc_level_cmd.mpc_table_update[f]:
                        self.contact_change_penalty -= self.cfg.contact_change_penalty_magnitude
                for l in range(self.mpc_level_cmd.adaptationSteps - 1):
                    for f in range(4):
                        if self.mpc_level_cmd.mpc_table_update[l*4+f] != self.mpc_level_cmd.mpc_table_update[(l+1)*4+f]:
                            self.contact_change_penalty -= self.cfg.contact_change_penalty_magnitude

            self.total_reward += self.contact_change_penalty

        if self.cfg.penalize_mean_motor_vel:
            mean_motor_vel = np.mean(np.abs(self.simulator.motor_velocities))
            self.mean_motor_vel_penalty = -mean_motor_vel * self.cfg.mean_motor_vel_penalty_magnitude
            self.total_reward += self.mean_motor_vel_penalty

        if self.cfg.reward_phase_duration:
            self.phase_duration_penalty = 0
            if self.cfg.binary_contact_actions:
                self.phase_duration_rew = np.sum(self.cfg.phase_duration_reward_coef * (np.minimum(self.contact_times, 3 * np.ones(4)) + np.minimum(self.flight_times, 3 * np.ones(4))))

            self.total_reward += self.phase_duration_rew

        if self.cfg.penalize_mean_motor_vel:
            self.extra_info["reward/mean_motor_vel_penalty"] = self.mean_motor_vel_penalty

        if self.mpc_controller is not None:
            self.mpc_solve_objective_loss = max(self.cfg.mpc_loss_penalty_coef * self.mpc_controller.mpc_objective_value, self.cfg.mpc_loss_penalty_min)
            self.total_reward += self.mpc_solve_objective_loss

            self.wbc_solve_objective_loss = max(-self.cfg.wbc_loss_penalty_coef * self.mpc_controller.wbc_objective_value, self.cfg.wbc_loss_penalty_min)
            self.total_reward += self.wbc_solve_objective_loss

       
        if isnan(self.total_reward):
            self.total_reward = 0
            self.BAD_TERMINATION = True

        return self.total_reward

    def update_extra_info(self):

        self.extra_info["reward/forward_vel_reward"] = self.forward_vel_reward
        self.extra_info["reward/vel_ceiling_penalty"] = -self.vel_ceiling_penalty
        self.extra_info["reward/roll_penalty"] = -self.roll_penalty
        self.extra_info["reward/pitch_penalty"] = -self.pitch_penalty
        self.extra_info["reward/yaw_penalty"] = -self.yaw_penalty
        self.extra_info["reward/offset_change_penalty"] = -self.offset_change_penalty
        self.extra_info["reward/duration_change_penalty"] = -self.duration_change_penalty
        if self.mpc_controller is not None:
            self.extra_info["reward/mpc_solve_objective_loss"] = self.mpc_solve_objective_loss
            self.extra_info["reward/wbc_solve_objective_loss"] = self.wbc_solve_objective_loss
        if self.cfg.reward_gap_crossing:
            self.extra_info["reward/gap_crossing_reward"] = self.gap_crossing_reward


        self.extra_info["forward_vel"] = self.cheat_state[18]
        self.extra_info["final/forward_distance"] = self.cheat_state[0]
        self.extra_info["vel state estimation error"] = np.linalg.norm(self.cheat_state[18:21] - self.est_state[18:21])
        self.extra_info["vel_rpy state estimation error"] = np.linalg.norm(self.cheat_state[21:24] - self.est_state[21:24])
        self.extra_info["pos state estimation error"] = np.linalg.norm(self.cheat_state[0:3] - self.est_state[0:3])
        self._compute_stance_rate()
        self.extra_info["stance_0"] = self.stance_rate[0] / 10.
        self.extra_info["stance_1"] = self.stance_rate[1] / 10.
        self.extra_info["stance_2"] = self.stance_rate[2] / 10.
        self.extra_info["stance_3"] = self.stance_rate[3] / 10.
        self.extra_info["stance_4"] = self.stance_rate[4] / 10.
        self.extra_info["failure"] = self.done
        
        self.extra_info["gap_crossing_count"] = self.heightmap_sensor.get_num_gaps_before(self.cheat_state[0])
        
        if self.cfg.use_curriculum:
            self.extra_info["min_gap_width"] = self.heightmap_sensor.hmap_cfg["min_gap_width"]
            self.extra_info["max_gap_width"] = self.heightmap_sensor.hmap_cfg["max_gap_width"]
        if self.cfg.penalize_contact_change:
            self.extra_info["reward/contact_change_penalty"] = self.contact_change_penalty
        if self.cfg.reward_phase_duration:
            self.extra_info["reward/phase_duration_rew"] = self.phase_duration_rew

        return self.extra_info

    def is_terminal_state(self):

        self.done = super().is_terminal_state()
        
        body_height = self.cheat_state[2]  # - self.ground_height_est
        if body_height < 0.18:
            if self.test_mode == "DEPLOY": 
                pass
            else: 
                if self.num_steps <= 1: self.BAD_TERMINATION = True
                self.done = True
        body_rpy = self.cheat_state[3:6]
        if np.abs(body_rpy[0]) > 0.7 or np.abs(body_rpy[1]) > 0.7:
            if self.num_steps <= 1: self.BAD_TERMINATION = True
            if self.test_mode == "DEPLOY": 
                pass
            else: 
                self.done = True

        if self.cfg.simulator_name == "DSIM":
            foot_positions = self.simulator.get_foot_positions()
            for fp in foot_positions:
                if self.simulator._illegal_contact(fp[0], fp[1], fp[2]):
                    if self.test_mode == "DEPLOY": 
                        print(f"Foot placed in gap, step {self.num_steps}")
                        pass
                    else: 
                        self.done = True
            if self.nanerror:
                if self.test_mode == "DEPLOY": print(f"Nan error, step {self.num_steps}")
                else: self.done = True

        if self.cfg.terminate_on_bad_step:
            stance_foot_pos = self.simulator.get_foot_positions()
            for f in range(4):
                if stance_foot_pos[f][2] < -0.03: # must have stepped in a gap!
                    if self.test_mode == "DEPLOY":
                        print("foot placement failure: {}".format(stance_foot_pos[f]))
                        pass
                    else:
                        if self.num_steps <= 1: self.BAD_TERMINATION = True
                        self.done = True

        if self.BAD_TERMINATION:
            if self.test_mode == "DEPLOY":
                print("NaN Error!")
                pass
            else: 
                self.done = True

        return self.done

    def close(self):
        self.simulator.close()
        super().close()




# Quick test
if __name__ == '__main__':

    from easyrl.configs.command_line import cfg_from_cmd
    from easyrl.configs import cfg, set_config
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

    import argparse

    set_config('ppo')

    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(cfg.alg, parser)

    cfg.alg.linear_decay_clip_range = False
    cfg.alg.linear_decay_lr = False
    #cfg.alg.simulator_name = "PYBULLET_MESHMODEL"
    cfg.alg.simulator_name = "PYBULLET"

    from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    #cfg.alg.terrain_cfg_file = "./terrain_config/flatworld/params.json"
    cfg.alg.terrain_cfg_file = './terrain_config/long_platformed_10cmgaps/params.json'

    if cfg.alg.terrain_cfg_file is not "None":
        hmap_generator = RandomizedGapGenerator()
        hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    else:
        hmap_generator = FileReader(dataset_size=1000, destination=cfg.alg.dataset_path)
        if cfg.alg.test and cfg.alg.fixed_heightmap_idx != -1:
            hmap_generator.fix_heightmap_idx(cfg.alg.fixed_heightmap_idx)

    
    cfg.alg.adaptation_steps = 1
    cfg.alg.nonzero_gait_adaptation = False
    #cfg.alg.nmpc_adaptive_foot_placements = False
    cfg.alg.fixed_gait_type = "trotting"
    #cfg.alg.fixed_gait_type = "pronking"
    #cfg.alg.observe_corrected_vel = True
    #cfg.alg.nominal_ground_friction = 10.0
    #cfg.alg.fpa_heuristic = True
    env = CheetahMPCEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    env.reset()
    action = np.zeros(19)
    phase_1 = [0, 1, 1, 0, 0, 1, 1, 0]
    phase_2 = [1, 0, 0, 1, 1, 0, 0, 1]
    action[4:12] = phase_1
    #action[12] = 15 # frequency parameter
    for t in range(1800):

        action[0] = -5./3.
        obs, reward, done, info = env.step(action)
