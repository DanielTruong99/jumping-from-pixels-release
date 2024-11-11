"""
Gym environment for mini cheetah gait parameters.

"""
import math
import numpy as np
import time
import os
import cv2
import yaml
import json
from collections import deque
from gym import spaces, Env
from numpy import linalg as LA

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rpy_from_quaternion
from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_rotation_matrix_from_rpy, inversion
from cheetahgym.utils.inverse_kinematics import Cheetah_Kinematics
from cheetahgym.envs.cheetah_mpc_env import CheetahMPCEnv
from cheetahgym.envs.cheetah_flat_env import CheetahFlatEnv
from cheetahgym.data_types.low_level_types import LowLevelCmd

import matplotlib

try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
    pass


def modulo_2pi(x):
    while x > 2 * np.pi:
        x = x - 2 * np.pi
    return x


class CheetahCartPMTGEnv(CheetahFlatEnv):
    """Gait Planning environment
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

    def __init__(self, hmap_generator=None, cfg=None, gui=None, expert_cfg=None, mpc_controller_obj=None, test_mode=False):

        self.q_targets_cmd = np.zeros(12)
        self.qd_targets_cmd = np.zeros(12)
        self.q_gains = np.ones(12) * cfg.pmtg_kp
        self.qd_gains = np.ones(12) * cfg.pmtg_kd
        self.ff_torque_cmd = np.zeros(12)
        self.f_tg = cfg.f_tg_center
        self.action_repeat = 1
        self.beta = cfg.pmtg_beta
        self.Cs = cfg.hip_center  # hip center
        self.Ck = 1.5708  # knee center
        self.gaps_crossed = 0
        self.ik = Cheetah_Kinematics()
        self.rf_hist = deque([[[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]]] * 3, maxlen=3)
        self.penalties = 0
        self.foot_residuals_prev = np.zeros(12)
        self.foot_residuals = np.zeros(12)
        # self.cpg_counter = 0
        # self.cpg_period = int(0.5 / (self.action_repeat * cfg.control_dt)) # 0.5 second gait period

        cfg.no_mpc_ctrl = True

        super().__init__(hmap_generator=hmap_generator, cfg=cfg, gui=gui, expert_cfg=expert_cfg, test_mode=test_mode)

    def _setup_action_space(self, cfg):

        cont_action_dim = 13
        disc_action_dim = 0
        action_dim = cont_action_dim + disc_action_dim

        return action_dim, cont_action_dim, disc_action_dim

    def _setup_observation_space(self, cfg, cont_action_dim, disc_action_dim):
        ob_dim = 51

        ob_mean = np.array([*np.zeros(ob_dim)])

        ob_std = np.array([*np.ones(ob_dim)])

        return ob_dim, ob_mean, ob_std

    def reset(self, terrain_parameters=None):
        self.phase = 0
        self._nsteps = 0
        self.gaps_crossed = 0
        if self.cfg.pmtg_gait_type == "TROT":
            self.phases = np.array([0, np.pi, np.pi, 0])
        elif self.cfg.pmtg_gait_type == "PRONK":
            self.phases = np.array([np.pi * 0.8, np.pi * 0.8, 0, 0])
        ret = super().reset(terrain_parameters=terrain_parameters)
        self.checkpoint = 0.6
        return ret

    def precompute_pd_command_list(self):
        self.alphak_tg = 0.1
        self.center_z = -0.29
        foot_pos = []
        len = int(self.cfg.control_dt / self.cfg.tg_update_dt)
        ts = np.linspace(0, 1, len+1)
        ts = ts[1:]
        dts = np.arange(1, len+1)
        interp_phases = 2 * np.pi * self.f_tg * self.cfg.tg_update_dt * dts  # compute phases
        interp_phases = np.where(interp_phases > 2 * np.pi, interp_phases - 2 * np.pi, interp_phases)  # modulo 2pi

        leg_phases = self.phases + np.tile(interp_phases, (4, 1)).T
        leg_phases = np.where(leg_phases > 2 * np.pi, leg_phases - 2 * np.pi, leg_phases)  # modulo 2pi
        self.phases = leg_phases[len-1]

        xts = np.outer((1 - ts), self.foot_residuals_prev[0::3]) + np.outer(ts, self.foot_residuals[0::3]) #+ 0.07*np.cos(leg_phases)
        yts = np.outer((1 - ts), self.foot_residuals_prev[1::3]) + np.outer(ts, self.foot_residuals[1::3])
        k = 2 * (leg_phases - np.pi) / np.pi
        zts = np.where((3 * np.pi / 2 > leg_phases) & (leg_phases >= np.pi),
                       self.alphak_tg * (-2 * k ** 3 + 3 * k ** 2) + self.center_z,
                       np.where((leg_phases >= 3 * np.pi / 2) & (leg_phases < 2 * np.pi),
                                self.alphak_tg * (2 * k ** 3 - 9 * k ** 2 + 12 * k - 4) + self.center_z,
                                self.center_z))
        zts = zts + np.outer((1 - ts), self.foot_residuals_prev[2::3]) + np.outer(ts, self.foot_residuals[2::3])
        zts = np.clip(zts, -0.31, -0.13)
        foot_pos = np.concatenate(([xts[len-1]],[yts[len-1]],[zts[len-1]])).T
        self.rf_hist.append(foot_pos)
        abductions, hips, knees = self.ik.inverseKinematics(xts, zts, yts, 1)
        q_targets = np.concatenate((abductions.T, hips.T + np.pi / 2, knees.T))[[0, 4, 8, 1, 5, 9, 2, 6, 10, 3, 7, 11],:]
        return q_targets


    def update_cmd(self, action):
        # input()

        action = np.clip(action, -1, 1)
        rx = self.cfg.rx
        ry = self.cfg.ry
        rz = self.cfg.rz
        action_scale = np.array([rx, ry, rz, rx, ry, rz, rx, ry, rz, rx, ry, rz, self.cfg.f_tg_scale])
        action_shift = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.cfg.f_tg_center])
        action = np.multiply(action, action_scale) + action_shift
        self.foot_residuals_prev = self.foot_residuals[:]
        self.foot_residuals = np.array([action[0], action[1], action[2],
                                        action[3], action[4], action[5],
                                        action[6], action[7], action[8],
                                        action[9], action[10], action[11]])

        self.f_tg = action[12]

    def build_ob_scaled_vis(self, cfg, ob_dim, ob_mean, ob_std, camera_params):
        # self.gc, self.gv = self.est_state[6:18], self.est_state[24:36]
        ob_double, ob_scaled = np.zeros(ob_dim), np.zeros(ob_dim)
        prev_foot_targets = np.array([self.rf_hist[2]]).flatten()
        phase_info = np.concatenate((np.cos(self.phases), np.sin(self.phases)))
                                            #height               #orientation        #Angular Velocities     #Joint Angles         #Joint Velocities           #TG Phase1               #TG Phase2          #TG_freq
        #self.ob_double = np.concatenate(([self.est_state[2]], self.est_state[3:6], self.est_state[21:24], self.est_state[6:18], self.est_state[24:36], [np.cos(self.phases[0]), np.sin(self.phases[0]), np.cos(self.phases[1]), np.sin(self.phases[1]), np.cos(self.phases[2]), np.sin(self.phases[2]),np.cos(self.phases[3]), np.sin(self.phases[3])], [self.f_tg], prev_foot_targets)) #, self.est_state[18:21]))
        ob_double = np.concatenate((self.est_state[3:6], self.est_state[21:24], self.est_state[6:18], self.est_state[24:36],phase_info, prev_foot_targets, [self.f_tg])) #, self.est_state[18:21]))


        ob_scaled = np.asarray((ob_double - ob_mean) / ob_std, dtype=np.float)

        if not self.cfg.use_vision:
            ob_scaled_vis = {"ob": 0, "state": ob_scaled}
        elif self.cfg.use_raw_depth_image:
            depth_camera_ob = self.get_depth_camera_ob(camera_params)
            ob_scaled_vis = {"ob": depth_camera_ob,
                                  "state": ob_scaled}
        elif self.cfg.use_grayscale_image:
            rgb_camera_ob = self.get_depth_camera_ob(camera_params)
            # convert to grayscale
            grayscale_ob = 0.299 * rgb_camera_ob[:, :, 0] + \
                           0.587 * rgb_camera_ob[:, :, 1] + \
                           0.114 * rgb_camera_ob[:, :, 2]
            # print(self.rgb_camera_ob.shape)
            ob_scaled_vis = {"ob": grayscale_ob,
                                  "state": ob_scaled}
        else:
            heightmap_ob = self.get_heightmap_ob(cfg.im_x_shift,
                                                 cfg.im_y_shift,
                                                 cfg.im_height,
                                                 cfg.im_width,
                                                 cfg.im_x_resolution,
                                                 cfg.im_y_resolution)
            ob_scaled_vis = {"ob": heightmap_ob,
                                  "state": ob_scaled}

        return ob_scaled_vis

    def update_reward(self):

        v_max = self.cfg.vel_ceiling
        if self._nsteps < 25:
            v_max = v_max / 2
        self._nsteps += 1
        #if self.cheat_state[18] < v_max:
        self.forward_vel_reward = np.exp(-10 * (v_max - self.cheat_state[18]) ** 2)
        #else:
        #    self.forward_vel_reward = 1.0

        current_base_posx = self.cheat_state[0]
        step_dist_reward = np.clip(round(100 * (current_base_posx - self.last_base_pos[0]), 5), -self.cfg.max_stride,self.cfg.max_stride)
        self.last_base_pos = self.cheat_state[:3]

        if not self.test_mode:
            if self._nsteps > 100:
                if current_base_posx < 0.3:
                    self.done = True
                    #print("robot didn't move enough")
            if abs(self.cheat_state[5]) > math.radians(55):
                self.done = True
                #print("robot turned a lot")

            if 0.20 > self.cheat_state[2] or self.cheat_state[2] > 0.65:
                self.done = True
                #print("Robot Fallen")

        desired_height = 0.285
        self.roll_reward = np.exp(-40 * self.max_roll ** 2)
        self.pitch_reward = np.exp(-45 * self.max_pitch ** 2)
        self.yaw_reward = np.exp(-41 * self.cheat_state[5] ** 2)
        self.height_reward = np.exp(-650 * (self.cheat_state[2] - desired_height) ** 2)
        second_order_diff_traj = np.array(self.rf_hist[2]) - 2 * np.array(self.rf_hist[1]) + np.array(self.rf_hist[0])
        smoothness_penalty = LA.norm(second_order_diff_traj, 2)
        torque_penalty = 0.008 * LA.norm(self.simulator.forces,1)

        self.total_reward = (1.5*self.height_reward + 1.5 * self.yaw_reward + 1.5*self.roll_reward + 1.5*self.pitch_reward + 0*step_dist_reward + 2*self.forward_vel_reward - 0.9 * abs(self.cheat_state[19]) - 1.0 * smoothness_penalty - 1.0*torque_penalty)

        #self.total_reward = self.height_reward + 1.5 * self.yaw_reward + 1.5*self.roll_reward + self.pitch_reward - 2.5 * abs(self.cheat_state[18]) - 2.5 * abs(self.cheat_state[19]) - 0.8 * smoothness_penalty
        # print("Step_dist Reward, Milestone Reward", step_dist_reward, milestone_reward)

        return self.total_reward

    def update_extra_info(self):
        self.extra_info["forward_vel"] = self.cheat_state[18]
        self.extra_info["reward/forward_vel_reward"] = self.forward_vel_reward
        self.extra_info["final/forward_distance"] = self.cheat_state[0]
        self.extra_info["gap_crossing_count"] = self.heightmap_sensor.get_num_gaps_before(self.cheat_state[0])

        return self.extra_info


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

    from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    cfg.alg.terrain_cfg_file = "./terrain_config/flatworld/params.json"
    cfg.alg.control_dt = 0.02
    cfg.alg.simulation_dt = 0.0002
    if cfg.alg.terrain_cfg_file is not "None":
        hmap_generator = RandomizedGapGenerator()
        hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)

    else:
        hmap_generator = FileReader(dataset_size=1000, destination=cfg.alg.dataset_path)
        if cfg.alg.test and cfg.alg.fixed_heightmap_idx != -1:
            hmap_generator.fix_heightmap_idx(cfg.alg.fixed_heightmap_idx)

    import cProfile

    pr = cProfile.Profile()
    pr.enable()

    env = CheetahCartPMTGEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=True)
    env.reset()
    for t in range(2400):
        if t % 800 == 0 or done: env.reset(); print("reset")
        action = 0.9 * np.random.randn(13) + 0
        # action = np.zeros(13)
        # print(action)
        obs, reward, done, info = env.step(action)
        # print("reward: {}".format(reward))
