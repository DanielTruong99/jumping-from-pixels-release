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

from gym import spaces, Env

from cheetahgym.envs.cheetah_flat_env import CheetahFlatEnv
from cheetahgym.data_types.low_level_types import LowLevelCmd


import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass


def modulo_2pi(x):
    while x > 2*np.pi:
        x = x - 2*np.pi
    return x


class CheetahPMTGEnv(CheetahFlatEnv):
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
    def __init__(self, hmap_generator=None, cfg=None, gui=None, expert_cfg=None, mpc_controller_obj=None, test_mode=False):
        
        self.q_targets_cmd = np.zeros(12)
        self.qd_targets_cmd = np.zeros(12)
        self.q_gains = np.ones(12) * cfg.pmtg_kp
        self.qd_gains = np.ones(12) * cfg.pmtg_kd
        self.ff_torque_cmd = np.zeros(12)

        self.action_repeat = 10
        self.beta = cfg.pmtg_beta
        self.Cs = -0.8086 # hip center
        self.Ck = 1.5708 # knee center

        #self.cpg_counter = 0
        #self.cpg_period = int(0.5 / (self.action_repeat * cfg.control_dt)) # 0.5 second gait period

        cfg.no_mpc_ctrl = True

        super().__init__(hmap_generator=hmap_generator, cfg=cfg, gui=gui, test_mode=test_mode)


    def _setup_action_space(self, cfg):

        cont_action_dim = 16
        disc_action_dim = 0
        action_dim = cont_action_dim + disc_action_dim

        return action_dim, cont_action_dim, disc_action_dim

    def _setup_observation_space(self, cfg, cont_action_dim, disc_action_dim):
        ob_dim = 36
        
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
            self.phases = np.array([0, 0, 0, 0])
        ret = super().reset(terrain_parameters=terrain_parameters)
        self.checkpoint = 0.6
        return ret

    def update_cmd(self, action):
        #input()

        action = np.clip(action, -1, 1)
        t = self.cfg.residual_hk
        a = self.cfg.residual_a
        action_scale = np.array([a, t, t, a, t, t, a, t, t, a, t, t, self.cfg.f_tg_scale, self.cfg.alpha_tg, self.cfg.alphak_tg, 0.0])
        action_shift = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, self.cfg.f_tg_center, self.cfg.alpha_center, self.cfg.alphak_center, 0.0])
        action = np.multiply(action, action_scale) + action_shift
        #print(action)

        self.q_targets_cmd = np.array([action[0], action[1], action[2],
                                       action[3], action[4], action[5],
                                       action[6], action[7], action[8],
                                       action[9], action[10], action[11]])

        ## trajectory generator (https://arxiv.org/pdf/1910.02812.pdf)
        # trajectory parameters
        f_tg = action[12]
        alpha_tg = action[13]
        alphak_tg = action[14]
        theta_tg = action[15]

        #print(f'f_tg: {f_tg}, alpha_tg: {alpha_tg}, alphak_tg: {alphak_tg}, theta_tg: {theta_tg}')
        
        tp = np.zeros(4)
        self.phase = modulo_2pi(self.phase + 2*np.pi*f_tg*self.cfg.control_dt)
        for leg_idx in range(4):
            leg_phase = modulo_2pi(self.phases[leg_idx] + self.phase)
            if 0 < leg_phase < 2*np.pi*self.beta:
                tp[leg_idx] = leg_phase / (2 * self.beta)
            else:
                tp[leg_idx] = 2*np.pi - (2*np.pi - leg_phase) / (2 * (1 - self.beta))

            Ht = self.Cs + alpha_tg * np.cos(tp[leg_idx])
            Kt = self.Ck - alphak_tg * np.sin(tp[leg_idx]) + theta_tg * np.cos(tp[leg_idx])

            self.q_targets_cmd[leg_idx * 3 + 1] += Ht
            self.q_targets_cmd[leg_idx * 3 + 2] += Kt
            
        #print(f'phase: {self.phase}')
        #print(f'tp: {tp}')
        #print(f'q_targets_cmd: {self.q_targets_cmd}')
        #print('q_targets_cmd', self.q_targets_cmd)
        #self.q_targets_cmd[-12:] = action[0:12] * 0.6 + self.gc_init[-12:]
        #self.qd_targets_cmd[-12:] = action[12:24] * 10
        #self.ff_torque_cmd[-12:] = action[24:36] * 10
        #self.update_heightmap_ob()

    def update_observation(self):
        #self.gc, self.gv = self.est_state[6:18], self.est_state[24:36]
        self.ob_double, self.ob_scaled = np.zeros(self.ob_dim), np.zeros(self.ob_dim)

        # body height
        self.ob_double[0] = self.est_state[2]

        # body orientation
        #self.ob_double[1:4] = self.est_state[3:6]
        #rot = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy(self.est_state[3:6]))
        #self.ob_double[1:4] = rot[:, 2]
        self.ob_double[1:4] = self.est_state[3:6]

        # joint angles and velocities
        self.ob_double[4:16] = self.est_state[6:18]
        self.ob_double[24:36] = self.est_state[24:36]

        # body (linear and angular) velocities
        self.body_linear_vel = self.est_state[18:21] # rotate by body orientation??
        self.body_angular_vel = self.est_state[21:24] # rotate by body orientation??
        self.ob_double[16:19] = self.body_linear_vel
        self.ob_double[19:22] = self.body_angular_vel

        # cpg counter
        self.ob_double[22] = np.cos(self.phase)
        self.ob_double[23] = np.sin(self.phase)

        
        # no noise on previous action part of observation
        #self.ob_double[22:28] = self.prev_action

        # displacement from last observation
        #self.ob_double[28:30] = self.est_state[0:2] - self.last_frame_pos

        #self.ob_double[34:38] = (np.array(self.offsets_smoothed) - self.mpc_progress) % 10
        #self.ob_double[38:42] = self.durations_smoothed
        #self.ob_double[42:45] = self.vel_cmd[0:3]
        #self.ob_double[45] = self.footswing_height

        # scale observation
        ob_scaled = np.asarray((self.ob_double - self.ob_mean) / self.ob_std, dtype=np.float)
    
        if not self.cfg.use_vision:
            self.ob_scaled_vis = {"ob": 0, "state": ob_scaled}
        elif self.cfg.use_raw_depth_image:
            depth_camera_ob = self.get_depth_camera_ob(self.camera_params)
            self.ob_scaled_vis = {"ob": depth_camera_ob,
                                    "state": ob_scaled}
        elif self.cfg.use_grayscale_image:
            rgb_camera_ob = self.get_depth_camera_ob(self.camera_params)
            # convert to grayscale
            grayscale_ob = 0.299 * rgb_camera_ob[:, :, 0] + \
                           0.587 * rgb_camera_ob[:, :, 1] + \
                           0.114 * rgb_camera_ob[:, :, 2]
            #print(self.rgb_camera_ob.shape)
            self.ob_scaled_vis = {"ob": grayscale_ob,
                             "state": ob_scaled}
        else:
            heightmap_ob = self.get_heightmap_ob(self.cfg.im_x_shift,
                                                 self.cfg.im_y_shift,
                                                 self.cfg.im_height,
                                                 self.cfg.im_width,
                                                 self.cfg.im_x_resolution,
                                                 self.cfg.im_y_resolution)
            self.ob_scaled_vis = {"ob": heightmap_ob,
                             "state": ob_scaled}

        return self.ob_scaled_vis


    def update_reward(self):

        v_max = self.cfg.vel_ceiling
        if self._nsteps < 30:
           v_max = v_max/2 
        self._nsteps +=1
        self.forward_vel_reward = np.exp(-6*(v_max - self.cheat_state[18])**2)
        current_base_posx = self.cheat_state[0]
        step_dist_reward = np.clip(round(100*(current_base_posx - self.last_base_pos[0]),5), -self.cfg.max_stride, self.cfg.max_stride)

        milestone_reward = 0
        
        if self.heightmap_sensor.get_num_gaps_before(current_base_posx - 0.15) - self.heightmap_sensor.get_num_gaps_before(self.last_base_pos[0] - 0.15) > 0:
            if self.gaps_crossed==0:
                milestone_reward = 10000*(1-self.done)
            else:
                milestone_reward = 1000*(1-self.done)
            self.gaps_crossed +=1
            #print("gap_crossed")
 
        self.last_base_pos = self.cheat_state[:3]

        """
        if current_base_posx > self.checkpoint:
            self.checkpoint +=0.6
            milestone_reward = 50
        """

        if self._nsteps > 110:
            if current_base_posx < 0.45:
               self.done = True
               milestone = -100 #+ 300*self.cheat_state[0]
               #print("poor performance")
        if abs(self.cheat_state[5]) > math.radians(50):
            self.done = True
            #print("robot turned a lot")

        if 0.225 > self.cheat_state[2] or self.cheat_state[2]> 0.65:
            self.done =True
            #print("Robot Fallen")

        desired_height = 0.295
        self.roll_reward = np.exp(-40 * self.max_roll**2)
        self.pitch_reward = np.exp(-40 * self.max_pitch**2)
        self.yaw_reward = np.exp(-42*self.cheat_state[5]**2)
        self.height_reward = 0 #np.exp(-650*(self.cheat_state[2] - desired_height)**2)

        self.total_reward = (2*self.forward_vel_reward + self.height_reward + 1.3*self.yaw_reward + self.roll_reward + self.pitch_reward + 2*step_dist_reward + milestone_reward)
        #print("Step_dist Reward, Milestone Reward", step_dist_reward, milestone_reward)
        #if self.done:
        #    self.total_reward += self.terminal_reward_coeff
        if self.done:
            if self.gaps_crossed >= 7:
                print("gap_crossed : ", self.gaps_crossed)

        return self.total_reward

   
    def update_extra_info(self):
        self.extra_info["forward_vel"] = self.cheat_state[18]
        self.extra_info["reward/forward_vel_reward"] = self.forward_vel_reward
        #self.extra_info["reward/no_progress_penalty"] = self.no_progress_penalty
        #self.extra_info["reward/roll_reward"] = self.roll_reward
        #self.extra_info["reward/pitch_reward"] = self.pitch_reward
        #self.extra_info["reward/height_reward"] = self.height_reward
        self.extra_info["final/forward_distance"] = self.cheat_state[0]

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

    env = CheetahPMTGEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    env.reset()
    for t in range(2400):
        if t % 800 == 0: env.reset(); print("reset")
        #action = np.random.normal(0.0, 0.01, size=4)
        action = np.zeros(16)
        action[12:16] = 0
        #print(action)
        obs, reward, done, info = env.step(action)
        #print("reward: {}".format(reward))

    pr.disable()
    pr.print_stats(sort='cumtime')
