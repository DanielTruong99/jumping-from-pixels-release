"""
Gym environment for mini cheetah gait parameters.

"""

import numpy as np
import time
import os
import cv2
import yaml
import json

from gym import spaces, Env
        

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rpy_from_quaternion
from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_rotation_matrix_from_rpy, inversion

from cheetahgym.envs.cheetah_mpc_env import CheetahMPCEnv
from cheetahgym.data_types.low_level_types import LowLevelCmd

import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass


class CheetahFlatEnv(CheetahMPCEnv):
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
    def __init__(self, hmap_generator=None, cfg=None, gui=None, expert_cfg=None, mpc_controller_obj=None, test_mode=False, lcm_publisher=None):
        
        self.q_targets_cmd = np.zeros(12)
        self.qd_targets_cmd = np.zeros(12)
        self.q_gains = np.ones(12) * cfg.pmtg_kp
        self.qd_gains = np.ones(12) * cfg.pmtg_kd
        self.ff_torque_cmd = np.zeros(12)

        self.action_repeat = 1

        self.cpg_counter = 0
        self.cpg_period = int(0.5 / (self.action_repeat * cfg.control_dt)) # 0.5 second gait period

        cfg.no_mpc_ctrl = True

        super().__init__(hmap_generator=hmap_generator, cfg=cfg, gui=gui, expert_cfg=expert_cfg, test_mode=test_mode, lcm_publisher=lcm_publisher)


    def _setup_action_space(self, cfg):

        cont_action_dim = 6
        disc_action_dim = 0
        action_dim = cont_action_dim + disc_action_dim

        return action_dim, cont_action_dim, disc_action_dim

    def render(self, mode=None):
        '''
        print(
            "q_targets_cmd: {}".format(
                self.q_targets_cmd))
        '''
        pass
        #return self.heightmap_ob.reshape(self.heightmap_ob.shape[0], self.heightmap_ob.shape[1], 1)

    def reset(self, terrain_parameters=None):
        self.cpg_counter = 0
        ret = super().reset(terrain_parameters=terrain_parameters)
        return ret

    def update_cmd(self, action):
        action = np.clip(action, -1, 1)
        action_scale = np.array([0.3, 0.7, 0.7, 0.3, 0.7, 0.7])
        action_shift = np.array([0.0, -0.8, 1.6, 0.0, -0.8, 1.6])
        action = np.multiply(action, action_scale) + action_shift
        #print(action)

        self.q_targets_cmd = np.array([action[0], action[1], action[2],
                                       action[3], action[4], action[5],
                                       action[3], action[4], action[5],
                                       action[0], action[1], action[2]])
        #print('q_targets_cmd', self.q_targets_cmd)
        #self.q_targets_cmd[-12:] = action[0:12] * 0.6 + self.gc_init[-12:]
        #self.qd_targets_cmd[-12:] = action[12:24] * 10
        #self.ff_torque_cmd[-12:] = action[24:36] * 10
        #self.update_heightmap_ob()
    def get_pd_command(self, t):
       raise NotImplementedError

    def get_pd_commandv(self, t):
        # self.low_level_cmd.p_targets[-12:] = self.q_targets_cmd
        self.low_level_cmd.v_targets[-12:] = self.qd_targets_cmd
        self.low_level_cmd.p_gains[-12:] = self.q_gains
        self.low_level_cmd.v_gains[-12:] = self.qd_gains

        return self.low_level_cmd

    def precompute_pd_command_list(self):

        return self.q_targets_cmd.reshape(-1, 1)

    def simulate_step(self):
        if self.cfg.debug_flag: print("Stepping");
        #print(self.num_steps)
        self.get_pd_commandv(0)
        q_targets = self.precompute_pd_command_list()
        self.mean_torque = 0.
        self.mean_pitch, self.mean_roll = 0., 0.
        self.max_roll, self.max_pitch, self.max_yaw = 0., 0., 0.
        self.d_foot_gap_mins = np.ones(4) * self.cfg.foot_clearance_limit

        #print(self.action_repeat, self.cfg.tg_update_dt, self.cfg.simulation_dt)

        loop_count = self.action_repeat * int(self.cfg.tg_update_dt / self.cfg.simulation_dt + 1.e-10)
        interpolation_steps = int(self.cfg.control_dt / self.cfg.tg_update_dt)
        for i in range(0, interpolation_steps):
            self.low_level_cmd.p_targets[-12:] = q_targets[:, i]
            self.low_level_ob = self.simulator.step_state_low_level(self.low_level_cmd, loop_count, interpolation_factor=interpolation_steps)
        #self.low_level_ob = self.simulator.step_state_low_level(self.low_level_cmd, loop_count)
        self.cpg_counter = (self.cpg_counter + 1) % self.cpg_period
        self.cheat_state = self.low_level_ob.to_vec()
        self.est_state = self.low_level_ob.to_vec()
        self.rot_w_b = inversion(get_rotation_matrix_from_rpy(self.est_state[3:6]))
        body_rpy = self.cheat_state[3:6]
        self.mean_pitch += np.abs(body_rpy[1]) / (
                self.iterationsBetweenMPC * self.cfg.mpc_steps_per_env_step)
        self.mean_roll += np.abs(body_rpy[0]) / (
                self.iterationsBetweenMPC * self.cfg.mpc_steps_per_env_step)

        self.max_roll = max(self.max_roll, abs(body_rpy[0]))
        self.max_pitch = max(self.max_pitch, abs(body_rpy[1]))
        self.max_yaw = max(self.max_yaw, abs(body_rpy[2]))

    def build_ob_scaled_vis(self, cfg, ob_dim, ob_mean, ob_std, camera_params):
        #self.gc, self.gv = self.est_state[6:18], self.est_state[24:36]
        ob_double, ob_scaled = np.zeros(ob_dim), np.zeros(ob_dim)

        # body height
        ob_double[0] = self.est_state[2]

        # body orientation
        #self.ob_double[1:4] = self.est_state[3:6]
        #rot = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy(self.est_state[3:6]))
        #self.ob_double[1:4] = rot[:, 2]
        ob_double[1:4] = self.est_state[3:6]

        # joint angles and velocities
        ob_double[4:16] = self.est_state[6:18]
        ob_double[16:28] = self.est_state[24:36]

        # body (linear and angular) velocities
        self.body_linear_vel = self.est_state[18:21] # rotate by body orientation??
        self.body_angular_vel = self.est_state[21:24] # rotate by body orientation??
        ob_double[28:31] = self.body_linear_vel
        ob_double[31:34] = self.body_angular_vel

        # cpg counter

        ob_double[35] = self.cpg_counter / self.cpg_period

        
        # no noise on previous action part of observation
        #self.ob_double[22:28] = self.prev_action

        # displacement from last observation
        #self.ob_double[28:30] = self.est_state[0:2] - self.last_frame_pos

        #self.ob_double[34:38] = (np.array(self.offsets_smoothed) - self.mpc_progress) % 10
        #self.ob_double[38:42] = self.durations_smoothed
        #self.ob_double[42:45] = self.vel_cmd[0:3]
        #self.ob_double[45] = self.footswing_height

        # scale observation
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
            #print(self.rgb_camera_ob.shape)
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
        #self.update_heightmap_ob()
        #self.ob_scaled_vis = {"ob": self.heightmap_ob,
        #                      "state": self.ob_scaled}

        return ob_scaled_vis


    def update_reward(self):
        ''' old reward
        forward_progress = self.cheat_state[0] - self.prev_x_loc
        self.forward_vel_reward = self.cfg.progress_reward_coef * forward_progress
        self.prev_x_loc = self.cheat_state[0]
         
        self.vel_ceiling_penalty = - self.cfg.vel_penalty_coef * max(0, np.linalg.norm(self.cheat_state[18:21]) - self.cfg.vel_ceiling)
        #print("vel: {}".format(np.linalg.norm(self.cheat_state[18:21])))
        
        body_rpy = self.cheat_state[3:6]
        self.roll_penalty = -self.cfg.roll_penalty_coef * self.max_roll
        self.pitch_penalty = -self.cfg.pitch_penalty_coef * self.max_pitch
        self.yaw_penalty = -self.cfg.yaw_penalty_coef * self.max_yaw
       
        self.height_penalty = -self.cfg.height_penalty_coef * max(0, self.cfg.height_floor - self.min_height)

        print(self.height_penalty)

        # torque penalty
        self.torque_penalty = -self.cfg.torque_penalty_coef * np.linalg.norm(self.ff_torque_cmd)**2

        # knee inversion penalty
        self.leg_inversion_penalty = 0.0
        for i in range(4):
            if self.cheat_state[8 + 3 * i] < 0.1:
                if self.cfg.debug_flag: print("leg inversion!") 
                self.leg_inversion_penalty += -self.cfg.leg_inversion_penalty_coef

        self.total_reward = self.forward_vel_reward + self.vel_ceiling_penalty + self.roll_penalty + self.pitch_penalty + self.yaw_penalty + self.torque_penalty + self.leg_inversion_penalty + self.height_penalty
        '''

        #self.torque_reward = -self.cfg.torque_penalty_coef * np.linalg.norm(self.cheetah_sim.cheetah.get_generalized_forces())**2
        #gc, gv = self.cheetah_sim.cheetah.get_states()
        #quat = gc[3:7]
        #rot = get_rotation_matrix_from_quaternion

        #self.roll_penalty = -self.cfg.roll_penalty_coef * self.max_roll
        #self.pitch_penalty = -self.cfg.pitch_penalty_coef * self.max_pitch
        #self.yaw_penalty = -self.cfg.yaw_penalty_coef * self.max_yaw
        self.roll_reward = np.exp(-25 * self.max_roll**2)
        self.pitch_reward = np.exp(-40 * self.max_pitch**2)
        self.height_reward = np.exp(-500 * (0.30 - self.cheat_state[2])**2)

        forward_progress = self.cheat_state[0] - self.prev_x_loc
        self.prev_x_loc = self.cheat_state[0]

        self.forward_vel_reward = self.cfg.progress_reward_coef * np.clip(self.cheat_state[18], -self.cfg.vel_ceiling, self.cfg.vel_ceiling)

        # no-progress penalty
        self.no_progress_penalty = 0.5 if abs(forward_progress) < 0.0003 else 0

        self.total_reward = (self.forward_vel_reward - 
                             self.no_progress_penalty + 
                             self.roll_reward + 
                             self.pitch_reward + 
                             self.height_reward)

        #if self.done:
        #    self.total_reward += self.terminal_reward_coeff



        return self.total_reward


    def update_extra_info(self):
        self.extra_info["forward_vel"] = self.cheat_state[18]
        self.extra_info["reward/forward_vel_reward"] = self.forward_vel_reward
        self.extra_info["reward/no_progress_penalty"] = self.no_progress_penalty
        self.extra_info["reward/roll_reward"] = self.roll_reward
        self.extra_info["reward/pitch_reward"] = self.pitch_reward
        self.extra_info["reward/height_reward"] = self.height_reward
        #self.extra_info["reward/roll_penalty"] = -self.roll_penalty
        #self.extra_info["reward/pitch_penalty"] = -self.pitch_penalty
        #self.extra_info["reward/yaw_penalty"] = -self.yaw_penalty
        self.extra_info["final/forward_distance"] = self.cheat_state[0]
        #self.extra_info["vel state estimation error"] = np.linalg.norm(self.cheat_state[18:21] - self.est_state[18:21])
        #self.extra_info["vel_rpy state estimation error"] = np.linalg.norm(self.cheat_state[21:24] - self.est_state[21:24])
        #self.extra_info["pos state estimation error"] = np.linalg.norm(self.cheat_state[0:3] - self.est_state[0:3])
        #self.extra_info["reward/torque_penalty"] = -self.torque_reward
        #self.extra_info["reward/leg_inversion_penalty"] = -self.leg_inversion_penalty
        self.extra_info["gap_crossing_count"] = self.heightmap_sensor.get_num_gaps_before(self.cheat_state[0])

      
        #print(self.extra_info)

        return self.extra_info


    def is_terminal_state(self):
        ''' old terminal condition
        # Originally: if the contact body is not the foot, the episode is over
        # we can do better than this!
        self.done = False
        #for contact in self.robot.get_contacts():
        #    print(contact.get_local_body_index())
        #    if contact.get_local_body_index() not in self.foot_indices:
        #        self.done = True
        body_height = self.cheat_state[2]# - self.ground_height_est
        #if body_height < 0.12:
        #    if self.env_params["debug_flag"]: print("body height failure: {} < 0.18".format(body_height))
        #    self.done = True
        body_rpy = self.cheat_state[3:6]
        if np.abs(body_rpy[0]) > 0.7 or np.abs(body_rpy[1]) > 0.7:
            if self.env_params["debug_flag"]: print("rpy failure: {} not upright".format(body_rpy))
            self.done = True

        if self.SIM_NAME == "DSIM":
            # check for bad footstep
            stance_foot_pos = self.quadruped_sys.get_stance_foot_pos()[0]
            #if self.num_steps > self.control_steps_per_mpc_step: # if this is left out, the footsteps from the previous episode are checked at the beginning because the state estimator has not run yet...
            for i in range(4): # for each foot
                x_pos = stance_foot_pos[i*3] # kinda hacky for now
                y_pos = stance_foot_pos[i*3+1]
                x_px, y_px = np.rint(self.convert_abs_pos_to_hmap_pixel(x_pos, y_pos)).astype(int)
                if y_px < 0 or y_px >= self.hmap.shape[0] or x_px < 0 or x_px >= self.hmap.shape[1]: # robot has left height map
                    if self.env_params["debug_flag"]: print("Robot left heightmap!")
                    self.done = True
                    break
                elif self.hmap[y_px, x_px] < -0.05: # stance foot is in a gap!
                    if self.env_params["debug_flag"]: print("foot placement failure: {}, {}".format(x_px, y_px))
                    #print("believed loc was: {}, {}".format(stance_foot_pos[i*3], stance_foot_pos[i*3+1]))
                    #self.done = True
            if self.nanerror:
                self.done = True
        
        return self.done

        '''
        # new terminal condition
        '''
        self.done = False
        for contact in self.cheetah_sim.cheetah.get_contacts():
            if contact.get_local_body_index not in self.foot_indices:
                self.done = True
        
        self.done = False
        body_rpy = self.cheat_state[3:6]
        if np.abs(body_rpy[0]) > 0.7 or np.abs(body_rpy[1]) > 0.7:
            if self.cfg.debug_flag: print("rpy failure: {} not upright".format(body_rpy))
            self.done = True

        body_pos = self.cheat_state[0:3]
        if body_pos[2] < 0.08:
            if self.cfg.debug_flag: print("body height failure: {} too low".format(body_pos[2]))
            self.done = True

        if body_pos[2] > 0.5:
            if self.cfg.debug_flag: print("body height failure: {} too high".format(body_pos[2]))
            self.done = True
        '''

        # same condition as MPC
        self.done = False

        #if self.test_mode:
        #    return self.done

        if self.test_mode == "DEPLOY":
            return self.done
        
        # for contact in self.robot.get_contacts():
        #    print(contact.get_local_body_index())
        #    if contact.get_local_body_index() not in self.foot_indices:
        #        self.done = True
        body_height = self.cheat_state[2]  # - self.ground_height_est
        #print(f"body_height: {body_height}")
        if body_height < 0.18:
            if self.cfg.debug_flag: print("body height failure: {} < 0.18".format(body_height))
            self.done = True
            #print("bad height")
        body_rpy = self.cheat_state[3:6]
        if np.abs(body_rpy[0]) > 0.75 or np.abs(body_rpy[1]) > 0.75:
            if self.cfg.debug_flag: print("rpy failure: {} not upright".format(body_rpy))
            self.done = True
            #print("bad rpy")

        self.shank_contacts = self.simulator.get_shank_contact_state()
        if self.cfg.pmtg_gait_type == "TROT":
            contacts_allowed = 0
        else:
            contacts_allowed = 0
        if self.shank_contacts is not None and sum(self.shank_contacts) > contacts_allowed:
            self.done = True
            #print("bad shank contact")

        if self.cfg.terminate_on_bad_step:
            if self.cfg.simulator_name == "DSIM":
                foot_positions = self.simulator.get_foot_positions()
                #print(foot_positions)
                for fp in foot_positions:
                    if self.simulator._illegal_contact(fp[0], fp[1], fp[2]):
                        #print(f"Nan error, step {self.num_steps}")
                        self.done = True
            else:
                stance_foot_pos = self.simulator.get_foot_positions()
                for f in range(4):
                    if stance_foot_pos[f][2] < self.cfg.step_in_gap: # must have stepped in a gap!
                        self.done = True

        
        return self.done


   

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

    env = CheetahFlatEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    env.reset()
    for t in range(2400):
        if t % 800 == 0: env.reset(); print("reset")
        #action = np.random.normal(0.0, 0.01, size=4)
        action = np.zeros(6)
        #print(action)
        obs, reward, done, info = env.step(action)
        #print("reward: {}".format(reward))

    pr.disable()
    pr.print_stats(sort='cumtime')
