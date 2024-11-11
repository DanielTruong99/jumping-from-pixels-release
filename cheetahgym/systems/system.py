import time
import numpy as np
from cheetahgym.data_types.low_level_types import LowLevelCmd, LowLevelState
from cheetahgym.data_types.dynamics_parameter_types import DynamicsParameters
from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_rpy, inversion

class System(object):
    def __init__(self, cfg):
        self.cfg = cfg
        self.sim_time = 0.
        self.total_time = 0.
        self.mpc_progress = -1
        self.forces = np.zeros(12)

        if self.cfg is None: # load default config
            from easyrl.configs.command_line import cfg_from_cmd
            from easyrl.configs import cfg, set_config
            from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
            import argparse

            set_config('ppo')

            parser = argparse.ArgumentParser()
            set_mc_cfg_defaults(parser)
            cfg_from_cmd(cfg.alg, parser)

            cfg.alg.observe_corrected_vel = True
            cfg.alg.terrain_cfg_file = "./terrain_config/flatworld/params.json"

            self.cfg = cfg.alg


        self.dp = DynamicsParameters(self.cfg)
        self.ob = LowLevelState()



    def reset_logger(self):
        self.body_heights = [] # body height
        self.body_xpos = [] # body height
        self.body_ypos = [] # body height
        self.body_vels = [] # CoM Velocity
        self.body_linear_vels = []
        self.body_angular_vels = []
        self.body_oris = [] # body orientation
        self.torques = [] # torques
        self.motor_positions = [] # motor velocities
        self.motor_velocities = [] # motor velocities
        self.foot_positions = [] # foot positions

    def get_obs(self):
        return LowLevelState()

    def check_numerical_error(self):
        return False

    def step_state_low_level(self, low_level_cmd, loop_count, lag_steps=0, resetlog=False, contact_state=None):
        pass

    def step_state_high_level(self, mpc_level_cmd, mpc_level_state, low_level_state, rot_w_b, override_forces=None, residual_forces=None, contact_state=[0, 0, 0, 0]):
        
        self.reset_logger()

        if self.mpc_controller is None:
            print("MPC Controller Not Loaded!")
            raise Exception

        llo = self.get_delayed_obs(latency=self.dp.pd_latency_seconds)
        rot_w_b_cur = inversion(get_rotation_matrix_from_rpy(llo.body_rpy))

        low_level_cmd = self.mpc_controller.step_with_params(mpc_level_cmd, mpc_level_state, llo, rot_w_b_cur, override_forces=override_forces, residual_forces=residual_forces)

        loop_count = int(self.cfg.control_dt / self.cfg.simulation_dt + 1.e-10)
        lag_steps = 0
        low_level_ob = self.step_state_low_level(low_level_cmd, loop_count, lag_steps, resetlog=False, contact_state=contact_state)

        return low_level_ob, low_level_cmd#, mpc_level_ob

    def step_state_high_level_tabular(self, mpc_level_cmd_tabular, mpc_level_state, low_level_state, rot_w_b, override_forces=None, residual_forces=None, contact_state=[0, 0, 0, 0]):
        
        if self.mpc_controller is None:
            print("MPC Controller Not Loaded!")
            raise Exception

        start_time = time.time()
        llo = self.get_delayed_obs(latency=self.dp.pd_latency_seconds)
        rot_w_b_cur = inversion(get_rotation_matrix_from_rpy(llo.body_rpy))

        foot_locations = self.get_foot_positions()
        if self.cfg.fpa_heuristic:
            for i in range(4):
                target = self.mpc_controller.nmpc.get_swing_traj_placement(i)
                new_target = self.apply_FPA_heuristic(target)
                mpc_level_cmd_tabular.fp_rel_cmd[i*2] = new_target[0] - target[0]
                mpc_level_cmd_tabular.fp_rel_cmd[i*2+1] = new_target[1] - target[1]
                if target[0] != new_target[0]: print(i, target, new_target)
                self.mpc_controller.nmpc.set_swing_traj_placement(i, [new_target[0], new_target[1], 0])

        low_level_cmd = self.mpc_controller.step_with_mpc_table(mpc_level_cmd_tabular, mpc_level_state, llo, rot_w_b_cur, foot_locations=foot_locations, override_forces=override_forces, residual_forces=residual_forces)
        loop_count = int(self.cfg.control_dt / self.cfg.simulation_dt + 1.e-10)
        lag_steps = 0
        ll_time = time.time()


        low_level_ob = self.step_state_low_level(low_level_cmd, loop_count, lag_steps, resetlog=False, contact_state=contact_state)


        self.sim_time = time.time()-ll_time
        self.total_time = time.time()-start_time
        return low_level_ob, low_level_cmd#, mpc_level_ob

    def step_state_wbc(self, wbc_level_cmd, low_level_state):
        
        self.reset_logger()

        if self.mpc_controller is None:
            print("MPC Controller Not Loaded!")
            raise Exception

        llo = self.get_delayed_obs(latency=self.dp.pd_latency_seconds)
        rot_w_b_cur = inversion(get_rotation_matrix_from_rpy(llo.body_rpy))

        foot_locations = self.get_foot_positions()
        low_level_cmd = self.mpc_controller.step_with_wbc_cmd(wbc_level_cmd, low_level_state, rot_w_b_cur, swap_legs=True)
        loop_count = int(self.cfg.control_dt / self.cfg.simulation_dt + 1.e-10)
        lag_steps = 0
        low_level_ob = self.step_state_low_level(low_level_cmd, loop_count, lag_steps, resetlog=False)

        return low_level_ob, low_level_cmd

    def apply_FPA_heuristic(self, nominal_foot_placement, max_displacement=6, safety_thresh = 2):

        for displacement in range(0, max_displacement):
            for alt_displacement in range(0, displacement+1):

                for (dx, dy) in [(displacement, alt_displacement), (displacement, -alt_displacement), (-displacement, alt_displacement), (-displacement, -alt_displacement),
                               (alt_displacement, displacement), (alt_displacement, -displacement), (-alt_displacement, displacement), (-alt_displacement, -displacement),]:
                # check validity
                    x, y = nominal_foot_placement[0] + dx*0.01, nominal_foot_placement[1] + dy*0.01
                    px, py = int(self.hmap_im.shape[0] / 2. - (self.hmap_body_pos[0] - x) / self.hmap_resolution) , int((y + self.hmap_body_pos[1]) / self.hmap_resolution + self.hmap_im.shape[1] / 2.)
                    if px < 0 or px >= self.hmap_im.shape[0] or py < 0 or py >= self.hmap_im.shape[1]:
                        continue

                    else: 
                        is_safe = True
                        for sx in range(safety_thresh):
                            for sy in range(safety_thresh):
                                if self.hmap_im.shape[0] > px+sx >= 0 and self.hmap_im.shape[1] > py+sy >= 0 and self.hmap_im[px+sx, py+sy] != 0:
                                    is_safe = False
                                if self.hmap_im.shape[0] > px-sx >= 0 and self.hmap_im.shape[1] > py+sy >= 0 and self.hmap_im[px-sx, py+sy] != 0:
                                    is_safe = False
                                if self.hmap_im.shape[0] > px+sx >= 0 and self.hmap_im.shape[1] > py-sy >= 0 and self.hmap_im[px+sx, py-sy] != 0:
                                    is_safe = False
                                if self.hmap_im.shape[0] > px-sx >= 0 and self.hmap_im.shape[1] > py-sy >= 0 and self.hmap_im[px-sx, py-sy] != 0:
                                    is_safe = False
                        if is_safe:
                            if dx > 0: print("FWD"); x += 0.2
                            return (x, y)

        print("No safe foothold available!!")
        return nominal_foot_placement

    def add_heightmap_file(self, hmap_filename, hmap_cfg):
        #raise NotImplementedError
        pass

    def add_heightmap_array(self, hmap_im, body_pos, resolution=0.02):
        #raise NotImplementedError
        pass

    def reset_system(self, coordinates):
        #raise NotImplementedError
        pass

    def get_nominal_dynamics(self):
        #raise NotImplementedError
        pass

    def set_nominal_dynamics(self):
        #raise NotImplementedError
        pass

    def get_foot_positions(self):
        # raise NotImplementedError
        pass

    def get_contact_state(self):
        # raise NotImplementedError
        pass

    def get_shank_contact_state(self):
        pass

    def close_and_plot(self):
        if self.cfg.plot_state:
            self.lls_plotter.show_plot()
            #pass
            
    def close(self):
        pass