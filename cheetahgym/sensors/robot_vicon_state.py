import threading
import time
from copy import deepcopy

import lcm
import matplotlib
#try:
matplotlib.use("GTK3Agg")
#except Exception:
#    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from cheetahgym.lcm_types.python.vicon_pose_lcmt import vicon_pose_lcmt
from cheetahgym.lcm_types.python.leg_control_data_lcmt import leg_control_data_lcmt
from cheetahgym.lcm_types.python.state_estimator_lcmt import state_estimator_lcmt
from cheetahgym.lcm_types.python.heightmap_image_lcmt import heightmap_image_lcmt
from cheetahgym.utils.raisim_visualizer import RaisimVisualizer
from cheetahgym.utils.rotation_utils import get_rpy_from_quaternion, inversion, get_rotation_matrix_from_quaternion, rot2euler, get_quaternion_from_rpy

import time

class RobotViconState:
    def __init__(self):
        #self.use_vslam = use_vslam
        #self.render = render
        #self.render_hmap = render_hmap
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

        self.pos = np.zeros(3)
        self.rpy = np.zeros(3)
        self.quat = np.array([1, 0, 0, 0])

        self.pos_rel = np.zeros(3)
        self.rpy_rel = np.zeros(3)
        self.quat_rel = np.array([1, 0, 0, 0])

        self.hmap_im = np.zeros((15, 72))

        self.pose_lock = threading.Lock()

        self.q = np.zeros(12)

        self.first_update = True
        self.ts = 0.0

        self.vicon_state_subscription = self.lc.subscribe("cheetah_mrg/cheetah_mrg", self.vicon_state_update_cb)
        self.legdata_state_subscription = self.lc.subscribe("leg_control_data", self._legdata_obs_cb)
        self.se_state_subscription = self.lc.subscribe("state_estimator_data", self._se_obs_cb)

        self.origin_T = np.eye(4)
        self.T_abs = np.eye(4)
    
        '''
        if self.render:
            print("Visualizing Robot!")
            self.visualizer = RaisimVisualizer(initial_coordinates=np.zeros(19), render_hmap=self.render_hmap)
            #self.visualizer.add_heightmap_file(hmap_filename="./terrains/default/test/hmap0.png",
            #                                   scale=500., shift=5000.)
            self.visualization_step = 0
            self.visualizer.set_state(self.pos, self.rpy, self.q)

            
            print("Visualizing Spoofed Heightmap!")
            plt.ion()
            hmap_sim = self.visualizer.get_heightmap_ob(0.6, 0.0, 15, 48, 1 / 30., 1 / 30., 1 / 30.)
            print(hmap_sim)
            plt.imshow(hmap_sim)
            plt.colorbar()
            plt.show()
        

        if self.render_hmap:
            self.fig = plt.gcf()
            self.fig.show()
            self.fig.canvas.draw()

        '''


        self.pos_state_history = []
        self.rot_state_history = []
        self.pos_imu_state_history = []
        self.rot_imu_state_history = []
        self.jpos_state_history = []
        self.jvel_state_history = []


    def vicon_state_update_cb(self, channel, data):

        msg = vicon_pose_lcmt.decode(data)
        self.pose_lock.acquire()
        if self.first_update:
            print("FIRST VICON UPDATE")
            self.origin_pos = np.array(msg.translation)#[msg.translation[1], msg.translation[0], msg.translation[2]]
            #self.origin_pos[2] = -0.0 # nominal origin location when standing
            self.origin_pos[2] = 0.112
            self.origin_ori = [msg.rotation[3], msg.rotation[0], msg.rotation[1], msg.rotation[2]]
            self.origin_T[0:3, 0:3] = get_rotation_matrix_from_quaternion(self.origin_ori)
            self.origin_T[0:3, 3] = self.origin_pos
            self.origin_T_inv = np.linalg.inv(self.origin_T)
            self.first_update = False
        
        self.pos = msg.translation#[msg.translation[1], msg.translation[0], msg.translation[2]]
        self.quat = [msg.rotation[3], msg.rotation[0], msg.rotation[1], msg.rotation[2]]
        self.T_abs[0:3, 0:3] = get_rotation_matrix_from_quaternion(self.quat)
        self.T_abs[0:3, 3] = self.pos
        self.T_rel = np.matmul(self.origin_T_inv, self.T_abs)
        
        # option 1: process everything in the initialization frame?
        #self.pos_rel = self.T_rel[0:3, 3]
        #self.quat_rel = get_quaternion_from_rpy(rot2euler(self.T_rel[0:3, 0:3], axes='xyz'))
       	
        # option 2: process everything in the real frame, but shift the origin location
        self.pos_rel = self.pos - self.origin_pos
        self.quat_rel = self.quat
       	#print(f"vicon pos: {self.pos_rel}")
        self.pose_lock.release()
        #print(f'{1/(time.time() - self.ts)} Hz')
        #self.ts = time.time()
        self.pos_state_history += [(time.time(), self.pos_rel)]
        self.rot_state_history += [(time.time(), self.quat_rel)]

    def _legdata_obs_cb(self, channel, data):
        msg = leg_control_data_lcmt.decode(data)
        #ob = LowLevelState()

        self.joint_pos = np.array(msg.q)
        self.joint_vel = np.array(msg.qd)

        self.joint_pos = np.concatenate((self.joint_pos[3:6], self.joint_pos[0:3], self.joint_pos[9:12], self.joint_pos[6:9]))
        self.joint_vel = np.concatenate((self.joint_vel[3:6], self.joint_vel[0:3], self.joint_vel[9:12], self.joint_vel[6:9]))

        self.jpos_state_history += [(time.time(), self.joint_pos)]
        self.jvel_state_history += [(time.time(), self.joint_vel)]

    def _se_obs_cb(self, channel, data):
        msg = state_estimator_lcmt.decode(data)
        #ob = LowLevelState()

        #self.ob.joint_pos = np.array(msg.q)
        #self.ob.joint_vel = np.array(msg.qd)

        #self.ob.joint_pos = np.concatenate((self.ob.joint_pos[3:6], self.ob.joint_pos[0:3], self.ob.joint_pos[9:12], self.ob.joint_pos[6:9]))
        #self.ob.joint_vel = np.concatenate((self.ob.joint_vel[3:6], self.ob.joint_vel[0:3], self.ob.joint_vel[9:12], self.ob.joint_vel[6:9]))

        '''
        if  (np.abs(np.max(ob.joint_pos)) > 100000 or np.abs(np.max(ob.joint_vel)) > 100000):
            
            print("NUMERICAL ERROR!!")
            ob = self.reset_system(self.initial_coords)
            return ob
        '''
        #if joint_only:
        #    return
        #print("IMU UPDATE", len(self.pos_imu_state_history))
        self.pos_imu = msg.p
        self.quat_imu = get_quaternion_from_rpy(msg.rpy)
        #print("ob update received")
        #self.ts = time.time()

        #print('pos', msg.p)
        self.pos_imu_state_history += [(time.time(), self.pos_imu)]
        self.rot_imu_state_history += [(time.time(), self.quat_imu)]
        #print(len(self.pos_state_history), self.pos_state_history[-1][1])

    def get_pose(self):
        self.pose_lock.acquire()
        pos = deepcopy(self.pos_rel)
        quat = deepcopy(self.quat_rel)
        #print(f"vicon pos: {pos}")
        self.pose_lock.release()
        return pos, quat

    def poll_lcm(self):
        try:
            while True:
                self.lc.handle()
        except KeyboardInterrupt:
            pass

    def run(self):
        self.run_thread = threading.Thread(target=self.poll_lcm, daemon=True)
        self.run_thread.start()
        #self.poll_lcm()

    def poll_forever(self):
        try:
            while True:
                self.lc.handle()
        except KeyboardInterrupt:
            pass





if __name__ == "__main__":
    from cheetahgym.systems.pybullet_system import PyBulletSystem
    from cheetahgym.data_types.low_level_types import LowLevelState
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
    from easyrl.configs.ppo_config import ppo_cfg
    from easyrl.configs.command_line import cfg_from_cmd
    import argparse
    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(ppo_cfg, parser)

    initial_coordinates = np.array(
            [0.0, 0.0, 0.30, 1.0, 0.0, 0.0, 0.0, 0.0, -0.80, 1.6, 0.0, -0.80, 1.6, 0.0, -0.8, 1.6, 0.0,
             -0.8, 1.6, ])

    psys = PyBulletSystem(cfg=ppo_cfg, gui=True, mpc_controller=None, initial_coordinates=initial_coordinates, fix_body=True)

    lls = LowLevelState()

    rvs = RobotViconState()
    rvs.run()

    while True:
        pos, quat = rvs.get_pose()

        lls.body_rpy = get_rpy_from_quaternion(quat)
        lls.body_pos = pos

        #print(rvs.origin_T)
        psys.set_state(lls)
        time.sleep(0.1)
