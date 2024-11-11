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
from cheetahgym.lcm_types.python.body_pose_type import body_pose_type
from cheetahgym.lcm_types.python.spi_data_lcmt import spi_data_lcmt
from cheetahgym.lcm_types.python.wbc_test_data_lcmt import wbc_test_data_lcmt
from cheetahgym.lcm_types.python.simulator_lcmt import simulator_lcmt
from cheetahgym.lcm_types.python.heightmap_image_lcmt import heightmap_image_lcmt
from cheetahgym.utils.raisim_visualizer import RaisimVisualizer
from cheetahgym.utils.rotation_utils import get_rpy_from_quaternion

class RobotState:
    def __init__(self, use_vslam=False, render=False, render_hmap=False):
        self.use_vslam = use_vslam
        self.render = render
        self.render_hmap = render_hmap
        self.lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=255")

        self.pos = np.zeros(3)
        self.rpy = np.zeros(3)
        self.quat = np.array([1, 0, 0, 0])

        self.hmap_im = np.zeros((15, 72))

        self.pose_lock = threading.Lock()

        self.q = np.zeros(12)
        if self.use_vslam:
            self.state_subscription = self.lc.subscribe("slam_pose", self.slam_state_update_cb)
        else:
            self.state_subscription = self.lc.subscribe("wbc_lcm_data", self.state_update_cb)
        self.joint_subscription = self.lc.subscribe("spi_data", self.spi_update_cb)
        #self.simulator_subscription = self.lc.subscribe("simulator_state", self.simulator_update_cb)
        self.hmap_ob_subscription = self.lc.subscribe("heightmap_ob_slam", self.hmap_ob_cb)
        self.hmap_ob_subscription = self.lc.subscribe("heightmap_ob_realsense", self.hmap_ob_cb)
        self.hmap_ob_subscription = self.lc.subscribe("heightmap_ob_spoof", self.hmap_ob_cb)

        if self.render:
            print("Visualizing Robot!")
            self.visualizer = RaisimVisualizer(initial_coordinates=np.zeros(19), render_hmap=self.render_hmap)
            #self.visualizer.add_heightmap_file(hmap_filename="./terrains/default/test/hmap0.png",
            #                                   scale=500., shift=5000.)
            self.visualization_step = 0
            self.visualizer.set_state(self.pos, self.rpy, self.q)

            '''
            print("Visualizing Spoofed Heightmap!")
            plt.ion()
            hmap_sim = self.visualizer.get_heightmap_ob(0.6, 0.0, 15, 48, 1 / 30., 1 / 30., 1 / 30.)
            print(hmap_sim)
            plt.imshow(hmap_sim)
            plt.colorbar()
            plt.show()
            '''

        if self.render_hmap:
            self.fig = plt.gcf()
            self.fig.show()
            self.fig.canvas.draw()

    def state_update_cb(self, channel, data):
        try:
            msg = wbc_test_data_lcmt.decode(data)
            self.pose_lock.acquire()
            self.pos = msg.body_pos
            self.quat = msg.body_ori
            self.q = np.array(msg.jpos[0:12]).flatten()

            if self.render:
                self.visualization_step += 1
                
                if self.visualization_step % 1 == 0:
                #    print(self.pos, self.rpy, self.q)
                    self.visualizer.set_state(self.pos, self.rpy, self.q)
                #if self.visualization_step % 1000 == 0:  # replace with framerate timer
                    #hmap_sim = self.visualizer.get_heightmap_ob(0.8, 0.0, 15, 48, 1 / 30., 1 / 30., 1 / 30.)
                    #plt.imshow(hmap_sim)
                    #plt.draw()
                    #plt.pause(0.001)

            self.pose_lock.release()

            self.rpy = get_rpy_from_quaternion(self.quat)
        except:
            #print("Failed to decode wbc_test_data_lcmt!!")
            pass


    def hmap_ob_cb(self, channel, data):
        msg = heightmap_image_lcmt.decode(data)
        self.pose_lock.acquire()
        self.hmap_im = np.array(msg.hmap).reshape(15, 72)
        self.pose_lock.release()
        '''
        if self.render_hmap:
            #print("rendering")
            plt.ion()
            plt.imshow(self.hmap_im)
            #plt.draw()
            #plt.pause(0.001)
            self.fig.canvas.draw()
            print("rendered")
        '''

    def slam_state_update_cb(self, channel, data):
        msg = body_pose_type.decode(data)
        self.pose_lock.acquire()
        self.pos = msg.body_pos
        self.quat = msg.body_ori_quat
        self.pose_lock.release()

    def get_pose(self):
        self.pose_lock.acquire()
        pos = deepcopy(self.pos)
        quat = deepcopy(self.quat)
        self.pose_lock.release()
        return pos, quat

    def spi_update_cb(self, channel, data):
        msg = spi_data_lcmt.decode(data)

        self.q = np.array([msg.q_abad[0], msg.q_hip[0], msg.q_knee[0],
                           msg.q_abad[1], msg.q_hip[1], msg.q_knee[1],
                           msg.q_abad[2], msg.q_hip[2], msg.q_knee[2],
                           msg.q_abad[3], msg.q_hip[3], msg.q_knee[3], ])

    '''
    def simulator_update_cb(self, channel, data):
        msg = simulator_lcmt.decode(data)
        self.q = np.array(msg.q).flatten()
        print("simupdate")
    '''

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


def get_cfg():
    from easyrl.configs.command_line import cfg_from_cmd
    from easyrl.configs.ppo_config import ppo_cfg
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
    import argparse

    parser = argparse.ArgumentParser()

    set_mc_cfg_defaults(parser)

    cfg_from_cmd(ppo_cfg, parser)

    print(ppo_cfg.num_envs)

    if 'diff_cfg' not in vars(ppo_cfg).keys():
        ppo_cfg.diff_cfg = {}

    ppo_cfg.linear_decay_clip_range = False
    ppo_cfg.linear_decay_lr = False

    ppo_cfg.episode_steps = 100


    return ppo_cfg

def poll_state():
    rs = RobotState(use_vslam=False, render=False, render_hmap=False)
    #

    print("initialized rs & visualizer")

    import gym
    from gym import register
    from cheetahgym.utils.heightmaps import FileReader

    register(id='CheetahMPCEnv-v0',
         entry_point='cheetahgym.envs.cheetah_mpc_env:CheetahMPCEnv',
         max_episode_steps=200,
         reward_threshold=2500.0,
         kwargs={})

    ppo_cfg = get_cfg()
    hmap_generator = FileReader(dataset_size=1, destination=ppo_cfg.dataset_path)
    if ppo_cfg.fixed_heightmap_idx != -1:
        hmap_generator.fix_heightmap_idx(ppo_cfg.fixed_heightmap_idx)
    dummyenv = gym.make(ppo_cfg.env_name, hmap_generator=hmap_generator, cfg=ppo_cfg, gui=False)
    dummyenv.reset()
    #visualizer.add_heightmap_array(dummyenv.hmap.T, None, resolution=dummyenv.hmap_cfg["resolution"])



    rs.run()
    print("running robot system")
    sim_dt = 0.002

    visualize_hmap = False#ppo_cfg.render_heightmap

    if ppo_cfg.record_video: 
        import raisimpy as raisim
        raisim.OgreVis.get().start_recording_video("/data/video/gp_deploy_vis.mp4")

    try:
        if visualize_hmap:

            
            print("initialized env")

            from matplotlib.animation import FuncAnimation
            plt.figure()
            im = plt.imshow(np.zeros((15, 72)))
            plt.colorbar()
            #plt.ion()
            

            #time.sleep(10) # wait for RaisimOgre to start
            hmap_dt = 0.33
            hmap_interval = int(hmap_dt / sim_dt)
            '''
            def update(i):
                #print(rs.pos, rs.rpy, rs.q)
                #for j in range(hmap_interval):
                #for i in range(10):
                #visualizer.set_state(rs.pos, rs.rpy, rs.q)
                
                #print(i)
                #if i % 100 == 0:
                im.set_array(rs.hmap_im)
                    #return [im]
                #else:
                #    return []
                #print(f"SET IMAGE {i}, min value {np.min(rs.hmap_im)}")
                #return [im]     
            
            ani = FuncAnimation(plt.gcf(), update, frames=range(hmap_interval), interval=int(sim_dt * 1000), blit=False)
            print("show")
            plt.show()
            '''
            
            while True:
                
                # print(np.max(hmap), np.min(hmap))
                plt.imshow(rs.hmap_im)
                plt.draw()
                plt.pause(0.001)

        else:
            visualizer = RaisimVisualizer(initial_coordinates=np.zeros(19), render_hmap=False)
            visualizer.add_heightmap_array(dummyenv.hmap.T, None, resolution=dummyenv.hmap_cfg["resolution"])
            for i in range(10000):
                visualizer.set_state(rs.pos, rs.rpy, rs.q)
                #print("viz")
                time.sleep(0.001)
    except KeyboardInterrupt:

        if ppo_cfg.record_video: 
            raisim.OgreVis.get().stop_recording_video_and_save()



if __name__ == "__main__":
    poll_state()