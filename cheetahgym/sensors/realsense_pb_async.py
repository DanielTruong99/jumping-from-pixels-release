import threading
import time

#import matplotlib.pyplot as plt
import numpy as np
import numpy_indexed as npi
import open3d

from cheetahgym.sensors.realsense import Realsense
from cheetahgym.sensors.realsense_pybullet import RealsensePB


import pybullet as pb

class RealsensePBAsync(RealsensePB):
    def __init__(self, rgb=True,
                 img_height=480,
                 img_width=640,
                 depth_min=0.15,
                 depth_max=4.0,
                 grid_size=0.020,
                 height_map_bounds=None,
                 min_height=-0.6,
                 physicsClientId=0):

        print("INITIALIZE ASYNC PB")

        
        #self.frame_lock = threading.lock()

        super().__init__(rgb=rgb,
                         img_height=img_height,
                         img_width=img_width,
                         depth_min=depth_min,
                         depth_max=depth_max,
                         grid_size=grid_size,
                         height_map_bounds=height_map_bounds,
                         min_height=min_height,
                         physicsClientId=physicsClientId)

        self.first_run = True

    def setup_camera(self, focus_pt=None, dist=3,
                     yaw=0, pitch=0, roll=0,
                     height=None, width=None,
                     aspect = 1.0, fov = 45.0,
                     znear = 0.1, zfar = 2.0):

        super().setup_camera(focus_pt=focus_pt, dist=dist,
                     yaw=yaw, pitch=pitch, roll=roll,
                     height=height, width=width,
                     aspect = aspect, fov = fov,
                     znear = znear, zfar = zfar)

        if self.first_run:
            self.recent_frames = (np.zeros((height, width)), np.zeros((height, width)))
            self.run()
            #print("RUNNING CAMERA THREAD")
            self.first_run = False


    def run(self):
        self.run_thread = threading.Thread(target=self.poll_frame, daemon=True)
        self.run_thread.start()

    def poll_frame(self):
        try:
            while True:
                print("getting async frames")
                self.recent_frames = super().get_frames()

                print("got async frames")
                time.sleep(0.05)
        except KeyboardInterrupt:
            return
    

    def get_frames(self, get_rgb=True, get_depth=True,
                   get_seg=False, **kwargs):
        """
        Return rgb, depth, and segmentation images.
        Args:
            get_rgb (bool): return rgb image if True, None otherwise.
            get_depth (bool): return depth image if True, None otherwise.
            get_seg (bool): return the segmentation mask if True,
                None otherwise.
        Returns:
            2-element tuple (if `get_seg` is False) containing
            - np.ndarray: rgb image (shape: [H, W, 3]).
            - np.ndarray: depth image (shape: [H, W]).
            3-element tuple (if `get_seg` is True) containing
            - np.ndarray: rgb image (shape: [H, W, 3]).
            - np.ndarray: depth image (shape: [H, W]).
            - np.ndarray: segmentation mask image (shape: [H, W]), with
              pixel values corresponding to object id and link id.
              From the PyBullet documentation, the pixel value
              "combines the object unique id and link index as follows:
              value = objectUniqueId + (linkIndex+1)<<24 ...
              for a free floating body without joints/links, the
              segmentation mask is equal to its body unique id,
              since its link index is -1.".
        """
        #print("get async frames")

        return self.recent_frames

    def __del__(self):
        pass


def show_pcd():
    camera = RealsensePB(depth_min=0.15,
                       depth_max=4.0)
    pts, colors = camera.get_pcd(pt_in_world=True)

    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)

    while True:
        pts, colors = camera.get_pcd(pt_in_world=True)

        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        time.sleep(0.1)


def main():


    from easyrl.configs.command_line import cfg_from_cmd
    from easyrl.configs import cfg, set_config
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
    from cheetahgym.envs.cheetah_mpc_env import CheetahMPCEnv

    import argparse

    set_config('ppo')

    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(cfg.alg, parser)

    cfg.alg.linear_decay_clip_range = False
    cfg.alg.linear_decay_lr = False
    cfg.alg.simulator_name = 'PYBULLET'
    cfg.alg.use_raw_depth_image = True

    from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    #cfg.alg.terrain_cfg_file = "./terrain_config/test_terrain/params.json"
    cfg.alg.terrain_cfg_file = "./terrain_config/long_platformed_30cmgaps/params.json"

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

    env = CheetahMPCEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    env.reset()

    camera = env.simulator.realsense_camera
    pts, colors = camera.get_pcd(pt_in_world=False)

    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)

    action = np.zeros(19)
    action[4:8] = np.array([0, 5, 5, 0]) # timings
    action[8:12] = np.array([5, 5, 5, 5]) # durations
    action[12] = 15 # frequency parameter
    for t in range(150):
        if t % 50 == 0: env.reset(); print("reset")
        #env.mpc_controller.nmpc.neuralGait.print_vel_table()
        pts, colors = camera.get_pcd(pt_in_world=False)

        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry()
        vis.poll_events()
        vis.update_renderer()
        vis.run()
        #print("")
        input()
        #action[0] = np.random.random() * 4 - 2
        if t % 3 == 0:
            action[12] = np.random.randint(0, 10)
            action[0] = np.random.random() * 4 - 2
        #action = np.random.normal(0.0, 0.01, size=4)
        #print(action)
        obs, reward, done, info = env.step(action)
        #print("reward: {}".format(reward))

    pr.disable()
    pr.print_stats(sort='cumtime')


'''
    camera = RealsensePB(depth_min=0.15,
                       depth_max=2.0)
    # pitch of -0.49 rad
    d435i_extrinsics = np.linalg.inv(np.array([[-0.47, 0.0, -0.883, 0.30],
                                               [0.0, 1.0, 0.0, 0.0],
                                               [0.883, 0.0, -0.47, 0.03],
                                               [0.0, 0.0, 0.0, 1.0]]))
    # d435i_extrinsics = np.asarray(np.linalg.inv(np.matrix([[-0.0,  -0.47,  -0.883, 0.30  ],
    #                                           [-1.0 ,       0.0, 0.0, 0.0   ],
    #                                           [0.0 , 0.88, -0.47,  0.03  ],
    #                                           [ 0.0,         0.0,    0.0,        1.0   ]])))
    camera.set_extrinsics(d435i_extrinsics)

    hmap, pts = camera.get_height_map(rgb=False)

    plt.ion()
    plt.imshow(hmap)
    plt.colorbar()
    plt.show()
    # vis = open3d.visualization.Visualizer()
    # vis.create_window("Point Cloud")

    # pcd = open3d.geometry.PointCloud()
    # pcd.points = open3d.utility.Vector3dVector(pts)
    # pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    # vis.add_geometry(pcd)

    while True:
        hmap, pts = camera.get_height_map(rgb=False)

        # print(np.max(hmap), np.min(hmap))
        if len(pts) < 1:
            continue
        plt.imshow(hmap)
        plt.draw()
        plt.pause(0.001)
        # pcd.points = open3d.utility.Vector3dVector(pts)
        # pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        # vis.update_geometry(pcd)
        # vis.poll_events()
        # vis.update_renderer()
'''

if __name__ == '__main__':
    main()
