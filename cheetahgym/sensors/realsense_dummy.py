import time

#import matplotlib.pyplot as plt
import numpy as np
import numpy_indexed as npi
import open3d

from cheetahgym.sensors.realsense import Realsense

import pybullet as pb

class RealsenseDummy(Realsense):
    def __init__(self, rgb=True,
                 img_height=480,
                 img_width=640,
                 depth_min=0.15,
                 depth_max=4.0,
                 grid_size=0.020,
                 height_map_bounds=None,
                 min_height=-0.6,
                 physicsClientId=0):

        self.physicsClient = physicsClientId

        super().__init__(rgb=rgb,
                         img_height=img_height,
                         img_width=img_width,
                         depth_min=depth_min,
                         depth_max=depth_max,
                         grid_size=grid_size,
                         height_map_bounds=height_map_bounds,
                         min_height=min_height)

    def setup_camera(self, focus_pt=None, dist=3,
                     yaw=0, pitch=0, roll=0,
                     height=None, width=None,
                     aspect = 1.0, fov = 45.0,
                     znear = 0.1, zfar = 2.0):
        """
        Setup the camera view matrix and projection matrix. Must be called
        first before images are renderred.
        Args:
            focus_pt (list): position of the target (focus) point,
                in Cartesian world coordinates.
            dist (float): distance from eye (camera) to the focus point.
            yaw (float): yaw angle in degrees,
                left/right around up-axis (z-axis).
            pitch (float): pitch in degrees, up/down.
            roll (float): roll in degrees around forward vector.
            height (float): height of image. If None, it will use
                the default height from the config file.
            width (float): width of image. If None, it will use
                the default width from the config file.
        """
        #print(f"focus camera at, {focus_pt}")

        if focus_pt is None:
            focus_pt = [0, 0, 0]
        if len(focus_pt) != 3:
            raise ValueError('Length of focus_pt should be 3 ([x, y, z]).')
        vm = pb.computeViewMatrixFromYawPitchRoll(      cameraTargetPosition=focus_pt,
                                                        distance=dist,
                                                        yaw=yaw,
                                                        pitch=pitch,
                                                        roll=roll,
                                                        upAxisIndex=2,
                                                        physicsClientId=self.physicsClient)
        self.view_matrix = np.array(vm).reshape(4, 4)
        self.img_height = height# if height else self.cfgs.CAM.SIM.HEIGHT
        self.img_width = width# if width else self.cfgs.CAM.SIM.WIDTH
        #aspect = self.img_width / float(self.img_height)
        #znear = self.cfgs.CAM.SIM.ZNEAR
        #zfar = self.cfgs.CAM.SIM.ZFAR
        #fov = self.cfgs.CAM.SIM.FOV
        #self.znear, self.zfar, self.aspect, self.fov = znear, zfar, aspect, fov
        self.znear, self.zfar, self.fov = znear, zfar, fov
        self.aspect = self.img_width / float(self.img_height)
        pm = pb.computeProjectionMatrixFOV(  fov=self.fov,
                                             aspect=self.aspect,
                                             nearVal=self.znear,
                                             farVal=self.zfar,
                                             physicsClientId=self.physicsClient)
        self.proj_matrix = np.array(pm).reshape(4, 4)
        rot = np.array([[1, 0, 0, 0],
                        [0, -1, 0, 0],
                        [0, 0, -1, 0],
                        [0, 0, 0, 1]])
        view_matrix_T = self.view_matrix.T
        self.cam_ext_mat = np.dot(np.linalg.inv(view_matrix_T), rot)

        vfov = np.deg2rad(fov)
        tan_half_vfov = np.tan(vfov / 2.0)
        tan_half_hfov = tan_half_vfov * self.img_width / float(self.img_height)
        # focal length in pixel space
        fx = self.img_width / 2.0 / tan_half_hfov
        fy = self.img_height / 2.0 / tan_half_vfov
        self.cam_int_mat = np.array([[fx, 0, self.img_width / 2.0],
                                     [0, fy, self.img_height / 2.0],
                                     [0, 0, 1]])
        self._init_pers_mat()

    def get_intrisics(self, frame):
        '''
        intrinsics = frame.profile.as_video_stream_profile().intrinsics
        fx = intrinsics.fx
        fy = intrinsics.fy
        cx = intrinsics.ppx
        cy = intrinsics.ppy
        intr_mat = np.array([[fx, 0, cx],
                             [0, fy, cy],
                             [0, 0, 1.]])
        '''
        return intr_mat

    def get_pcd2(self,
                 pt_in_world=False):
        pt_color = None
        ret = self.get_frames(get_depth=True,
                              get_rgb=self.stream_rgb)
        depth_image, rgb_image = ret

        if rgb_image is not None:
            if depth_image.shape != rgb_image.shape[:2]:
                raise ValueError(f'The shape of the depth image and rgb image should be same.')
            pt_color = rgb_image.reshape(-1, 3)
        depth_im = depth_image.reshape(-1) * self.depth_scale
        uv_one_in_cam = self.uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth_im)
        if not pt_in_world:
            pcd_pts = pts_in_cam.T
            pcd_rgb = pt_color
        else:
            extrinsics = self.get_extrinsics()
            pts_in_cam = np.concatenate((pts_in_cam,
                                         np.ones((1, pts_in_cam.shape[1]))),
                                        axis=0)
            pts_in_world = np.dot(extrinsics, pts_in_cam)
            pcd_pts = pts_in_world[:3, :].T
            pcd_rgb = pt_color
        return pcd_pts, pcd_rgb

    def get_pcd(self, pt_in_world=True, filter_depth=True,
                depth_min=None, depth_max=None):
        """
        Get the point cloud from the entire depth image
        in the camera frame or in the world frame.
        Args:
            in_world (bool): return point cloud in the world frame, otherwise,
                return point cloud in the camera frame.
            filter_depth (bool): only return the point cloud with depth values
                lying in [depth_min, depth_max].
            depth_min (float): minimum depth value. If None, it will use the
                default minimum depth value defined in the config file.
            depth_max (float): maximum depth value. If None, it will use the
                default maximum depth value defined in the config file.
        Returns:
            2-element tuple containing
            - np.ndarray: point coordinates (shape: :math:`[N, 3]`).
            - np.ndarray: rgb values (shape: :math:`[N, 3]`).
        """
        depth_im, rgb_im = self.get_frames(get_rgb=True, get_depth=True)
        # pcd in camera from depth
        #print(depth_im.shape, rgb_im.shape, self.depth_min, self.depth_max)
        depth = depth_im.reshape(-1)# * self.depth_scale
        rgb = None
        if rgb_im is not None:
            rgb = rgb_im.reshape(-1, 3)
        depth_min = depth_min if depth_min else self.depth_min
        depth_max = depth_max if depth_max else self.depth_max
        if filter_depth:
            valid = depth > depth_min
            valid = np.logical_and(valid,
                                   depth < depth_max)
            depth = depth[valid]
            if rgb is not None:
                rgb = rgb[valid]
            uv_one_in_cam = self._uv_one_in_cam[:, valid]
        else:
            uv_one_in_cam = self._uv_one_in_cam

        #print(depth.shape)
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        #print(pts_in_cam.shape)
        if not pt_in_world:
            pcd_pts = pts_in_cam.T
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb
        else:
            if self.cam_ext_mat is None:
                raise ValueError('Please call set_cam_ext() first to set up'
                                 ' the camera extrinsic matrix')
            pts_in_cam = np.concatenate((pts_in_cam,
                                         np.ones((1, pts_in_cam.shape[1]))),
                                        axis=0)
            pts_in_world = np.dot(self.cam_ext_mat, pts_in_cam)
            pcd_pts = pts_in_world[:3, :].T
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb

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

        img_height_des = self.img_height #480
        img_width_des = self.img_width #640

        if self.view_matrix is None:
            raise ValueError('Please call setup_camera() first!')
        #if pb.opengl_render:
        renderer = pb.ER_BULLET_HARDWARE_OPENGL
        #else:
        #    renderer = pb.ER_TINY_RENDERER
        cam_img_kwargs = {
            'width': img_width_des,#self.img_width,
            'height': img_height_des,#self.img_height,
            'viewMatrix': self.view_matrix.flatten(),
            'projectionMatrix': self.proj_matrix.flatten(),
            'flags': pb.ER_NO_SEGMENTATION_MASK,
            'renderer': renderer,
            'shadow': False,
            'physicsClientId': self.physicsClient
        }
        if get_seg:
            pb_seg_flag = pb.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX
            cam_img_kwargs['flags'] = pb_seg_flag

        cam_img_kwargs.update(kwargs)
        #images = pb.getCameraImage(**cam_img_kwargs)
        rgb = np.zeros((self.img_height, self.img_width))
        depth = np.zeros((self.img_height, self.img_width))
        
        if get_seg:
            seg = np.reshape(images[4], [self.img_height,
                                         self.img_width])
            return depth, rgb, seg
        else:
            return depth, rgb

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
