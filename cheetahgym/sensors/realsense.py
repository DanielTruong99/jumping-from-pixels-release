import time

#import matplotlib.pyplot as plt
import numpy as np
import numpy_indexed as npi
import open3d

import cv2


class Realsense:
    def __init__(self, rgb=True,
                 img_height=480,
                 img_width=640,
                 depth_min=0.15,
                 depth_max=4.0,
                 grid_size=0.020,
                 height_map_bounds=None,
                 min_height=-0.6):
        self.stream_rgb = rgb
        self.img_height = img_height
        self.img_width = img_width
        self.grid_size = grid_size
        self.depth_min = depth_min
        self.depth_max = depth_max
        self.min_height = min_height
        self.depth_scale = 0.001
        
        self.intr_mat = None
        self.intr_mat_inv = None
        img_pixs = np.mgrid[0: self.img_height,
                   0: self.img_width].reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self.uv_one = np.concatenate((img_pixs,
                                      np.ones((1, img_pixs.shape[1]))))
        #self.uv_one_in_cam = np.dot(self.intr_mat_inv, self.uv_one)

        # self.height_map_bounds = np.array([[-0.25, 0.3],  # [x_min, y_min]
        #                                   [0.25, 0.8]])  # [x_max, y_max]
        if height_map_bounds is None:
            self.height_map_bounds = np.array([[0.3, -0.25],  # [x_min, y_min]
                                               [0.8, 0.25]])  # [x_max, y_max]
        else:
            self.height_map_bounds = height_map_bounds
        # self.height_map_bounds = np.array([[-2, -2],  # [x_min, y_min]
        #                                    [2, 2]])  # [x_max, y_max]
        self.hmap_shape = np.ceil(np.diff(self.height_map_bounds,
                                          axis=0) / self.grid_size).astype(int)[0]
        self.extr_mat = np.eye(4)

    def get_intrisics(self, frame):
        raise NotImplementedError
        
        #return intr_mat

    def get_extrinsics(self):
        return self.extr_mat

    def set_extrinsics(self, extr_mat):
        self.extr_mat = extr_mat

    def _init_pers_mat(self):
        """
        Initialize related matrices for projecting
        pixels to points in camera frame.
        """
        self.cam_int_mat_inv = np.linalg.inv(self.cam_int_mat)

        img_pixs = np.mgrid[0: self.img_height,
                            0: self.img_width].reshape(2, -1)
        img_pixs[[0, 1], :] = img_pixs[[1, 0], :]
        self._uv_one = np.concatenate((img_pixs,
                                       np.ones((1, img_pixs.shape[1]))))
        self._uv_one_in_cam = np.dot(self.cam_int_mat_inv, self._uv_one)

    def get_pcd2(self,
                 pt_in_world=False):
        pt_color = None
        depth_image, rgb_image = self.get_frames(depth=True,
                                                 rgb=self.stream_rgb)[:2]
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

    def get_pcd(self, pt_in_world=False, return_homogeneous=False):
        # use built-in realsense function
        raise NotImplementedError

        # return points, pt_color

    def get_height_map(self, pts=None, rgb=False):
        if pts is None:
            pts, color = self.get_pcd(pt_in_world=True)
        else:
            rgb = False
        x = pts[:, 0]
        y = pts[:, 1]

        valid = x > self.height_map_bounds[0, 0]
        valid = np.logical_and(valid, x < self.height_map_bounds[1, 0])
        valid = np.logical_and(valid, y > self.height_map_bounds[0, 1])
        valid = np.logical_and(valid, y < self.height_map_bounds[1, 1])
        valid_pts = pts[valid]
        hmap_im = np.full(self.hmap_shape, self.min_height)

        if len(valid_pts) > 0:
            xy_grid_pts = self.convert_xy_to_grid(valid_pts)
            _, idx = npi.group_by(xy_grid_pts[:, :2]).argmax(xy_grid_pts[:, 2])
            hmap = xy_grid_pts[idx]
            xs = hmap[:, 0].astype(int)
            ys = hmap[:, 1].astype(int)
            hmap_im[xs, ys] = hmap[:, 2]
        if rgb:
            valid_pt_colors = color[valid]
            return hmap_im, valid_pts, valid_pt_colors
        else:
            return hmap_im, valid_pts

    def convert_xy_to_grid(self, pts):
        xy_grid_pts = pts.copy()
        xy_grid_pts[:, :2] = np.floor((xy_grid_pts[:, :2] - self.height_map_bounds[0]) / self.grid_size)
        return xy_grid_pts

    def get_frames(self, depth=True, rgb=False):
        raise NotImplementedError

        #return depth_image, color_image, depth_frame, color_frame

    def downsample_image(self, image, height_des, width_des):
        return cv2.resize(image, (width_des, height_des), interpolation=cv2.INTER_NEAREST)

    def __del__(self):
        return
