import threading
import time
from copy import deepcopy

import airobot as ar
import message_filters
import numpy as np
import rospy
from airobot.utils.common import to_rot_mat
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import numpy_indexed as npi
import open3d

from cheetahgym.sensors.realsense import Realsense

from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_quaternion_from_rpy

import matplotlib.pyplot as plt


class RealsenseROS(Realsense):
    def __init__(self,
                 depth_min=0.15,
                 depth_max=2.0,
                 grid_size=0.020,
                 height_map_bounds=None,
                 min_height=-0.6
                 ):

         super().__init__(rgb=rgb,
                         img_height=img_height,
                         img_width=im_width,
                         depth_min=depth_min,
                         depth_max=depth_max,
                         grid_size=grid_size,
                         height_map_bounds=height_map_bounds,
                         min_height=min_height)

        # initialize ros interface

        rospy.init_node('camera_sub', anonymous=True)
        
        self.cam_int_mat = None
        self.img_height = None
        self.img_width = None
        self.cam_ext_mat = None  # extrinsic matrix T
        self._depth_topic = '/camera/aligned_depth_to_color/image_raw'
        self._rgb_topic = '/camera/color/image_rect_color'
        self._cam_info_topic = '/camera/color/camera_info'

        self._cv_bridge = CvBridge()
        self._cam_info_lock = threading.RLock()
        self._cam_img_lock = threading.RLock()
        self._rgb_img = None
        self._depth_img = None
        self._cam_info = None
        self._cam_P = None
        self._rgb_img_shape = None
        self._depth_img_shape = None
        rospy.Subscriber(self._cam_info_topic,
                         CameraInfo,
                         self._cam_info_callback)

        self._rgb_sub = message_filters.Subscriber(self._rgb_topic,
                                                   Image)
        self._depth_sub = message_filters.Subscriber(self._depth_topic,
                                                     Image)
        img_subs = [self._rgb_sub, self._depth_sub]
        self._sync = message_filters.ApproximateTimeSynchronizer(img_subs,
                                                                 queue_size=2,
                                                                 slop=0.2)
        self._sync.registerCallback(self._sync_callback)
        start_time = time.time()
        while True:
            if self.cam_int_mat is not None and self._rgb_img is not None:
                break
            time.sleep(0.02)
            if time.time() - start_time > 4:
                raise RuntimeError('Cannot fetch the camera info and images!')

        self._init_pers_mat()

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
                                       np.ones((1, img_pixs.shape[1])))).T
        self._uv_one_in_cam = np.dot(self._uv_one, self.cam_int_mat_inv.T)
        # self._uv_one_in_cam = np.dot(self.cam_int_mat_inv, self._uv_one)

    def _cam_info_callback(self, msg):
        self._cam_info_lock.acquire()
        if self._cam_info is None:
            self._cam_info = msg
        if self.img_height is None:
            self.img_height = int(msg.height)
        if self.img_width is None:
            self.img_width = int(msg.width)
        if self._cam_P is None:
            self._cam_P = np.array(msg.P).reshape((3, 4))
        if self.cam_int_mat is None:
            self.cam_int_mat = self._cam_P[:3, :3]
        self._cam_info_lock.release()

    def _sync_callback(self, color, depth):
        self._cam_img_lock.acquire()
        try:
            bgr_img = self._cv_bridge.imgmsg_to_cv2(color, "bgr8")
            self._rgb_img = bgr_img[:, :, ::-1]
            self._depth_img = self._cv_bridge.imgmsg_to_cv2(depth,
                                                            "passthrough")
            if self._rgb_img_shape is None:
                self._rgb_img_shape = self._rgb_img.shape
            if self._depth_img_shape is None:
                self._depth_img_shape = self._depth_img.shape
        except CvBridgeError as e:
            ar.log_error(e)
        self._cam_img_lock.release()

    def set_cam_ext(self, pos=None, ori=None, cam_ext=None):
        """
        Set the camera extrinsic matrix.
        Args:
            pos (np.ndarray): position of the camera (shape: :math:`[3,]`).
            ori (np.ndarray): orientation.
                It can be rotation matrix (shape: :math:`[3, 3]`)
                quaternion ([x, y, z, w], shape: :math:`[4]`), or
                euler angles ([roll, pitch, yaw], shape: :math:`[3]`).
            cam_ext (np.ndarray): extrinsic matrix (shape: :math:`[4, 4]`).
                If this is provided, pos and ori will be ignored.
        """
        if cam_ext is not None:
            self.cam_ext_mat = cam_ext
        else:
            if pos is None or ori is None:
                raise ValueError('If cam_ext is not provided, '
                                 'both pos and ori need'
                                 'to be provided.')
            ori = to_rot_mat(ori)
            cam_mat = np.eye(4)
            cam_mat[:3, :3] = ori
            cam_mat[:3, 3] = pos.flatten()
            self.cam_ext_mat = cam_mat

    def get_images(self, get_rgb=True, get_depth=True, **kwargs):
        """
        Return rgb/depth images.
        Args:
            get_rgb (bool): return rgb image if True, None otherwise.
            get_depth (bool): return depth image if True, None otherwise.
        Returns:
            2-element tuple containing
            - np.ndarray: rgb image (shape: :math:`[H, W, 3]`).
            - np.ndarray: depth image (shape: :math:`[H, W]`).
        """
        rgb_img = None
        depth_img = None
        self._cam_img_lock.acquire()
        if get_rgb:
            rgb_img = deepcopy(self._rgb_img)
        if get_depth:
            depth_img = deepcopy(self._depth_img)
        self._cam_img_lock.release()
        return rgb_img, depth_img

    def get_pcd(self, in_world=True, filter_depth=True,
                depth_min=None, depth_max=None,
                get_rgb=False, return_homogeneous=False,
                depth_im=None, rgb_im=None):
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
        if depth_im is None:
            rgb_im, depth_im = self.get_images(get_rgb=get_rgb, get_depth=True)
        # pcd in camera from depth
        depth = depth_im.reshape(-1, 1) * self.depth_scale
        rgb = None
        if rgb_im is not None:
            rgb = rgb_im.reshape(-1, 3)
        depth_min = depth_min if depth_min else self.depth_min
        depth_max = depth_max if depth_max else self.depth_max
        if filter_depth:
            valid = depth[:, 0] > depth_min
            valid = np.logical_and(valid,
                                   depth[:, 0] < depth_max)
            depth = depth[valid]
            if rgb is not None:
                rgb = rgb[valid]
            uv_one_in_cam = self._uv_one_in_cam[valid, :]
        else:
            uv_one_in_cam = self._uv_one_in_cam
        pts_in_cam = np.multiply(uv_one_in_cam, depth)
        if not in_world:
            pcd_pts = pts_in_cam
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb
        else:
            print("executed!")
            if self.cam_ext_mat is None:
                raise ValueError('Please call set_cam_ext() first to set up'
                                 ' the camera extrinsic matrix')
            pts_in_cam = np.concatenate((pts_in_cam,
                                         np.ones((pts_in_cam.shape[0], 1))),
                                        axis=1)
            pts_in_world = np.dot(pts_in_cam, self.cam_ext_mat.T)
            if return_homogeneous:
                pcd_pts = pts_in_world
            else:
                pcd_pts = pts_in_world[:, :3]
            pcd_rgb = rgb
            return pcd_pts, pcd_rgb
            

def test_pcd():
    camera = Realsense()
    pts, colors = camera.get_pcd(in_world=False, get_rgb=True)
    vis = open3d.visualization.Visualizer()
    vis.create_window("Point Cloud")

    pcd = open3d.geometry.PointCloud()
    pcd.points = open3d.utility.Vector3dVector(pts)
    pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    vis.add_geometry(pcd)
    stime = time.time()
    while True:
        pts, colors = camera.get_pcd(in_world=False, get_rgb=True)
        # from IPython import embed
        # embed()
        pcd.points = open3d.utility.Vector3dVector(pts)
        pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        vis.update_geometry(pcd)
        vis.poll_events()
        vis.update_renderer()
        print(f'hz:{1 / (time.time() - stime)}')
        stime = time.time()

def test_image():
    camera = Realsense()

    rot_cam_base = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy([0.0, 0.3983, 0.0]))
    #axis_swap_mat = np.array([[0, 0, 1],
    #                          [-1, 0, 0],
    #                          [0, -1, 0]])
    #rot_90_about_y = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy([np.pi/2, 0.0, 0.0]))
    #rot_180_about_z = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy([0.0, 0.0,    np.pi/2]))
    #print(rot_90_about_y, rot_180_about_z)
    #axis_swap_mat = np.dot(rot_180_about_z, rot_90_about_y)
    #'''
    axis_swap_mat = np.array([[0, 0, 1],
                              [-1, 0, 0],
                              [0, -1, 0]])
    #'''
    print(axis_swap_mat)
    total_rot = np.dot(rot_cam_base, axis_swap_mat)
    #rot_cam_base = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy([0.0, 0.0, 0]))
    print(total_rot)
    # transformation matrix from the base to the camera


    trans_b_cover = np.array([[-0.    , -0.3879,  0.9217,  0.2812],
                                   [-1.    ,  0.    ,  0.    , -0.0027],
                                   [-0.    , -0.9217, -0.3879, -0.0101],
                                   [ 0.    ,  0.    ,  0.    ,  1.    ]])

    trans_b_cover[:3, :3] = np.array([[-0.    , -0.3879,  0.9217],
                                            [-1.    ,  0.    ,  0.    ],
                                            [-0.    , -0.9217, -0.3879]])

    #trans_b_cover[:3, :3] = total_rot

    #trans_camera_frame = np.eye(4)
    #trans_camera_frame[:3, :3] = np.array([[ -1, -0.,  0.],
    #                            [ 0.,  -1., -0.],
    #                            [ 0.,  0.,  1.]])

    trans_cover_camera = np.array([[1., 0., 0., -0.0175],
                                        [0., 1., 0., 0.],
                                        [0., 0., 1., -0.0042],
                                        [0., 0., 0., 1.]])
    trans_b_c = np.dot(trans_b_cover, trans_cover_camera) #np.dot(trans_camera_frame, trans_cover_camera))
    trans_c_b = np.linalg.inv(trans_b_c)
    # transformation matrix from the world frame to the body frame at t=0
    trans_w_b0 = np.eye(4)
    trans_w_b0[2, 3] = 0.2763
    trans_w_c0 = np.dot(trans_w_b0, trans_b_c)

    # trans_w_c0 = np.eye(4)
    # trans_w_c0[:3, :3] = np.array([[ 0., -0.,  1.],
    #                                     [ 1.,  0., -0.],
    #                                     [ 0.,  1.,  0.]])


    camera.set_cam_ext(cam_ext=trans_b_c)


    hmap, pts, colors = camera.get_height_map(rgb=True)
    plt.ion()
    plt.imshow(hmap)
    plt.colorbar()
    plt.show()
    #vis = open3d.visualization.Visualizer()
    #vis.create_window("Point Cloud")

    #pcd = open3d.geometry.PointCloud()
    #pcd.points = open3d.utility.Vector3dVector(pts)
    #pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
    #vis.add_geometry(pcd)
    while True:
        hmap, pts, colors = camera.get_height_map(rgb=True)

        # print(np.max(hmap), np.min(hmap))
        if len(pts) < 1:
            continue
        plt.imshow(hmap)
        plt.draw()
        plt.pause(0.001)
        #pcd.points = open3d.utility.Vector3dVector(pts)
        #pcd.colors = open3d.utility.Vector3dVector(colors / 255.0)
        #vis.update_geometry(pcd)
        #vis.poll_events()
        #vis.update_renderer()

if __name__ == '__main__':
    #test_pcd()
    test_image()