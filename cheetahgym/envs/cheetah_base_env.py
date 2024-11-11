"""
Gym environment for mini cheetah gait parameters.

"""

import cv2
import time
import numpy as np

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rpy_from_quaternion
from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_rotation_matrix_from_rpy, inversion
from gym import Env
from gym import spaces

from cheetahgym.sensors.heightmap_sensor import HeightmapSensor

from cheetahgym.estimators.body_state_estimator_enkf import BodyStateEstimatorENKF
from cheetahgym.sensors.accelerometer_pybullet import AccelerometerPB


from cheetahgym.systems.system import System

from cheetahgym.data_types.low_level_types import LowLevelCmd, LowLevelState

from cheetahgym.data_types.camera_parameters import cameraParameters

import cProfile


import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass



class CheetahBaseEnv(Env):

    def __init__(self, hmap_generator=None, cfg=None, gui=None, expert_cfg=None, lcm_publisher=None, test_mode=False):

        if cfg is None:
            print("LOAD CFG")
            from cheetahgym.config.mc_cfg import load_cfg
            cfg = load_cfg()

        self.cfg = cfg
        self.expert_cfg = expert_cfg
        self.iterationsBetweenMPC = self.cfg.iterationsBetweenMPC
        self.test_mode = test_mode
        self.lcm_publisher=lcm_publisher

        # set variables
       
        if gui is None:
            self.visualizable = self.cfg.render
        else:
            self.visualizable = gui
        
        # set up heightmap
        self.hmap = None
        self.heightmap_sensor = HeightmapSensor(self.cfg.terrain_cfg_file)


        self.camera_params = self._load_camera_params(self.cfg)
        if self.expert_cfg is not None:
            self.expert_camera_params = self._load_camera_params(self.expert_cfg)

        self.prev_cmd = None

        self.extra_info = dict()  # {str: float}
        
        self.num_joints = 12

        # this is the nominal configuration of cheetah
        self.gc_init = np.array(
            [0.0, 0.0, 0.30, 1.0, 0.0, 0.0, 0.0, 0.0, -0.80, 1.6, 0.0, -0.80, 1.6, 0.0, -0.8, 1.6, 0.0,
             -0.8, 1.6, ])
        
        # initialize action space and observation space
        
        self.action_dim, self.cont_action_dim, self.disc_action_dim = self._setup_action_space(self.cfg)

        self.num_discrete_actions = self.cfg.num_discrete_actions
        
        self._action_low = -1.0
        self._action_high = 1.0
        self.action_space = spaces.Box(low=self._action_low,
                                       high=self._action_high,
                                       shape=[self.action_dim],
                                       dtype=np.float32)

        self.prev_action = np.zeros(self.action_dim)

        self.ob_dim, self.ob_mean, self.ob_std = self._setup_observation_space(self.cfg, self.cont_action_dim, self.disc_action_dim)
        if self.expert_cfg is not None:
            self.expert_ob_dim, self.expert_ob_mean, self.expert_ob_std = self._setup_observation_space(self.expert_cfg, self.cont_action_dim, self.disc_action_dim)

        self.state_low=-1.0
        self.state_high=1.0

        spaces_dict = {}

        if not self.cfg.use_vision or self.cfg.observe_gap_state:
            spaces_dict = {'ob': spaces.Box(low=self.state_low, high=self.state_high, dtype=np.float32,
                                                                      shape=(1,)),
                                                  'state': spaces.Box(low=self.state_low, high=self.state_high, dtype=np.float32,
                                                                      shape=(self.ob_dim,))}
        elif self.cfg.use_raw_depth_image or self.cfg.use_grayscale_image: # use raw depth image
            spaces_dict = {'ob': spaces.Box(low=-1.0, high=1.0,
                                                                   shape=(self.camera_params.height, self.camera_params.width),
                                                                   dtype=np.float32),
                                                  'state': spaces.Box(low=self.state_low, high=self.state_high, dtype=np.float32,
                                                                      shape=(self.ob_dim,))}
        else: # use cropped heightmap
            spaces_dict = {'ob': spaces.Box(low=-1.0, high=1.0,
                                                                   shape=(self.cfg.im_height, self.cfg.im_width),
                                                                   dtype=np.float32),
                                                  'state': spaces.Box(low=self.state_low, high=self.state_high, dtype=np.float32,
                                                                      shape=(self.ob_dim,))}

        if self.expert_cfg is not None:
            spaces_dict['expert_state'] = spaces.Box(low=self.state_low, high=self.state_high, dtype=np.float32,
                                                                      shape=(self.expert_ob_dim,))
            if self.expert_cfg.use_raw_depth_image or self.expert_cfg.use_grayscale_image: # use raw depth image
                spaces_dict['expert_ob'] = spaces.Box(low=-1.0, high=1.0,
                                                                       shape=(self.expert_camera_params.height, self.expert_camera_params.width),
                                                                       dtype=np.float32)
            else: # use cropped heightmap
                spaces_dict['expert_ob'] = spaces.Box(low=-1.0, high=1.0,
                                                    shape=(self.expert_cfg.im_height, self.expert_cfg.im_width),
                                                    dtype=np.float32)
                                                      
        
        self.observation_space = spaces.Dict(spaces_dict)


        self.est_state = np.zeros(self.ob_dim)
        self.cheat_state = np.zeros(self.ob_dim)
        self.heightmap_ob = None

        self.cam_pose_noise = 0
        self.cam_rpy_noise = 0

        # initialize simulator

        self.mpc_controller = None
        self._initialize_simulator(self.cfg)

        self.low_level_ob = LowLevelState()
        self.low_level_cmd = LowLevelCmd()

        self._initialize_state_estimator(self.cfg)

        # initialize dynamics randomization
        self.nominal_dynamics = self.simulator.get_nominal_dynamics()


        if self.nominal_dynamics is not None:
            self.nominal_dynamics.dynamics_randomization_rate = self.cfg.dynamics_randomization_rate
            self.simulator.set_dynamics(self.nominal_dynamics)

        if self.cfg.render_heightmap:
            try:
                plt.ion()
                plt.imshow(np.random.random((32, 32)) * 4 - 2)
                plt.colorbar()
                plt.show()
            except Exception:
                print("failed to display heightmap")
                pass

        self.num_steps = 0

    '''
        Definitions for action and observation space, supporting hybrid discrete/continuous spaces
    '''

    def load_simulator(self, simulator):
        self.simulator = simulator

    def _setup_action_space(self, cfg):
        ## define the action space

        raise NotImplementedError

        return action_dim, cont_action_dim, disc_action_dim

    def _get_action_space_params(self):
        return self.action_dim, self.cont_action_dim, self.disc_action_dim

    def _setup_observation_space(self, cfg, cont_action_dim, disc_action_dim):
         ## define the action space

        raise NotImplementedError

        return ob_dim, ob_mean, ob_std

    def _get_ob_space_params(self):
        return self.ob_dim, self.ob_mean, self.ob_std

    '''
        Initialization helpers
    '''

    def _load_camera_params(self, cfg):

        return cameraParameters(width=cfg.depth_cam_width, height=cfg.depth_cam_height, 
                                x=cfg.depth_cam_x, y=cfg.depth_cam_y, z=cfg.depth_cam_z, 
                                roll=cfg.depth_cam_roll, pitch=cfg.depth_cam_pitch, yaw=cfg.depth_cam_yaw, 
                                fov=cfg.depth_cam_fov, aspect=cfg.depth_cam_aspect, 
                                nearVal=cfg.depth_cam_nearVal, farVal=cfg.depth_cam_farVal,
                                cam_pose_std=cfg.cam_pose_std, cam_rpy_std=cfg.cam_rpy_std)

    def _initialize_simulator(self, cfg):

        if cfg.simulator_name == "PYBULLET":
            from cheetahgym.systems.pybullet_system import PyBulletSystem
            self.simulator=PyBulletSystem(cfg, gui=self.visualizable, mpc_controller=self.mpc_controller, initial_coordinates=self.gc_init, fix_body=self.cfg.fix_body, lcm_publisher=self.lcm_publisher)
        elif cfg.simulator_name == "PYBULLET_MESHMODEL":
            from cheetahgym.systems.pybullet_system_meshmodel import PyBulletSystemMesh
            self.simulator = PyBulletSystemMesh(cfg, gui=self.visualizable, mpc_controller=self.mpc_controller,
                                                initial_coordinates=self.gc_init, fix_body=self.cfg.fix_body)
        else:
            self.simulator  = System(cfg)
        self.nanerror = False

    '''
        Main methods
    '''

    def _initialize_state_estimator(self, cfg):
        if cfg.state_estimation_mode == "enkf":
            if cfg.simulator_name in ["PYBULLET", "PYBULLET_MESHMODEL"]:
                self.accelerometer = AccelerometerPB(self.simulator.robot, self.simulator.physicsClient, accel_noise=0.0, rpy_noise=0)
            else:
                print("Accelerometer only implemented for PyBullet!")
                raise Exception
            self.estimator = BodyStateEstimatorENKF(initial_pos = self.gc_init[0:3], initial_rpy = get_rpy_from_quaternion(self.gc_init[3:7]), dt=cfg.control_dt)
            #input(self.gc_init[0:3])

    def reset(self, terrain_parameters=None):

        if self.cfg.profile:
            try:
                self.pr.disable()
                import pstats, io
                s = io.StringIO()
                p = pstats.Stats(self.pr, stream=s).sort_stats('cumulative')
                p.print_stats(10)
                print(s.getvalue())
            except Exception:
                print("profiling failed")
                pass
            self.pr = cProfile.Profile()
            self.pr.enable()

        if terrain_parameters: 
            self.heightmap_sensor.set_terrain_parameters_from_dict(terrain_parameters)

        if self.cfg.print_time: reset_time = time.time()

      
        self.hmap = self.heightmap_sensor.load_new_heightmap()
        if self.lcm_publisher is not None:
            self.lcm_publisher.broadcast_gt_terrain_map_lcm(self.heightmap_sensor)
        self.num_steps = 0
        self.done = False
        self.BAD_TERMINATION = False

        if self.cfg.command_conditioned:
            self._randomize_command()

        # randomize initial coords
        reset_coords = np.copy(self.gc_init)

        yaw_rand = self.cfg.max_initial_yaw * np.random.rand() - self.cfg.max_initial_yaw/2.

        reset_coords[3:7]  = get_quaternion_from_rpy(np.array([0., 0., yaw_rand]))
        if not self.heightmap_sensor.hmap_cfg["linear"]: 
            reset_coords[0:2] = [(np.random.random() - 0.5) * self.heightmap_sensor.hmap_cfg["hmap_length_px"] * self.heightmap_sensor.hmap_cfg["resolution"] / 2., 
                                 (np.random.random() - 0.5) * self.heightmap_sensor.hmap_cfg["hmap_width_px"] * self.heightmap_sensor.hmap_cfg["resolution"] / 2.] 

            reset_px = self.heightmap_sensor.convert_abs_pos_to_hmap_pixel(reset_coords[0], reset_coords[1])
            self.hmap[int(reset_px[1] - 0.3/self.heightmap_sensor.hmap_cfg["resolution"]):int(reset_px[1] + 0.3/self.heightmap_sensor.hmap_cfg["resolution"]), 
                      int(reset_px[0] - 0.3/self.heightmap_sensor.hmap_cfg["resolution"]):int(reset_px[0] + 0.3/self.heightmap_sensor.hmap_cfg["resolution"]) ] = 0.0


        ob = self.simulator.reset_system(reset_coords) # randomize yaw angle
        
        self.simulator.add_heightmap_array(self.hmap.T, body_pos = self.heightmap_sensor.body_pos, resolution=self.heightmap_sensor.hmap_cfg["resolution"])
        self.prev_action = np.zeros(self.action_dim)
        self.action = np.zeros(self.action_dim)


        self.cheat_state = ob.to_vec()
        self.est_state = ob.to_vec()
        self.rot_w_b = inversion(get_rotation_matrix_from_rpy(self.est_state[3:6]))
        self.prev_x_loc = self.cheat_state[0]
        self.last_base_pos = self.cheat_state[:3]

        self.update_observation()

        if self.cfg.debug_flag: print("RANDOMIZE DYNAMICS: {}".format(self.cfg.randomize_dynamics))
        if self.cfg.randomize_dynamics:
            self.randomize_dynamics(extreme=self.cfg.randomize_dynamics_ood)
        if self.cfg.randomize_contact_dynamics:
            self.randomize_contact_dynamics()

        self.spacing_counter = 1
        
        if self.cfg.print_time: print(f'reset time: {time.time() - reset_time}')

        return self.ob_scaled_vis

    def render(self, mode=None):
        print(
            "ob pos: {}, ob_rpy: {}, ".format(
                self.cheat_state[0:3], self.cheat_state[3:6]))
        self.update_heightmap_ob()
        self.update_depth_camera_ob()
        if self.cfg.render_heightmap:
            if self.cfg.use_raw_depth_image:
                if self.depth_camera_ob is not None:
                    print(self.depth_camera_ob.shape)
                    plt.imshow(self.depth_camera_ob)
                    plt.draw()
                    plt.pause(0.01)
                else:
                    print(self.depth_camera_ob)
            else:
                plt.imshow(self.heightmap_ob)
                plt.draw()
                plt.pause(0.01)


    def update_cmd(self, action):
       raise NotImplementedError


    def simulate_step(self):
        raise NotImplementedError

    def randomize_dynamics(self, extreme=False):
        if self.cfg.debug_flag: print("randomizing dynamics!") 
        self.simulator.set_dynamics(self.nominal_dynamics.apply_randomization())

    def randomize_contact_dynamics(self):
        if self.cfg.debug_flag: print("randomizing dynamics!") 
        self.simulator.set_dynamics(self.nominal_dynamics.apply_contact_randomization())

    def set_gains(self):
        pass

    def update_ob_noise_process(self, cfg):
        '''
        A function for generating red noise for different state variables.
        Call once and then apply the updated noise in `build_ob_scaled_vis`. 
        For example, this line from `cheetah_mpc_env` adds the noise to the robot height :
            ob_double[0] = self.est_state[2] + cfg.height_std * self.height_noise
        '''
        if self.test_mode == "DEPLOY":
            self.height_noise = 0
            self.rpy_noise = np.zeros(3)
            self.vel_noise = np.zeros(3)
            self.vel_rpy_noise = np.zeros(3)
            self.joint_pos_noise = np.zeros(12)
            self.joint_vel_noise = np.zeros(12)
            self.cam_pose_noise = np.zeros(3)
            self.cam_rpy_noise = np.zeros(3)

        if self.num_steps == 0:
            self.height_noise = np.random.randn()
            self.rpy_noise = np.random.randn(3)
            self.vel_noise = np.random.randn(3)
            self.vel_rpy_noise = np.random.randn(3)
            self.joint_pos_noise = np.random.randn(12)
            self.joint_vel_noise = np.random.randn(12)
            self.cam_pose_noise = np.random.randn(3)
            self.cam_rpy_noise = np.random.randn(3)
        else:
            self.height_noise = cfg.ob_noise_autocorrelation * self.height_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn()
            self.rpy_noise = cfg.ob_noise_autocorrelation * self.rpy_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn(3)
            self.vel_noise = cfg.ob_noise_autocorrelation * self.vel_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn(3)
            self.vel_rpy_noise = cfg.ob_noise_autocorrelation * self.vel_rpy_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn(3)
            self.joint_pos_noise = cfg.ob_noise_autocorrelation * self.joint_pos_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn(12)
            self.joint_vel_noise = cfg.ob_noise_autocorrelation * self.joint_vel_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn(12)
            self.cam_pose_noise = cfg.ob_noise_autocorrelation * self.cam_pose_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn()
            self.cam_rpy_noise = cfg.ob_noise_autocorrelation * self.cam_rpy_noise + \
                                        (1-cfg.ob_noise_autocorrelation**2)**0.5 * np.random.randn()                            
    
    def step(self, action):

        self.prev_action = self.action[:]
        self.action = action
        self.update_cmd(action)
        self.total_reward = 0

        if self.cfg.print_time: step_time = time.time()
        self.simulate_step()
        self.simulator.check_numerical_error()
        if self.cfg.print_time: print(f'simulate step time: {time.time() - step_time}')

        # update observation
        if self.cfg.print_time: ob_time = time.time()
        self.update_observation()
        if self.cfg.print_time: print(f'ob time: {time.time() - ob_time}')

        if self.cfg.visualize_traj:
            self.simulator.vis_foot_traj()

        # update if episode is over or not
        self.is_terminal_state()

        if self.done and self.BAD_TERMINATION == True:
            print("HARD RESET DUE TO IMMEDIATE TERMINATION")
            self.simulator.reset_system_hard(self.gc_init)

        # update reward
        if not self.cfg.command_conditioned:
            self.update_reward()

        # update extra info
        self.update_extra_info()

        self.num_steps += 1

        return self.ob_scaled_vis, self.total_reward, self.done, self.extra_info


    '''
        Observation generation
    '''

    def update_observation(self):

        self.ob_scaled_vis = self.build_ob_scaled_vis(self.cfg, self.ob_dim, self.ob_mean, self.ob_std, self.camera_params)

        if self.expert_cfg is not None:
            ob_scaled_vis_expert = self.build_ob_scaled_vis(self.expert_cfg, self.expert_ob_dim, self.expert_ob_mean, self.expert_ob_std, self.expert_camera_params)

            self.ob_scaled_vis["expert_ob"] = ob_scaled_vis_expert["ob"]
            self.ob_scaled_vis["expert_state"] = ob_scaled_vis_expert["state"]

        return self.ob_scaled_vis

    def run_state_estimator(self, ob):
        if self.cfg.state_estimation_mode == "enkf":
            self.accelerometer.update()
            accel_meas, rot_meas = self.accelerometer.get_measurement()
            contactEstimate = self.simulator.get_contact_state() # replace with contact estimator!
            #contactEstimate = self.mpc_controller.getContactState()
            self.estimator.update(accel_meas, rot_meas, self.cfg.control_dt, omega=ob.body_angular_vel, q=ob.joint_pos, qd=ob.joint_vel, contactEstimate=contactEstimate)
            pos_est, vel_est, rpy_est = self.estimator.get_state()
            ob.body_pos = pos_est
            #ob.body_pos[0:2] = pos_est[0:2] # for some reason, the position makes the controller fail!
            ob.body_linear_vel = vel_est
            ob.body_rpy = rpy_est
        return ob

    def build_ob_scaled_vis(self, cfg, ob_dim, ob_mean, ob_std, camera_params):

        ob_double, ob_scaled = np.zeros(ob_dim), np.zeros(ob_dim)

        raise NotImplementedError


    def update_heightmap_ob(self):
        self.heightmap_ob = self.get_heightmap_ob(self.cfg.im_x_shift,
                                                  self.cfg.im_y_shift,
                                                  self.cfg.im_height,
                                                  self.cfg.im_width,
                                                  self.cfg.im_x_resolution,
                                                  self.cfg.im_y_resolution)
    
    def get_heightmap_ob(self, x_shift, y_shift, im_height, im_width, im_x_resolution, im_y_resolution):
        robot_xyz = self.cheat_state[0:3] + self.cam_pose_noise*self.cfg.cam_pose_std
        robot_rpy = self.cheat_state[3:6] + self.cam_rpy_noise*self.cfg.cam_rpy_std
        #print("xyz", robot_xyz)
        heightmap_ob = self.heightmap_sensor.get_heightmap_ob(robot_xyz, robot_rpy, x_shift, y_shift, im_height, im_width, im_x_resolution, im_y_resolution, self.cfg)
        #print("min heightmap ob", np.min(heightmap_ob))

        if self.lcm_publisher is not None and self.cfg.simulator_name is not "HARDWARE":
            self.lcm_publisher.broadcast_local_heightmap_lcm(heightmap_ob, self.simulator.ob, self.num_steps)

        return heightmap_ob

    def update_depth_camera_ob(self):
        self.depth_camera_ob = self.get_depth_camera_ob(self.camera_params)
        

    def get_depth_camera_ob(self, camera_params):
        depthImg, rgbImg = self.simulator.render_camera_image(camera_params, gimbal_camera=self.cfg.gimbal_camera)

        if self.cfg.clip_image_left_px > 0: # clip to avoid common edge fragments
            #print("CLIP", self.cfg.clip_image_left_px)
            depthImg[:, :self.cfg.clip_image_left_px] = 0

        return depthImg

    def update_rgb_camera_ob(self):
        self.rgb_camera_ob = self.get_rgb_camera_ob(self.camera_params)

    def get_rgb_camera_ob(self, camera_params):
        depthImg, rgbImg = self.simulator.render_camera_image(camera_params, gimbal_camera=self.cfg.gimbal_camera)
        return rgbImg

    def observe(self):
        return self.ob_scaled_vis

    '''
        Reward, Termination, Info
    '''

    def update_reward(self):
        
        raise NotImplementedError

        return self.total_reward

    def update_extra_info(self):
        raise NotImplementedError

        return self.extra_info

    def is_terminal_state(self):

        # universal safety checks for the robot!

        if self.test_mode == "DEPLOY":
            return self.done

        max_roll = 40 * np.pi/180
        max_pitch = 40 * np.pi/180

        body_rpy = self.cheat_state[3:6]
        if abs(body_rpy[0]) > max_roll or abs(body_rpy[1]) > max_pitch:
            self.done = True

        relative_foot_positions = np.array(self.simulator.get_foot_positions()) - np.array(self.cheat_state[0:3])
        for leg in range(4):
            if relative_foot_positions[leg][2] > 0:
                self.done = True

        return self.done

    def set_seed(self, seed=None):
        if seed is not None:
            np.random.seed(seed)

    def close(self):
        pass


    def __del__(self):
        self.close()
