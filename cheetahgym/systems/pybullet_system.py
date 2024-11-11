from cheetahgym.systems.system import System
from cheetahgym.systems.actuator_model import ActuatorModel
from cheetahgym.data_types.low_level_types import LowLevelCmd, LowLevelState
from cheetahgym.data_types.dynamics_parameter_types import DynamicsParameters
import pybullet as p
import pybullet_data

import time
import numpy as np
import copy
import collections
import random
import itertools

from PIL import Image, ImageDraw, ImageFont
import os
import cv2
import cProfile
from pathlib import Path

import pathlib

import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
   pass

from cheetahgym.utils.rotation_utils import get_rotation_matrix_from_quaternion, get_quaternion_from_rpy, get_rotation_matrix_from_rpy, get_rpy_from_quaternion, inversion

import pkgutil
#egl = pkgutil.get_loader('eglRenderer')

class PyBulletSystem(System):
    def __init__(self, cfg=None, gui=False, mpc_controller=None, initial_coordinates=None, fix_body=False, log_hd = False, is_shell=False, lcm_publisher=None):
        super().__init__(cfg)

        self.dt = self.cfg.simulation_dt
        self.gui = gui
        self.step_counter = 0
        self.sim_steps = 0
        self.desired_fps=60
        self.fix_body = fix_body
        self.terrain = None
        self.robot = None
        self.hmap_im = None
        self.terrainShape = None
        self.resetStateId = -1
        self.lcm_publisher = lcm_publisher
        

        self.record_frames = False#True
        self.rgb, self.depth = None, None
        self.FIXED_DEPTH_IMAGE = None

        if self.record_frames:
            try:
                self.datetime = cfg.datetime
            except Exception:
                self.datetime = time.strftime("%Y%m%d-%H%M%S")
                try:
                    os.makedirs(f'/data/img/pb_recordings/{self.datetime}/')
                except Exception:
                    pass

            self.render_width, self.render_height = 1800, 900

            import imageio
            self.mp4_writer = imageio.get_writer(f'/data/img/pb_recordings/{self.datetime}/recording.mp4', fps=self.desired_fps)# rgbImg[:, :, :3].transpose(2, 0, 1) , fps = 2)
            self.gif_writer = imageio.get_writer(f'/data/img/pb_recordings/{self.datetime}/recording.gif', fps=self.desired_fps)# rgbImg[:, :, :3].transpose(2, 0, 1) , fps = 2)



        #self.latency = 0.002
        self._obs_history = collections.deque(maxlen=100)
        self._pd_plus_torque_history = collections.deque(maxlen=100)
        self._applied_force_history = collections.deque(maxlen=100)
        self.foot_pos = np.zeros((4, 3))
        self.foot_vel = np.zeros((4, 3))

        self.log_hd = log_hd
        self.reset_logger()

        self.forces = np.zeros(12)
        self.external_force_vec, self.external_torque_vec = np.zeros(3), np.zeros(3)

        if self.cfg.use_actuator_model:
            self.actuator_model = ActuatorModel()

        

        if self.gui:
            optionstring = "--width=480 --height=360"
            self.physicsClient = p.connect(p.GUI, optionstring)#, key=np.random.randint(1, 10))
            p.setRealTimeSimulation(0, physicsClientId=self.physicsClient)
            #p.configureDebugVisualizer(p.COV_ENABLE_SINGLE_STEP_RENDERING,1)
            #p.configureDebugVisualizer(p.COV_ENABLE_GUI,0, physicsClientId=self.physicsClient)
            p.configureDebugVisualizer(p.COV_ENABLE_DEPTH_BUFFER_PREVIEW,1, physicsClientId=self.physicsClient)
            p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0, physicsClientId=self.physicsClient)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0, physicsClientId=self.physicsClient)
            #p.configureDebugVisualizer(shadowMapWorldSize=5, physicsClientId=self.physicsClient)
            #p.configureDebugVisualizer(shadowMapResolution=8192, physicsClientId=self.physicsClient)
            p.configureDebugVisualizer(lightPosition=[1, 0, 10], physicsClientId=self.physicsClient)
            print("GUI Connected")
            if self.cfg.record_video:
                print("Recording video to /data/cheetah.mp4")
                p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4, "/data/cheetah.mp4", physicsClientId = self.physicsClient)
        else:

            optionstring = "--width=480 --height=360"
            self.physicsClient = p.connect(p.DIRECT, optionstring)
            p.setRealTimeSimulation(0, physicsClientId=self.physicsClient)

            if self.cfg.use_egl:
                self.egl = pkgutil.get_loader('eglRenderer')
                if self.egl:
                    p.loadPlugin(self.egl.get_filename(), "_eglRendererPlugin",
                                 physicsClientId=self.physicsClient)
                else:
                    p.loadPlugin("eglRendererPlugin",
                                 physicsClientId=self.physicsClient)
            #print(p.isNumpyEnabled())
            #input()
        
        if initial_coordinates is None:
            initial_coordinates = np.array(
            [0.0, 0.0, 0.30, 1.0, 0.0, 0.0, 0.0, 0.0, -0.80, 1.6, 0.0, -0.80, 1.6, 0.0, -0.8, 1.6, 0.0,
             -0.8, 1.6, ])
        self.initial_coords = initial_coordinates
        self.mpc_controller = mpc_controller

        self._setup_pybullet()

        if is_shell:
            return


        self.ob = self.reset_system(initial_coordinates)
        #input()

        if self.cfg.camera_source == "PYBULLET" or self.cfg.camera_source == "BOTH":
            from cheetahgym.sensors.realsense_pybullet import RealsensePB
            self.realsense_camera = RealsensePB(rgb=False,
                                                img_height=480,
                                                img_width=640,
                                                depth_min=0.15,
                                                depth_max=4.0,
                                                grid_size=0.020,
                                                height_map_bounds=None,
                                                physicsClientId=self.physicsClient)
        elif self.cfg.camera_source == "PYBULLET_ASYNC":
            from cheetahgym.sensors.realsense_pb_async import RealsensePBAsync
            self.realsense_camera = RealsensePBAsync(rgb=False,
                                                img_height=480,
                                                img_width=640,
                                                depth_min=0.15,
                                                depth_max=4.0,
                                                grid_size=0.020,
                                                height_map_bounds=None,
                                                physicsClientId=self.physicsClient)


        elif self.cfg.camera_source == "RS_API":
            from cheetahdeploy.sensors.realsense_api import RealsenseAPI
            self.realsense_camera = RealsenseAPI(rgb=False,
                                                img_height=480,
                                                img_width=640,
                                                depth_min=0.15,
                                                depth_max=4.0,
                                                grid_size=0.020,
                                                height_map_bounds=None)
        elif self.cfg.camera_source == "DUMMY":
            from cheetahgym.sensors.realsense_dummy import RealsenseDummy
            self.realsense_camera = RealsenseDummy(rgb=False,
                                                img_height=480,
                                                img_width=640,
                                                depth_min=0.15,
                                                depth_max=4.0,
                                                grid_size=0.020,
                                                height_map_bounds=None,
                                                physicsClientId=self.physicsClient)

        #print("SET UP PB")

        if self.resetStateId < 0:
            self.resetStateId = p.saveState(physicsClientId=self.physicsClient)


    def _setup_pybullet(self):
        p.setGravity(0, 0, -9.8, physicsClientId=self.physicsClient)
        #p.setPhysicsEngineParameter(#fixedTimeStep=self.cfg.simulation_dt,
                            #solverResidualThreshold=1e-30,
                            #numSolverIterations=200,
                            #numSubSteps=4,
        #                   physicsClientId=self.physicsClient)
        #p.setTimeStep(self.cfg.simulation_dt, physicsClientId=self.physicsClient)
        p.resetDebugVisualizerCamera(1.0, 0, -15.0, [0, 0, 0], physicsClientId=self.physicsClient)
        p.setPhysicsEngineParameter(fixedTimeStep=self.cfg.simulation_dt,
                                    numSolverIterations=50,#300, 
                                    solverResidualThreshold=1e-30, 
                                    numSubSteps=1,
                                    physicsClientId=self.physicsClient)

        robot_start_pos = self.initial_coords[0:3]
        robot_start_ori = [self.initial_coords[4], self.initial_coords[5], self.initial_coords[6], self.initial_coords[3]]

        self.robot = p.loadURDF(str(pathlib.Path(__file__).parent.parent.absolute())+"/urdf/mini_cheetah_simple.urdf", robot_start_pos, robot_start_ori,
                       useFixedBase=False,
                       #useMaximalCoordinates=True,
                       physicsClientId=self.physicsClient)

        if self.cfg.simulator_name == "PYBULLET":
            self.motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14]
            self.foot_frames = [3, 7, 11, 15]
            self.num_joints = 16
        else:
            self.motor_id_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
            self.foot_frames = [2, 5, 8, 11]
            self.num_joints = 112

        jointIds = []
        for j in range(p.getNumJoints(self.robot, physicsClientId=self.physicsClient)):
            jointIds.append(j)


    def update_obs(self, joint_only=False):
        self._obs_history.appendleft(self.get_obs(joint_only=joint_only))
        self.ob = self.get_delayed_obs(self.dp.control_latency_seconds)
        return self.ob

    def get_delayed_obs(self, latency):
        if latency <= 0 or len(self._obs_history) == 1:
            #print("STARTER OBS")
            ob = self._obs_history[0]
        else:
            n_steps_ago = int(latency / self.dt)
            if n_steps_ago + 1 >= len(self._obs_history):
                return self._obs_history[-1]
            remaining_latency = latency - n_steps_ago * self.dt
            blend_alpha = remaining_latency / self.dt
            ob_vec = ((1.0 - blend_alpha) * np.array(self._obs_history[n_steps_ago].to_vec()) + 
                    blend_alpha * np.array(self._obs_history[n_steps_ago+1].to_vec()))
            ob = LowLevelState()
            ob.from_vec(ob_vec)
        return ob

    def get_delayed_pd_plus_torque(self, latency):
        #input(latency)
        if latency <= 0 or len(self._pd_plus_torque_history) == 1:
            #print("STARTER OBS")
            pd_plus_torque = self._pd_plus_torque_history[0]
        else:
            n_steps_ago = int(latency / self.dt)
            if n_steps_ago + 1 >= len(self._pd_plus_torque_history):
                return self._pd_plus_torque_history[-1]
            pd_plus_torque = self._pd_plus_torque_history[n_steps_ago]
        return pd_plus_torque

    def get_obs(self, joint_only=False):

        ob = LowLevelState()

        joint_state = np.array(p.getJointStates(self.robot, self.motor_id_list, physicsClientId=self.physicsClient))



        ob.joint_pos = joint_state[:, 0]
        ob.joint_vel = joint_state[:, 1]

        if(np.max(np.abs(ob.joint_pos)) > 100000 or np.max(np.abs(ob.joint_vel)) > 100000):
            ob = self.reset_system(self.initial_coords)
            return ob


        pose = p.getBasePositionAndOrientation(self.robot, physicsClientId=self.physicsClient)
        ob.body_pos = np.array(pose[0])
        robot_quat = [pose[1][3], pose[1][0], pose[1][1], pose[1][2]]
        try:
            ob.body_rpy = get_rpy_from_quaternion(robot_quat)
            #ob.body_rpy[1] = -1 * ob.body_rpy[1]
        except ValueError:
            print("NaN in robot_quat!! step ", self.step_counter)
            print("state on failure", vars(ob))
            ob = self.reset_system(self.initial_coords)
            return ob
        rot = get_rotation_matrix_from_quaternion(robot_quat)

        ob.body_pos[2] = ob.body_pos[2] + self.cfg.adjust_body_height_pb

        vel = np.array(p.getBaseVelocity(self.robot, physicsClientId=self.physicsClient))


        if self.cfg.observe_corrected_vel:
            ob.body_linear_vel = rot.T.dot(vel[0, :])
            ob.body_angular_vel = rot.T.dot(vel[1, :])
        else:
            ob.body_linear_vel = rot.dot(vel[0, :])
            ob.body_angular_vel = rot.dot(vel[1, :])

        #ob.body_angular_vel[1] = -1 * ob.body_angular_vel[1]
            
        if(np.max(np.abs(ob.body_pos)) > 100000 or np.max(np.abs(ob.body_rpy)) > 100000 or
                np.max(np.abs(ob.body_linear_vel)) > 100000 or np.max(np.abs(ob.body_angular_vel)) > 100000):
            ob = self.reset_system(self.initial_coords)
        if(np.isnan(np.sum(ob.to_vec()))):
            ob = self.reset_system(self.initial_coords)
            print("NaN ERROR!!")
            print(ob)
             


        #print("lv", vel[0])
        #print("ob.lv", ob.body_linear_vel)


        self.update_foot_positions()

        # always track motor vels
        self.motor_velocities += [ob.joint_vel]
        # log hd state
        if self.log_hd:
            #print("LOG HD")
            self.body_heights += [ob.body_pos[2]] # body height
            self.body_xpos += [ob.body_pos[0]] # body x
            self.body_ypos += [ob.body_pos[1]] # body y
            self.body_oris += [ob.body_rpy]
            self.body_vels += [ob.body_linear_vel[0]] # CoM Velocity
            self.body_linear_vels += [ob.body_linear_vel]
            self.body_angular_vels += [ob.body_angular_vel]
            self.torques += [self.forces] # torques
            self.motor_positions += [ob.joint_pos] # motor velocities
            #self.motor_velocities += [ob.joint_vel] # motor velocities
            self.foot_positions += [self.get_foot_positions()]



        return ob

    def check_numerical_error(self):
        if  (np.abs(np.max(self.ob.joint_pos)) > 100000 or np.abs(np.max(self.ob.joint_vel)) > 100000):
            
            print("PYBULLET NUMERICAL ERROR!!")
            self.ob = self.reset_system(self.initial_coords)
            return True

        if  (np.abs(np.max(self.ob.body_pos)) > 100000 or np.abs(np.max(self.ob.body_rpy)) > 100000 or
             np.abs(np.max(self.ob.body_linear_vel)) > 100000 or np.abs(np.max(self.ob.body_angular_vel)) > 100000):
            
            print("PYBULLET NUMERICAL ERROR!!")
            self.ob = self.reset_system(self.initial_coords)

        if np.any(np.isnan(self.ob.to_vec())):
            print("PYBULLET NUMERICAL ERROR!! NaN Error")
            self.ob = self.reset_system(self.initial_coords)
            #ob = LowLevelState()

        return False

    def set_state(self, state):
        quat = get_quaternion_from_rpy(state.body_rpy)
        quat = [quat[1], quat[2], quat[3], quat[0]]
        rot = get_rotation_matrix_from_quaternion(quat)
        #quat = [quat[3], quat[0], quat[1], quat[2]]
        p.resetBasePositionAndOrientation(
            self.robot, state.body_pos, quat, physicsClientId=self.physicsClient)
        p.resetBaseVelocity(self.robot, state.body_linear_vel, state.body_angular_vel, physicsClientId=self.physicsClient)

        #if self.fix_body:
        # p.createConstraint(
        #    self.robot, -1, -1, -1, p.JOINT_FIXED,
        #    [0, 0, 0], [0, 0, 0], robot_start_pos, physicsClientId=self.physicsClient)

        for j in range(12):
            p.resetJointState(  self.robot,
                                (self.motor_id_list[j]), 
                                targetValue=state.joint_pos[j],
                                targetVelocity=state.joint_vel[j], 
                                physicsClientId=self.physicsClient)
        self.update_obs()

        if self.gui: # and self.step_counter % 30 == 0 
            #print("cam", self.ob.body_pos)
            #pass
            p.resetDebugVisualizerCamera(1.0, 0, -15.0, self.ob.body_pos, physicsClientId=self.physicsClient)
            # self.render_camera_image()
        #print("STATE SET")

    def set_camera_image(self, depth_image): # manually set percepy
        #print("SETTING CAM IMAGE")
        #input()
        if self.FIXED_DEPTH_IMAGE is None:
            self.reset_system_hard()
        self.FIXED_DEPTH_IMAGE = depth_image

    def step_state_low_level(self, low_level_cmd, loop_count, lag_steps=0, interpolation_factor = 1, forces=None, resetlog=True, contact_state=None):
        

        if resetlog:
            self.reset_logger()

        t = time.time()

        p_targets_cmd, v_targets_cmd = low_level_cmd.p_targets, low_level_cmd.v_targets
        p_gains_cmd, v_gains_cmd = low_level_cmd.p_gains, low_level_cmd.v_gains
        ff_torque_cmd = low_level_cmd.ff_torque


        ob = self.get_delayed_obs(latency=self.dp.pd_latency_seconds)
        p_current = ob.joint_pos
        v_current = ob.joint_vel


        vis_decimation = int(1. / (self.desired_fps * self.dt + 1e-10))

        #if np.sum(ff_torque) == 0:
        #    PD_CONTROL_MODE = True
        #else:
        #    PD_CONTROL_MODE = False
        PD_CONTROL_MODE=False

        if PD_CONTROL_MODE:
                p.setJointMotorControlArray(bodyIndex=self.robot,
                      jointIndices=self.motor_id_list,
                      controlMode=p.POSITION_CONTROL,
                      targetPositions=p_targets[-12:],
                      targetVelocities=v_targets[-12:],
                      forces=[15 for j in range(12)],
                      physicsClientId=self.physicsClient,
                      positionGains=p_gains[-12:], 
                      velocityGains=v_gains[-12:], 
                )

        MAX_FORCE = self.dp.motor_strength
       
        self.custom_timer = 0.
        for i in range(loop_count - lag_steps):
            ctime = time.time()
            if i > 0:
                self.update_obs()#joint_only=True)
            ob = self.get_delayed_obs(latency=self.dp.pd_latency_seconds)
            
            self._pd_plus_torque_history.appendleft([p_targets_cmd, v_targets_cmd, ff_torque_cmd, p_gains_cmd, v_gains_cmd])
            [p_targets, v_targets, ff_torque, p_gains, v_gains] = self.get_delayed_pd_plus_torque(latency=self.dp.pdtau_command_latency_seconds)

            p_targets = p_targets_cmd
            v_targets = v_targets_cmd

            p_current = ob.joint_pos
            v_current = ob.joint_vel
            
            if PD_CONTROL_MODE:
                pass
            else:
                if self.cfg.use_actuator_model:
                    self.actuator_model.update_cmd(p_targets[-12:], v_targets[-12:], ff_torque[-12:], p_gains[-12:], v_gains[-12:])
                    tau_des = self.actuator_model.compute_low_level_cmd(p_current, v_current)
                    self.forces = self.actuator_model.get_torque(tau_des, v_current)
                else:
                    self.forces = ff_torque[-12:] - np.multiply(p_gains[-12:], p_current - p_targets[-12:]) - np.multiply(v_gains[-12:], v_current - v_targets[-12:])

                self.custom_timer += time.time() - ctime

                # enforce max force
                self.forces = np.core.umath.clip(self.forces, -MAX_FORCE, MAX_FORCE)

                # apply joint friction and damping
                self.forces -= self.dp.joint_dampings * v_current 
                try:
                    self.forces -= self.dp.joint_frictions * np.sign(v_current)
                except Exception:
                    print("ERROR IN PBSYSTEM LINE 500")
                    print(self.forces)
                    print(self.dp.joint_frictions)
                    print(ff_torque, p_gains, p_current, p_targets, v_gains, v_current, v_targets)


                self._applied_force_history.appendleft(self.forces)

                applied_forces = np.mean(np.array(list(itertools.islice(self._applied_force_history, 0, self.dp.force_lowpass_filter_window))), axis=0)

                p.setJointMotorControlArray(bodyIndex=self.robot,
                          jointIndices=self.motor_id_list,
                          controlMode=p.TORQUE_CONTROL,
                          targetVelocities=[0 for j in range(12)],
                          forces=applied_forces,
                          physicsClientId=self.physicsClient)


            force_application_duration = int(self.cfg.external_force_interval_ts/self.cfg.simulation_dt)
            if self.sim_steps % force_application_duration == 0 and self.sim_steps > 0:  # set external force
                perturbation_force = self.cfg.external_force_magnitude * np.random.choice([0,1], p=[1-self.cfg.external_force_prob,self.cfg.external_force_prob])

                force_direction = np.random.choice(np.arange(0,8)*np.pi/4)
                self.external_force_vec = np.array([perturbation_force*np.cos(force_direction), perturbation_force * np.sin(force_direction), 0.])
                self.external_torque_vec = self.cfg.external_torque_magnitude * np.array([0., 0., np.random.choice([-1, 0, 1], p=[self.cfg.external_torque_prob/2, 1-self.cfg.external_torque_prob,self.cfg.external_torque_prob/2])])
               
            p.applyExternalForce(objectUniqueId=self.robot,
                             linkIndex=0,
                             forceObj=self.external_force_vec,
                             posObj=np.zeros(3),
                             flags=p.LINK_FRAME,
                             physicsClientId=self.physicsClient)

            p.applyExternalTorque(objectUniqueId=self.robot,
                                  linkIndex=0,
                                  torqueObj=self.external_torque_vec,
                                  flags=p.LINK_FRAME,#WORLD_FRAME,
                                  physicsClientId=self.physicsClient)

            #  implement external force perturbations to the feet

            if self.cfg.foot_external_force_magnitude != 0:
                foot_positions = self.get_foot_positions()
                for i in range(4):
                    if self.sim_steps % force_application_duration == 0 and self.sim_steps > 0:  # set foot external force
                        if self.cfg.foot_external_force_prob > np.random.random():
                            # print("apply foot force!")
                            self.foot_external_force_vec[i] = self.cfg.foot_external_force_magnitude * np.array(
                                [2 * np.random.random() - 1, 2 * np.random.random() - 1, 0.])
                        else:
                            self.foot_external_force_vec[i] = np.zeros(3)

                    p.applyExternalForce(objectUniqueId=self.robot,
                                         linkIndex=self.foot_frames[i],
                                         forceObj=self.foot_external_force_vec[i],
                                         posObj=foot_positions[i],
                                         flags=p.WORLD_FRAME,
                                         physicsClientId=self.physicsClient)

            p.stepSimulation(physicsClientId=self.physicsClient)
            self.sim_steps += 1


            if self.record_frames:
                vis_decimation = int(1. / (self.desired_fps * self.dt + 1e-10))
                if (self.step_counter * loop_count + i) % vis_decimation == 0:
                    self.save_render()

        if self.lcm_publisher is not None:
            self.lcm_publisher.broadcast_action_lcm(low_level_cmd, self.step_counter)
            self.lcm_publisher.broadcast_state_lcm(ob, self.step_counter, applied_forces, self.get_foot_positions(), False)

        self.step_counter += 1
        self.update_obs()



        
        if self.gui:
            p.resetDebugVisualizerCamera(1.0, 0, -15.0, self.ob.body_pos, physicsClientId=self.physicsClient)
            
        return self.ob

    def add_heightmap_file(self, hmap_filename, hmap_cfg):
        raise NotImplementedError

    def add_heightmap_array(self, hmap_im, body_pos, resolution=0.02):
        #input("ADDING")
        #if np.min(hmap_im) == 0:
        #    return
        if self.cfg.rough_terrain:
            heightPerturbationRange = self.cfg.pertub_h
            numHeightfieldRows = 140
            numHeightfieldColumns = 50
            heightfieldData = [0] * numHeightfieldRows * numHeightfieldColumns

            for j in range(int(numHeightfieldColumns / 2)):
                for i in range(int(numHeightfieldRows / 2)):
                    height = random.uniform(0, heightPerturbationRange)
                    heightfieldData[2 * i + 2 * j * numHeightfieldRows] = height
                    heightfieldData[2 * i + 1 + 2 * j * numHeightfieldRows] = height
                    heightfieldData[2 * i + (2 * j + 1) * numHeightfieldRows] = height
                    heightfieldData[2 * i + 1 + (2 * j + 1) * numHeightfieldRows] = height

            if self.terrainShape is None:
                self.terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1],
                                                           heightfieldTextureScaling=(numHeightfieldRows - 1) / 16,
                                                           heightfieldData=heightfieldData,
                                                           numHeightfieldRows=numHeightfieldRows,
                                                           numHeightfieldColumns=numHeightfieldColumns,
                                                           physicsClientId=self.physicsClient)
                self.terrain = p.createMultiBody(0, self.terrainShape, physicsClientId=self.physicsClient)
                try:
                    text_file = Path(__file__).resolve().parent.parent + self.cfg.texture
                    text_id = p.loadTexture(text_file, physicsClientId=self.physicsClient)
                    p.changeVisualShape(self.terrain, -1, textureUniqueId=text_id, rgbaColor=[1., 1., 1., 1.0],physicsClientId=self.physicsClient)
                except Exception:
                    pass
                p.resetBasePositionAndOrientation(self.terrain, [3, 0, 0], [0, 0, 0, 1],physicsClientId=self.physicsClient)
                p.changeDynamics(self.terrain, -1, lateralFriction=self.dp.ground_friction,
                             spinningFriction=self.dp.ground_spinning_friction,
                             rollingFriction=self.dp.ground_rolling_friction,
                             restitution=1,
                             contactProcessingThreshold=self.dp.contact_processing_threshold,
                             physicsClientId=self.physicsClient)
                #self.resetStateId = p.saveState(physicsClientId=self.physicsClient)

            else:
                self.terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.05, .05, 1],
                                                           heightfieldTextureScaling=(numHeightfieldRows - 1) / 16,
                                                           heightfieldData=heightfieldData,
                                                           numHeightfieldRows=numHeightfieldRows,
                                                           numHeightfieldColumns=numHeightfieldColumns,replaceHeightfieldIndex = self.terrainShape,
                                                           physicsClientId=self.physicsClient)
                p.changeDynamics(self.terrain, -1, lateralFriction=self.dp.ground_friction,
                             spinningFriction=self.dp.ground_spinning_friction,
                             rollingFriction=self.dp.ground_rolling_friction,
                             restitution=1,
                             contactProcessingThreshold=self.dp.contact_processing_threshold,
                             physicsClientId=self.physicsClient)

        else:
            # add flat heightfield
            HORIZONTAL_STRETCH_FACTOR=4 # to speed up gap terrains
            self.hmap_im = hmap_im
            self.body_pos = body_pos
            self.hmap_body_pos = body_pos
            self.hmap_resolution = resolution
            numHeightfieldRows = hmap_im.shape[0]
            numHeightfieldColumns = hmap_im.shape[1] // HORIZONTAL_STRETCH_FACTOR
            if HORIZONTAL_STRETCH_FACTOR > 1:
                heightfieldData = hmap_im[:, 0:hmap_im.shape[1]:HORIZONTAL_STRETCH_FACTOR].T.flatten()
            else:
                heightfieldData = hmap_im.T.flatten()

            text_file = str(Path(__file__).resolve().parent.parent) + "/" + self.cfg.texture
            text_id = p.loadTexture(text_file, physicsClientId = self.physicsClient)
                
            if self.terrainShape is None:
                self.terrainShape = p.createCollisionShape(  shapeType = p.GEOM_HEIGHTFIELD,
                                                        meshScale=[resolution, resolution*HORIZONTAL_STRETCH_FACTOR,1],
                                                        heightfieldTextureScaling=self.cfg.texture_scale *(numHeightfieldRows-1)/2,
                                                        heightfieldData=heightfieldData,
                                                        numHeightfieldRows=numHeightfieldRows,
                                                        numHeightfieldColumns=numHeightfieldColumns,
                                                        physicsClientId=self.physicsClient)
                self.terrain  = p.createMultiBody(0, self.terrainShape, physicsClientId=self.physicsClient)
                #try:
                p.changeVisualShape(self.terrain, -1, textureUniqueId = text_id, rgbaColor=[1.,1.,1.,1.0], physicsClientId=self.physicsClient)
                #except Exception:
                #    pass
                p.setDebugObjectColor(self.terrain, -1, objectDebugColorRGB=[0, 0, 1], physicsClientId=self.physicsClient)
                p.resetBasePositionAndOrientation(self.terrain, [body_pos[0], body_pos[1], (np.min(heightfieldData) - np.max(heightfieldData))/2 + heightfieldData[0]], [0, 0, 0, 1], physicsClientId=self.physicsClient)
                p.changeDynamics(self.terrain, -1, lateralFriction=self.dp.ground_friction,
                                                   spinningFriction=self.dp.ground_spinning_friction,
                                                   rollingFriction=self.dp.ground_spinning_friction,
                                                   restitution=1,
                                                   contactProcessingThreshold=self.dp.contact_processing_threshold,
                                                   physicsClientId=self.physicsClient)
                self.resetStateId = p.saveState(physicsClientId=self.physicsClient)
            else:
                p.removeBody(bodyUniqueId=self.terrain, physicsClientId=self.physicsClient)
                #p.removeCollisionShape(collisionShapeId=self.terrainShape, physicsClientId=self.physicsClient)
                self.terrainShape = p.createCollisionShape(  shapeType = p.GEOM_HEIGHTFIELD,
                                                        meshScale=[resolution, resolution*HORIZONTAL_STRETCH_FACTOR,1],
                                                        heightfieldTextureScaling= self.cfg.texture_scale *(numHeightfieldRows-1)/2,
                                                        heightfieldData=heightfieldData,
                                                        numHeightfieldRows=numHeightfieldRows,
                                                        numHeightfieldColumns=numHeightfieldColumns,
                                                        #replaceHeightfieldIndex = self.terrainShape,
                                                        physicsClientId=self.physicsClient)
                self.terrain  = p.createMultiBody(0, self.terrainShape, physicsClientId=self.physicsClient)
                p.changeVisualShape(self.terrain, -1, textureUniqueId = text_id, rgbaColor=[1.,1.,1.,1.0], physicsClientId=self.physicsClient)
                p.setDebugObjectColor(self.terrain, -1, objectDebugColorRGB=[0, 0, 1], physicsClientId=self.physicsClient)
                p.resetBasePositionAndOrientation(self.terrain, [body_pos[0], body_pos[1], (np.min(heightfieldData) - np.max(heightfieldData))/2 + heightfieldData[0]], [0, 0, 0, 1], physicsClientId=self.physicsClient)
                p.changeDynamics(self.terrain, -1, lateralFriction=self.dp.ground_friction,
                                                   spinningFriction=self.dp.ground_spinning_friction,
                                                   rollingFriction=self.dp.ground_spinning_friction,
                                                   restitution=1,
                                                   contactProcessingThreshold=self.dp.contact_processing_threshold,
                                                   physicsClientId=self.physicsClient)

    def reset_system_hard(self, initial_coords=None):
        if initial_coords is None:
            initial_coords = np.array( [0.0, 0.0, 0.295, 1.0, 0.0, 0.0, 0.0, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62,])
        if self.gui: p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0, physicsClientId=self.physicsClient)
        self.terrain = None
        self.robot = None
        self.hmap_im = None
        self.terrainShape = None
        self.resetStateId = -1
        p.resetSimulation(physicsClientId=self.physicsClient)
        self._setup_pybullet()
        self.prev_feet_pos = self.get_foot_positions()
        
        return self.reset_system(initial_coords)

    def reset_system(self, initial_coords=None):

        if initial_coords is None:
            initial_coords = np.array( [0.0, 0.0, 0.295, 1.0, 0.0, 0.0, 0.0, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62, 0.057, -0.80, 1.62,])
        if self.gui: p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,0, physicsClientId=self.physicsClient)
        self.remove_debug_items()
        self._obs_history.clear()
        self._pd_plus_torque_history.clear()
        self._applied_force_history.clear()
        # initialize pdplustorque history
        for i in range(100):
            self._pd_plus_torque_history.appendleft([initial_coords, np.zeros(18), np.zeros(18), 40 * np.ones(19), 2 * np.ones(18)])
            self._applied_force_history.appendleft(np.zeros(12))

        self.external_force_vec, self.external_torque_vec = np.zeros(3), np.zeros(3)
        self.foot_external_force_vec = [np.zeros(3) for i in range(4)]
        self.step_counter = 0
        self.sim_steps = 0
        self.target_x_pos = 0

        if self.mpc_controller is not None:
            self.mpc_controller.reset_ctrl()
        if self.resetStateId >= 0:
            p.restoreState(self.resetStateId, physicsClientId=self.physicsClient)
            if self.gui: p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1, physicsClientId=self.physicsClient)
            robot_start_pos = initial_coords[0:3]
            robot_start_ori = [initial_coords[4], initial_coords[5], initial_coords[6], initial_coords[3]]

            for body in range(p.getNumBodies()):
                p.resetBaseVelocity(body, [0,0,0], [0,0,0], physicsClientId=self.physicsClient)

            p.resetBasePositionAndOrientation(
                self.robot, robot_start_pos, robot_start_ori, physicsClientId=self.physicsClient)

            
            for j in range(12):
                p.resetJointState(  self.robot,
                                    (self.motor_id_list[j]), 
                                    targetValue=initial_coords[j+7],
                                    targetVelocity=0, 
                                    physicsClientId=self.physicsClient)

            ob = self.update_obs()
            return ob

        robot_start_pos = initial_coords[0:3]
        robot_start_ori = [initial_coords[4], initial_coords[5], initial_coords[6], initial_coords[3]]
        robot_start_lin_vel = [0, 0, 0]
        robot_start_ang_vel = [0, 0, 0]

        p.resetBasePositionAndOrientation(
            self.robot, robot_start_pos, robot_start_ori, physicsClientId=self.physicsClient)
        p.resetBaseVelocity(self.robot, robot_start_lin_vel, robot_start_ang_vel, physicsClientId=self.physicsClient)
        self.prev_feet_pos = self.get_foot_positions()

        if self.fix_body:
         p.createConstraint(
            self.robot, -1, -1, -1, p.JOINT_FIXED,
            [0, 0, 0], [0, 0, 0], robot_start_pos, physicsClientId=self.physicsClient)

        for j in range(12):
            p.resetJointState(  self.robot,
                                (self.motor_id_list[j]), 
                                targetValue=initial_coords[j+7],
                                targetVelocity=0, 
                                physicsClientId=self.physicsClient)

        p.setJointMotorControlArray(bodyIndex=self.robot,
                  jointIndices=self.motor_id_list,
                  controlMode=p.POSITION_CONTROL,
                  targetPositions=self.initial_coords[-12:],
                  targetVelocities=[0 for j in range(12)],
                  forces=[20 for j in range(12)],
                  physicsClientId=self.physicsClient)

        p.setJointMotorControlArray(bodyIndex=self.robot,
                      jointIndices=[j for j in range(self.num_joints)],
                      controlMode=p.VELOCITY_CONTROL,
                      targetVelocities=[0 for j in range(self.num_joints)],
                      forces=[0 for j in range(self.num_joints)],
                      physicsClientId=self.physicsClient)

        p.setJointMotorControlArray(bodyIndex=self.robot,
                      jointIndices=[j for j in range(self.num_joints)],
                      controlMode=p.TORQUE_CONTROL,
                      targetVelocities=[0 for j in range(self.num_joints)],
                      forces=[0 for j in range(self.num_joints)],
                      physicsClientId=self.physicsClient)


        p.stepSimulation(physicsClientId=self.physicsClient)
        self.sim_steps+=1

        if self.resetStateId < 0:
            if self.terrainShape is None:
                self.add_heightmap_array(np.zeros((2000, 200)), body_pos=[19.0, 0.0])

            self.resetStateId = p.saveState(physicsClientId=self.physicsClient)
            
        if self.gui: p.configureDebugVisualizer(p.COV_ENABLE_RENDERING,1, physicsClientId=self.physicsClient)

        return self.update_obs()


    def get_nominal_dynamics(self):
        
        dp = DynamicsParameters(self.cfg)
        
        for i in range(self.num_joints):
            dynamicsInfo = p.getDynamicsInfo(bodyUniqueId=self.robot,
                                             linkIndex=i,
                                             physicsClientId=self.physicsClient,)
            dp.link_masses[i] = dynamicsInfo[0]
            dp.link_inertias[i] = dynamicsInfo[2]

        dp.motor_strength_std = 0.20 * dp.motor_strength
        dp.link_masses_std = 0.50 * dp.link_masses
        dp.link_inertias_std = 0.05 * dp.link_inertias

        return dp

    def set_dynamics(self, dynamics):

        self.dp = copy.deepcopy(dynamics)
        for i in range(16):
            p.changeDynamics(bodyUniqueId=self.robot,
                             linkIndex=i,
                             physicsClientId = self.physicsClient,
                             mass=self.dp.link_masses[i],
                             restitution=self.dp.ground_restitution,
                             localInertiaDiagonal = self.dp.link_inertias[i])

    def update_foot_positions(self):
        # only call when necessary
        self.foot_pos = [None, None, None, None]
        self.foot_vel = [None, None, None, None]
        for idx in range(4):
            self.foot_pos[idx], _, _, _, _, _, self.foot_vel[idx], _ = p.getLinkState(self.robot, 
                                                 linkIndex=self.foot_frames[idx],
                                                 computeLinkVelocity=True,
                                                 physicsClientId=self.physicsClient)


    def get_foot_positions(self, relative=False):

        if relative:
            R = get_rotation_matrix_from_rpy(self.ob.body_rpy)
            return [R.T.dot(np.array(self.foot_pos[i]) - self.ob.body_pos) for i in range(4)]
        return self.foot_pos

    def get_foot_velocities(self, relative=False):
        
        if relative:
            R = get_rotation_matrix_from_rpy(self.ob.body_rpy)
            return [R.T.dot(np.array(self.foot_vel[i]) - self.ob.body_linear_vel) for i in range(4)]
        return self.foot_vel
    
    def remove_debug_items(self):
        p.removeAllUserDebugItems(physicsClientId=self.physicsClient)
    
    def vis_foot_traj(self, line_thickness=2.0,life_time = 6):
        current_feet_pos = np.zeros((4,3))
        feet_colors = np.array([[1,0,0],[1,1,0],[0,0,1],[0,1,0]])
        for i, link_id in enumerate(self.foot_frames):
            current_feet_pos[i]  = np.array(p.getLinkState(self.robot, link_id, physicsClientId=self.physicsClient)[0])
            p.addUserDebugLine(current_feet_pos[i],self.prev_feet_pos[i],feet_colors[i],line_thickness,lifeTime=life_time, physicsClientId=self.physicsClient)
        self.prev_feet_pos = current_feet_pos

    def get_shank_contact_state(self):
        if self.terrain is None:
            return [0, 0, 0, 0]
        contact_state = np.zeros(4)
        if self.cfg.simulator_name == "PYBULLET":
            foot_indices = {2: 0, 6: 1, 10: 2, 14: 3}
        else: #if self.cfg.simulator_name == "PYBULLET_MESHMODEL":
            foot_indices = {1: 0, 4: 1, 7: 2, 10: 3}
        for contact in p.getContactPoints(bodyA=self.robot, bodyB=self.terrain, physicsClientId=self.physicsClient):
            if contact[3] in foot_indices:
                contact_state[foot_indices[contact[3]]] = 1
        return contact_state

    def get_contact_state(self):
        if self.terrain is None:
            return [0, 0, 0, 0]
        contact_state = np.zeros(4)
        foot_indices = {self.foot_frames[0]: 0, 
                        self.foot_frames[1]: 1, 
                        self.foot_frames[2]: 2, 
                        self.foot_frames[3]: 3}

        for contact in p.getContactPoints(bodyA=self.robot, bodyB=self.terrain, physicsClientId=self.physicsClient):
            if contact[3] in foot_indices:
                contact_state[foot_indices[contact[3]]] = 1
        return contact_state

    def save_render(self):


        #distance = 2.0
        distance = 1.5 #2.0
        focus_height = 0.4 # 0.5
        render_text = True     
        self.target_x_pos = 0.1 * self.ob.body_pos[0] + 0.9 * self.target_x_pos 

        viewMatrix = p.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=[self.target_x_pos, 0, focus_height],
            distance=distance,
            yaw=0,
            pitch=-10,
            roll=0,
            upAxisIndex=2,
            physicsClientId=self.physicsClient)

        projectionMatrix = p.computeProjectionMatrixFOV(
                fov=40,
                aspect=self.render_width/self.render_height,
                nearVal=0,
                farVal=100,
                physicsClientId=self.physicsClient)

        width, height, rgbImg, depthImg, segImg = p.getCameraImage(
                width=self.render_width, 
                height=self.render_height,
                viewMatrix=viewMatrix,
                projectionMatrix=projectionMatrix,
                physicsClientId=self.physicsClient)
        print(rgbImg.shape)
        #nC = 900
        #nR = 1800
        #rgbImg = np.array([[rgbImg[int(rgbImg.shape[0]*r/nR), int(rgbImg.shape[1]*c/nC), :] for c in range(nC)] for r in range(nR)])
        rgbImg = cv2.resize(rgbImg, dsize=(1800, 900), interpolation=cv2.INTER_CUBIC)

        print(rgbImg.shape)
        #rgb_rs, depth_rs = self.render_camera_image()

        # put rgb, depth in corner if they exist
        if self.depth is not None:
            print("RENDER DEPTH IMAGE!")
            print(self.depth.shape)
            depth = cv2.resize(self.depth, dsize=(160, 120), interpolation=cv2.INTER_CUBIC)

            cm = plt.get_cmap('viridis')
            colored_depth = cm(depth)
            rgbImg[75:75+depth.shape[0], 700:700+depth.shape[1], :3] = colored_depth[:, :, :3] * 255


        if self.rgb is not None:
            print(self.rgb.shape)
            rgb = cv2.resize(self.rgb, dsize=(160, 120), interpolation=cv2.INTER_CUBIC)
            #rgbImg[450:450+self.depth.shape[0], 300:300+self.depth.shape[1], :3] = self.rgb
            rgbImg[75:75+rgb.shape[0], 960:960+rgb.shape[1], :3] = rgb


        im = Image.fromarray(rgbImg[:, :, :3])
        draw = ImageDraw.Draw(im)
        font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf", 16, encoding='unic')

        if self.depth is not None:
            msg = "Depth Image"
            w, h = draw.textsize(msg, font=font)
            draw.text((780-w/2, 40-h/2), msg, 'black', font=font)
        if self.rgb is not None:
            msg = "RGB Image"
            w, h = draw.textsize(msg, font=font)
            draw.text((1040-w/2, 40-h/2), msg, 'black', font=font)
        rgbImg = np.array(im)



        im.save(f'/data/img/pb_recordings/{self.datetime}/{self.step_counter}.pdf')
        im.save(f'/data/img/pb_recordings/{self.datetime}/{self.step_counter}.jpg')


        #print(rgbImg.shape)
        #self.pipe.stdin.write( rgbImg[:, :, :3].astype(np.uint8).tobytes() )
        
        #import imageio
        #imageio.mimwrite(f'/data/img/pb_recordings/{self.datetime}/recording.mp4', rgbImg[:, :, :3].transpose(2, 0, 1) , fps = 2)

        self.mp4_writer.append_data( rgbImg[:, :, :3])
        self.gif_writer.append_data( rgbImg[:, :, :3])

        #p.resetDebugVisualizerCamera(2.5, -0.2, -0, [2.5, 0.0, 0.3], physicsClientId=self.physicsClient)
        #input()

    def render_camera_image(self, camera_params, gimbal_camera=False):

        if self.FIXED_DEPTH_IMAGE is not None:
            # wait for next camera image
            while len(self.FIXED_DEPTH_IMAGE) == 0:
                time.sleep(0.001)

            ret = self.FIXED_DEPTH_IMAGE[:], None
            self.FIXED_DEPTH_IMAGE = []
            return ret

        return self.render_camera_from_pb(camera_params, gimbal_camera)

    def render_camera_from_pb(self, camera_params, gimbal_camera=False):
        if gimbal_camera:
            self.realsense_camera.setup_camera(focus_pt=self.ob.body_pos + camera_params.pose,
                                               dist = 0.0001,
                                               yaw = (self.ob.body_rpy[2] - np.pi/2 + camera_params.rpy[2])*180/np.pi,
                                               pitch = (camera_params.rpy[1])*180/np.pi,
                                               roll = (camera_params.rpy[0])*180/np.pi,
                                               height = camera_params.height,
                                               width = camera_params.width,
                                               aspect = camera_params.aspect,
                                               fov = camera_params.fov,
                                               znear = camera_params.nearVal,
                                               zfar = camera_params.farVal)
        else:
            cam_rpy = [self.ob.body_rpy[0], self.ob.body_rpy[1], self.ob.body_rpy[2]] + (np.random.random((3,)) * 2 - 1) * camera_params.cam_rpy_std
            cam_rot = get_rotation_matrix_from_quaternion(get_quaternion_from_rpy(cam_rpy))
            cam_trans = cam_rot.dot(camera_params.pose) + (np.random.random((3,)) * 2 - 1) * camera_params.cam_pose_std
            self.realsense_camera.setup_camera(focus_pt=self.ob.body_pos + cam_trans,
                                               dist = 0.001,
                                               yaw = (self.ob.body_rpy[2] - np.pi/2 + camera_params.rpy[2])*180/np.pi,
                                               pitch = (-1 * self.ob.body_rpy[1] + camera_params.rpy[1]+0.03)*180/np.pi,
                                               roll = (self.ob.body_rpy[0] + camera_params.rpy[0])*180/np.pi,
                                               height = camera_params.height,
                                               width = camera_params.width,
                                               aspect = camera_params.aspect,
                                               fov = camera_params.fov,
                                               znear = camera_params.nearVal,
                                               zfar = camera_params.farVal)

        ret = self.realsense_camera.get_frames()
        depth, rgb = ret[0], ret[1]
        
        self.rgb, self.depth = rgb, depth

        if self.lcm_publisher is not None:
            self.lcm_publisher.broadcast_depth_image_lcm(self.depth, self.ob, self.step_counter)

        return depth, rgb

    def close(self):
        p.disconnect()
