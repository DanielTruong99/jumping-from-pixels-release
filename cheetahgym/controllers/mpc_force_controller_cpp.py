import numpy as np
import scipy
import time
import os

import pycheetah
from pycheetah import NeuralMPCLocomotion

from cheetahgym.controllers.mpc_force_controller import MPCForceController
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion


class MPCForceControllerCpp(MPCForceController):
    def __init__(self, dt, whole_body_controller=None):
        if whole_body_controller is None:
            print("ERROR: must pass whole-body controller to access Cpp bindings efficiently!")
            return
        self.whole_body_controller = whole_body_controller
        super().__init__(dt=dt)

        self.nmpc = NeuralMPCLocomotion(whole_body_controller.dt, int(self.dt / whole_body_controller.dt), whole_body_controller.estimator_params)
        self.nmpc.initialize()

    def solve_forces(self, low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations):

        base_orientation = get_quaternion_from_rpy(low_level_state.body_rpy)
        base_pos = low_level_state.body_pos
        base_omega = rot_w_b.dot(low_level_state.body_angular_vel)
        base_vel = rot_w_b.dot(low_level_state.body_linear_vel)
        base_accel = np.zeros(3)

        self.whole_body_controller._set_cartesian_state(np.concatenate((base_orientation, base_pos, base_omega, base_vel, base_accel)), rot_w_b)

        q = np.concatenate((low_level_state.joint_pos[3:6], low_level_state.joint_pos[0:3], low_level_state.joint_pos[9:12], low_level_state.joint_pos[6:9]))
        qd = np.concatenate((low_level_state.joint_vel[3:6], low_level_state.joint_vel[0:3], low_level_state.joint_vel[9:12], low_level_state.joint_vel[6:9]))
        
        self.whole_body_controller._set_joint_state(q, qd)

        self.nmpc.set_horizon(10)

        self.nmpc.pBody_des = wbc_level_cmd.pBody_des
        self.nmpc.vBody_des = wbc_level_cmd.vBody_des
        self.nmpc.aBody_des = wbc_level_cmd.aBody_des
        self.nmpc.pBody_RPY_des = wbc_level_cmd.pBody_RPY_des
        self.nmpc.vBody_Ori_des = wbc_level_cmd.vBody_Ori_des
        foot_mapping = {0:0, 1:1, 2:2, 3:3}
        for idx in range(4):
            self.nmpc.set_pFoot_des(foot_mapping[idx], wbc_level_cmd.pFoot_des[3*idx:3*idx+3])
            self.nmpc.set_vFoot_des(foot_mapping[idx], wbc_level_cmd.vFoot_des[3*idx:3*idx+3])
            self.nmpc.set_aFoot_des(foot_mapping[idx], wbc_level_cmd.aFoot_des[3*idx:3*idx+3])
            self.nmpc.set_pFoot_true(foot_mapping[idx], foot_locations[idx])

        self.nmpc.contact_state = wbc_level_cmd.contact_state #np.array([wbc_level_cmd.contact_state[0], wbc_level_cmd.contact_state[1], wbc_level_cmd.contact_state[2], wbc_level_cmd.contact_state[3]])


        trajAllIn = np.zeros(12*36)
        trajAllIn[:len(trajAll)] = trajAll

        self.nmpc.solve_dense_mpc(mpc_table.T.flatten(), iters_list, self.whole_body_controller.fsm.data, trajAllIn)
        self.mpc_objective_value = self.nmpc.objVal

        foot_mapping = {0:0, 1:1, 2:2, 3:3}

        Fr_des = np.zeros(12)

        for idx in range(4):
            Fr_des[3*idx:3*idx+3] = self.nmpc.get_Fr_des(foot_mapping[idx]) * 1

        return Fr_des

    def reset(self):
        self.nmpc.initialize()
        self.nmpc.iterationCounter = 0
