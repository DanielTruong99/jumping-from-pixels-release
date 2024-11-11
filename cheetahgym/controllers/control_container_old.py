import numpy as np

#from cheetahgym.controllers.trajectory_generator import TrajectoryGenerator
from cheetahgym.controllers.whole_body_controller import WholeBodyController
from cheetahgym.controllers.mpc_force_controller_py import MPCForceControllerPy
from cheetahgym.controllers.mpc_force_controller_cpp import MPCForceControllerCpp

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion
from cheetahgym.data_types.wbc_level_types import WBCLevelCmd


class ControlContainer:
    def __init__(self, dt = 0.002, iterationsBetweenMPC = 13):

        self.dt = dt
        self.iterationsBetweenMPC = iterationsBetweenMPC
        self.wbc_step_counter = 0

        # dummy for this implementation
        self.mpc_objective_value = 0
        self.wbc_objective_value = 0

        self.reverse_legs = True

        # initialize control layers
        #self.trajectory_generator = TrajectoryGenerator()
        self.whole_body_controller = WholeBodyController( dt = self.dt )
        # self.mpc_force_controller = MPCForceControllerPy( dt = self.dt * self.iterationsBetweenMPC )
        self.mpc_force_controller = MPCForceControllerCpp( dt = self.dt * self.iterationsBetweenMPC, whole_body_controller = self.whole_body_controller )

    def step_with_mpc_table(self, mpc_level_cmd_tabular, mpc_level_state, low_level_state, rot_w_b, foot_locations=None, residual_forces=None, override_forces=None):

        base_orientation = get_quaternion_from_rpy(low_level_state.body_rpy)
        base_pos = low_level_state.body_pos

        #if not (self.cfg is None) and not self.cfg.observe_corrected_vel:
        base_omega = rot_w_b.dot(low_level_state.body_angular_vel)
        base_vel = rot_w_b.dot(low_level_state.body_linear_vel)
        # else:
        #     base_omega = low_level_state.body_angular_vel
        #     base_vel = low_level_state.body_linear_vel

        base_accel = np.zeros(3)

        self.whole_body_controller._set_cartesian_state(np.concatenate((base_orientation, base_pos, base_omega, base_vel, base_accel)), rot_w_b)

        if self.reverse_legs:
            q = low_level_state.joint_pos[[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]]
            qd = low_level_state.joint_vel[[3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8]]
        else:
            q = low_level_state.joint_pos[0:12]
            qd = low_level_state.joint_vel[0:12]
     
        self.whole_body_controller._set_joint_state(q, qd)
        
        #if not (self.cfg is None) and not self.cfg.binary_contact_actions and not self.cfg.symmetry_contact_actions and not self.cfg.pronk_actions:
        #    mpc_level_cmd_tabular.mpc_table_update = build_mpc_table_from_params(mpc_level_cmd_tabular.offsets_smoothed, mpc_level_cmd_tabular.durations_smoothed, cycleLength=10)
        mpc_level_cmd_tabular.set_vel(mpc_level_cmd_tabular.vel_cmd)
        mpc_level_cmd_tabular.set_vel_rpy(mpc_level_cmd_tabular.vel_rpy_cmd)

        mpc_level_cmd_tabular.set_iterations(mpc_level_cmd_tabular.iterationsBetweenMPC)

        nominal_target_ys = [-0.1, 0.1, -0.1, 0.1]

        if self.reverse_legs:
            foot_mapping = {0:1, 1:0, 2:3, 3:2}
        else:
            foot_mapping = {0:0, 1:1, 2:2, 3:3}
        if foot_locations is not None:
            for idx in range(4):
                self.mpc_force_controller.nmpc.set_pFoot_true(foot_mapping[idx], foot_locations[idx])

        self.mpc_force_controller.nmpc.run(self.whole_body_controller.fsm.data, mpc_level_cmd_tabular.vel_cmd, mpc_level_cmd_tabular.vel_rpy_cmd, mpc_level_cmd_tabular.fp_rel_cmd, 
             mpc_level_cmd_tabular.fh_rel_cmd, mpc_level_cmd_tabular.footswing_height, mpc_level_cmd_tabular.iterationsBetweenMPC, 
             mpc_level_cmd_tabular.mpc_table_update, mpc_level_cmd_tabular.vel_table_update, 
             mpc_level_cmd_tabular.vel_rpy_table_update, mpc_level_cmd_tabular.iterations_table_update, 
             mpc_level_cmd_tabular.planningHorizon, mpc_level_cmd_tabular.adaptationHorizon, mpc_level_cmd_tabular.adaptationSteps)
        
        self.mpc_progress = self.mpc_force_controller.nmpc.iterationCounter//self.mpc_force_controller.nmpc.iterationsBetweenMPC % 10 # amount of MPC iterations through cycle
        self.mpc_objective_value = self.mpc_force_controller.nmpc.objVal

        wbc_cmd = WBCLevelCmd()
        wbc_cmd.pBody_des = self.mpc_force_controller.nmpc.pBody_des
        wbc_cmd.vBody_des = self.mpc_force_controller.nmpc.vBody_des
        wbc_cmd.aBody_des = self.mpc_force_controller.nmpc.aBody_des
        wbc_cmd.pBody_RPY_des = self.mpc_force_controller.nmpc.pBody_RPY_des
        wbc_cmd.vBody_Ori_des = self.mpc_force_controller.nmpc.vBody_Ori_des
        wbc_cmd.pFoot_des = np.concatenate([self.mpc_force_controller.nmpc.get_pFoot_des(foot_mapping[i]) for i in range(4)])
        wbc_cmd.vFoot_des = np.concatenate([self.mpc_force_controller.nmpc.get_vFoot_des(foot_mapping[i]) for i in range(4)])
        wbc_cmd.aFoot_des = np.concatenate([self.mpc_force_controller.nmpc.get_aFoot_des(foot_mapping[i]) for i in range(4)])
        wbc_cmd.contact_state = self.mpc_force_controller.nmpc.contact_state[[foot_mapping[i] for i in range(4)]]
        wbc_cmd.Fr_des = np.concatenate([self.mpc_force_controller.nmpc.get_Fr_des(foot_mapping[i]) for i in range(4)])
 
        self.low_level_command = self.whole_body_controller.optimize_targets_whole_body(wbc_cmd, low_level_state, rot_w_b, swap_legs=self.reverse_legs)

        return self.low_level_command

    def reset_ctrl(self):
        self.mpc_force_controller.reset()
        self.whole_body_controller.reset()

    def set_gains(self, kpJoint, kdJoint):
        self.kpJoint, self.kdJoint = kpJoint, kdJoint
        #self.quadruped_params.initializeVec3d("Kp_joint", kpJoint)
        #self.quadruped_params.initializeVec3d("Kd_joint", kdJoint)
        #print("DID NOT SET GAINS -- NOT IMPLEMENTED")

    def getIterationsToNextUpdate(self, adaptation_steps):
        return adaptation_steps * self.iterationsBetweenMPC
    
    def set_accept_update(self):
        if isinstance(self.mpc_force_controller, MPCForceControllerCpp):
            self.mpc_force_controller.nmpc.set_accept_update()

    def getContactState(self):
        if isinstance(self.mpc_force_controller, MPCForceControllerCpp):
            return self.mpc_force_controller.nmpc.neuralGait.getContactState()
        return np.ones(4)