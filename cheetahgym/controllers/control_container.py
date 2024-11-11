import numpy as np

from cheetahgym.controllers.trajectory_generator import TrajectoryGenerator
from cheetahgym.controllers.whole_body_controller import WholeBodyController
from cheetahgym.controllers.mpc_force_controller_py import MPCForceControllerPy
from cheetahgym.controllers.mpc_force_controller_cpp import MPCForceControllerCpp


class ControlContainer:
    def __init__(self, dt = 0.002, iterationsBetweenMPC = 13):

        self.dt = dt
        self.iterationsBetweenMPC = iterationsBetweenMPC
        self.wbc_step_counter = 0

        # dummy for this implementation
        self.mpc_objective_value = 0
        self.wbc_objective_value = 0

        # initialize control layers
        self.trajectory_generator = TrajectoryGenerator()
        self.whole_body_controller = WholeBodyController( dt = self.dt )
        # self.mpc_force_controller = MPCForceControllerPy( dt = self.dt * self.iterationsBetweenMPC )
        self.mpc_force_controller = MPCForceControllerCpp( dt = self.dt * self.iterationsBetweenMPC, whole_body_controller = self.whole_body_controller )

    def step_with_mpc_table(self, mpc_level_cmd_tabular, mpc_level_state, low_level_state, rot_w_b, foot_locations=None, residual_forces=None, override_forces=None):

        
        #if self.cfg is None or (not (self.cfg is None) and not self.cfg.binary_contact_actions and not self.cfg.symmetry_contact_actions and not self.cfg.pronk_actions):
        #mpc_level_cmd_tabular.mpc_table_update = build_mpc_table_from_params(mpc_level_cmd_tabular.offsets_smoothed, mpc_level_cmd_tabular.durations_smoothed, cycleLength=10)
        mpc_level_cmd_tabular.set_vel(mpc_level_cmd_tabular.vel_cmd)
        mpc_level_cmd_tabular.set_vel_rpy(mpc_level_cmd_tabular.vel_rpy_cmd)
        mpc_level_cmd_tabular.set_iterations(mpc_level_cmd_tabular.iterationsBetweenMPC)

        self.trajectory_generator.update(   offset=0, 
                                            adaptation_steps=mpc_level_cmd_tabular.adaptationSteps,
                                            contact_table=mpc_level_cmd_tabular.mpc_table_update.reshape((-1, 4))[:, [1, 0, 3, 2]].T, 
                                            vel_table=mpc_level_cmd_tabular.vel_table_update.reshape((-1, 3)).T, 
                                            vel_rpy_table=mpc_level_cmd_tabular.vel_rpy_table_update.reshape((-1, 3)).T, 
                                            iteration_table=mpc_level_cmd_tabular.iterations_table_update.reshape((-1, 1)).T
                                        )

        self.trajectory_generator.step(np.array(foot_locations), low_level_state)
        trajAll = self.trajectory_generator.get_traj(low_level_state)
        wbc_cmd = self.trajectory_generator.to_wbc_cmd(low_level_state)
        mpc_table = self.trajectory_generator.get_mpc_table()
        iters_list = np.ones(10) * self.iterationsBetweenMPC


        if self.wbc_step_counter % self.iterationsBetweenMPC == 0:
            self.Fr = self.mpc_force_controller.solve_forces(low_level_state, rot_w_b, wbc_cmd, mpc_table, iters_list, trajAll, foot_locations)
        wbc_cmd.Fr_des = self.Fr

        self.low_level_command = self.whole_body_controller.optimize_targets_whole_body(wbc_cmd, low_level_state, rot_w_b)

        self.wbc_step_counter += 1

        return self.low_level_command

    def reset_ctrl(self):
        self.mpc_force_controller.reset()
        self.whole_body_controller.reset()
        self.trajectory_generator.reset()

    def set_gains(self, kpJoint, kdJoint):
        self.kpJoint, self.kdJoint = kpJoint, kdJoint
        #self.quadruped_params.initializeVec3d("Kp_joint", kpJoint)
        #self.quadruped_params.initializeVec3d("Kd_joint", kdJoint)
        print("DID NOT SET GAINS -- NOT IMPLEMENTED")

    def getIterationsToNextUpdate(self, adaptation_steps):
        return adaptation_steps * self.iterationsBetweenMPC
    
    def set_accept_update(self):
        if isinstance(self.mpc_force_controller, MPCForceControllerCpp):
            self.mpc_force_controller.nmpc.set_accept_update()

    def getContactState(self):
        if isinstance(self.mpc_force_controller, MPCForceControllerCpp):
            return self.mpc_force_controller.nmpc.neuralGait.getContactState()
        return np.ones(4)