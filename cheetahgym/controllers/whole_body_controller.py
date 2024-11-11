import numpy as np
import scipy
import time
import os

import pathlib

import pycheetah
from pycheetah import *

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion

from cheetahgym.data_types.low_level_types import LowLevelCmd
from cheetahgym.data_types.wbc_level_types import WBCLevelCmd


class WholeBodyController:
    def __init__(self, dt=0.002):

        robot_filename = f"{pathlib.Path(__file__).parent.parent.absolute()}/config/mini-cheetah-defaults.yaml"
        user_filename = f"{pathlib.Path(__file__).parent.parent.absolute()}/config/mc-mit-ctrl-user-parameters.yaml"

        print(robot_filename)

        # define structs
        self.quadruped_params, self.estimator_params = None, None
        self.cheetah, self.model = None, None
        #self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate = CheaterState(), VectorNavData(), LegControllerData(), StateEstimate()
        #self.stateEstimator = StateEstimatorContainer(self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate, self.quadruped_params)

        self.fsm, self.vizData = None, None

        self.dt = dt

        self.low_level_command = LowLevelCmd()

        # initialization
        self._load_params(robot_filename, user_filename)
        self._make_model()
        self._make_state_estimator()
        self._make_fsm()

        self.p_targets, self.v_targets, self.p_gains, self.v_gains, self.generalized_forces = np.zeros(19), np.zeros(18), np.zeros(18), np.zeros(18), np.zeros(18)

        self.wbc = LocomotionCtrl(self.model)
        self.wbc_data = LocomotionCtrlData()

    def optimize_targets_whole_body(self, wbc_level_cmd, low_level_state, rot_w_b, swap_legs=True):

        base_orientation = get_quaternion_from_rpy(low_level_state.body_rpy)
        base_pos = low_level_state.body_pos
        base_omega = rot_w_b.dot(low_level_state.body_angular_vel)
        base_vel = rot_w_b.dot(low_level_state.body_linear_vel)
        base_accel = np.zeros(3)

        self._set_cartesian_state(np.concatenate((base_orientation, base_pos, base_omega, base_vel, base_accel)), rot_w_b)

        if swap_legs:
            q = np.concatenate((low_level_state.joint_pos[3:6], low_level_state.joint_pos[0:3], low_level_state.joint_pos[9:12], low_level_state.joint_pos[6:9]))
            qd = np.concatenate((low_level_state.joint_vel[3:6], low_level_state.joint_vel[0:3], low_level_state.joint_vel[9:12], low_level_state.joint_vel[6:9]))
        else:
            q = low_level_state.joint_pos
            qd = low_level_state.joint_vel

        self._set_joint_state(q, qd)
        
        # trajectory generator
        pBody_des = wbc_level_cmd.pBody_des
        vBody_des = wbc_level_cmd.vBody_des
        aBody_des = wbc_level_cmd.aBody_des
        pBody_RPY_des = wbc_level_cmd.pBody_RPY_des
        vBody_Ori_des = wbc_level_cmd.vBody_Ori_des
        pFoot_des = wbc_level_cmd.pFoot_des
        vFoot_des = wbc_level_cmd.vFoot_des
        aFoot_des = wbc_level_cmd.aFoot_des
        contact_state = wbc_level_cmd.contact_state
        # mpc's work
        Fr_des = wbc_level_cmd.Fr_des


        self.wbc_data.setBodyDes(pBody_des, vBody_des, aBody_des, pBody_RPY_des, vBody_Ori_des)
        if swap_legs:
            foot_mapping = {0:1, 1:0, 2:3, 3:2}
        else:
            foot_mapping = {0:0, 1:1, 2:2, 3:3}
        for i in range(4):
            self.wbc_data.setFootDes(foot_mapping[i], pFoot_des[i*3:i*3+3], vFoot_des[i*3:i*3+3], aFoot_des[i*3:i*3+3], Fr_des[i*3:i*3+3])
        
        if swap_legs:
            self.wbc_data.setContactState(contact_state[[1, 0, 3, 2]])
        else:
            self.wbc_data.setContactState(contact_state)
        
        self.wbc.run(self.wbc_data, self.fsm.data)

        tauff, forceff, qDes, qdDes, pDes, vDes, kpCartesian, kdCartesian, kpJoint, kdJoint = self._get_joint_commands()
        
        qDes = np.concatenate((qDes[3:6], qDes[0:3], qDes[9:12], qDes[6:9]))
        qdDes = np.concatenate((qdDes[3:6], qdDes[0:3], qdDes[9:12], qdDes[6:9]))
        tauff = np.concatenate((tauff[3:6], tauff[0:3], tauff[9:12], tauff[6:9]))


        # action scaling
        p_targets, v_targets, p_gains, v_gains, generalized_forces = np.zeros(19), np.zeros(18), np.zeros(18), np.zeros(18), np.zeros(18)
        p_targets[-12:] = qDes
        v_targets[-12:] = qdDes
        generalized_forces[-12:] = tauff
        p_gains[-12:] = np.array([kpJoint[i, i%3] for i in range(len(kpJoint))])
        v_gains[-12:] = np.array([kdJoint[i, i%3] for i in range(len(kdJoint))])

        low_level_command = LowLevelCmd()
        low_level_command.from_vec(np.concatenate((p_targets, v_targets, p_gains, v_gains, generalized_forces)))


        return low_level_command


    def _load_params(self, robot_filename, user_filename):
        self.quadruped_params = RobotControlParameters()
        self.estimator_params = MIT_UserParameters()
        # load parameters
        self.quadruped_params.initializeFromYamlFile(robot_filename)
        self.estimator_params.initializeFromYamlFile(user_filename)
        
    def _make_model(self):
        self.cheetah = buildMiniCheetah()
        self.model = self.cheetah.buildModel()

    def _make_state_estimator(self):
        self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate = CheaterState(), VectorNavData(), LegControllerData(), StateEstimate()
        self.legControllerData.setQuadruped(self.cheetah)
        self.stateEstimator = StateEstimatorContainer(self.cheaterState, self.vnavData, self.legControllerData, self.stateEstimate, self.quadruped_params)
        self.stateEstimator.initializeCheater()
        #self.stateEstimator.initializeRegular()

    def _make_fsm(self):
        self.gamepadCmd, self.rc_command = GamepadCommand(), rc_control_settings()
        self.desiredStateCmd =  DesiredStateCommand(self.gamepadCmd, self.rc_command, self.quadruped_params, self.stateEstimate, self.dt)

        self.gaitScheduler = GaitScheduler(self.estimator_params, self.dt)
        self.legController = LegController(self.cheetah)
        self.legController.zeroCommand()
        self.legController.setEnabled(True)

        self.vizData = VisualizationData()
        self.fsm = ControlFSM(self.cheetah, self.stateEstimator, self.legController, self.gaitScheduler, self.desiredStateCmd, self.quadruped_params, self.vizData, self.estimator_params)
        self.fsm.initialize()
        self.fsm.runFSM()

    def _set_cartesian_state(self, cartesian_state, rot_w_b):
        self.cheaterState.orientation = cartesian_state[0:4]
        self.cheaterState.position = cartesian_state[4:7]
        self.cheaterState.omegaBody = rot_w_b.dot(cartesian_state[7:10])
        self.cheaterState.vBody = rot_w_b.dot(cartesian_state[10:13])
        self.cheaterState.acceleration = cartesian_state[13:16]

        self.stateEstimator.run()

    def _set_joint_state(self, joint_state, d_joint_state):
        spiData = SpiData()
        for idx in range(4):
            q_abad, q_hip, q_knee = joint_state[3 * idx], joint_state[3 * idx + 1], joint_state[3 * idx + 2]
            qd_abad, qd_hip, qd_knee = d_joint_state[3 * idx], d_joint_state[3 * idx + 1], d_joint_state[3 * idx + 2]
            spiData.setLegData(idx, q_abad, q_hip, q_knee, qd_abad, qd_hip, qd_knee)
        self.legController.updateData(spiData)

    def _get_joint_commands(self):
        commands = [self.legController.getCommands(i) for i in range(4)]

        tauff = np.concatenate(([cmd.tauFeedForward for cmd in commands]))
        forceff = np.concatenate(([cmd.forceFeedForward for cmd in commands]))
        qDes = np.concatenate(([cmd.qDes for cmd in commands]))
        qdDes = np.concatenate(([cmd.qdDes for cmd in commands]))
        pDes = np.concatenate(([cmd.pDes for cmd in commands]))
        vDes = np.concatenate(([cmd.vDes for cmd in commands]))

        kpCartesian = np.concatenate(([cmd.kpCartesian for cmd in commands]))
        kdCartesian = np.concatenate(([cmd.kdCartesian for cmd in commands]))
        kpJoint = np.concatenate(([cmd.kpJoint for cmd in commands]))
        kdJoint = np.concatenate(([cmd.kdJoint for cmd in commands]))

    
        return tauff, forceff, qDes, qdDes, pDes, vDes, kpCartesian, kdCartesian, kpJoint, kdJoint

    def reset(self):
        pass
