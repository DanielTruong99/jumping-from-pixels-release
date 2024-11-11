import numpy as np
import scipy
import time
import os

from cheetahgym.controllers.mpc_force_controller import MPCForceController
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion

#from cheetahgym.data_types.low_level_types import LowLevelCmd
#from cheetahgym.data_types.wbc_level_types import WBCLevelCmd

from qpoases import PyQProblem as QProblem
from qpoases import PyBooleanType as BooleanType
from qpoases import PySubjectToStatus as SubjectToStatus
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel



class MPCForceControllerPy(MPCForceController):
    def __init__(self, dt):
        super().__init__(dt=dt)

        N_BIG_NUMBER = 5e10
        weights = [0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2] # tunable gains
        mu = 0.4
        horizon = 10
        mass = 9. # kg

        self.I_body = np.array(  [[0.07,     0,      0],
                                 [0,        0.26,   0],
                                 [0,        0,      0.242]] )

        self.A = np.zeros((13, 13))
        self.B = np.zeros((13, 12))
        self.A[3, 9] = 1
        self.A[9, 9] = 0 # x_drag?
        self.A[4, 10] = 1
        self.A[5, 11] = 1
        self.A[11, 12] = 1
        for b in range(4):
            self.B[9:12, b*3:b*3+3] = np.eye(3) / mass


        self.A_qp = np.zeros((13*horizon, 13))
        self.B_qp = np.zeros((13*horizon, 12*horizon))

        self.ABc = np.zeros((25, 25))

        self.U_b = np.zeros(5*horizon*4)
        self.fmat = np.zeros((5*horizon*4, 3*horizon*4))

        for i in range(horizon):
            for j in range(4):
                self.U_b[i*4*5 + j*5:i*4*5 + j*5 + 4] = N_BIG_NUMBER

        mu_inv = 1./mu
        f_block = np.array([[1./mu, 0, 1.],
                            [-1./mu, 0, 1.],
                            [0, 1./mu, 1.],
                            [0, -1./mu, 1.],
                            [0, 0, 1.]])

        for i in range(horizon*4):
            self.fmat[i*5:(i*5+5), i*3:(i*3+3)] = f_block

        full_weight = np.concatenate((weights, np.array([0])))
        self.S = np.diag(np.repeat(full_weight, horizon)) # check size of this

        self.X_d = np.zeros((horizon*13, 1))

        self.lb = np.zeros(20*horizon)

    def solve_forces(self, low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations):

        dt = self.dt
        f_max = 120
        mass = 9. # kg

        alpha = 4e-5

        yaw = low_level_state.body_rpy[2]
        p = low_level_state.body_pos # body position
        v = low_level_state.body_linear_vel # world-frame body vel
        w = low_level_state.body_angular_vel # world-frame body rotational vel
        q = get_quaternion_from_rpy(low_level_state.body_rpy) # body orientation
        rpy = low_level_state.body_rpy

        r = np.zeros(12) # relative foot locations (extremely approximate..)
        for i in range(4):
            r[3*i:3*i+3] = foot_locations[i] - p

        dtMPClist = iters_list * dt

        # neural_setup_problem

        # update_x_drag

        # neural_update_problem_data_floats(p,v,q,w,r,yaw,weights,trajAll,alpha,mpcTable)

        x0 = np.array([rpy[2], rpy[1], rpy[0], p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], -9.8]).reshape(-1, 1)
        R_yaw = np.array(  [[np.cos(yaw), -np.sin(yaw), 0.],
                            [np.sin(yaw), np.cos(yaw), 0,],
                            [0, 0, 1]])
        

        I_world = np.dot(R_yaw, np.dot(self.I_body, R_yaw.T)) # compute world-frame inertia

        ## neural_ct_ss_mats

        self.A[0:3, 6:9] = R_yaw.T

        I_inv = np.linalg.inv(I_world)
        for b in range(4):
            self.B[6:9, b*3:b*3+3] = np.cross(I_inv, r[b*3:b*3+3])

        ## neural_c2qp
        horizon = 10 # planning horizon!
        
        self.ABc[:13, :13] = dt * self.A
        self.ABc[:13, 13:] = dt * self.B

        expmm = scipy.linalg.expm(self.ABc) # check dimension
        Adt = expmm[:13, :13]
        Bdt = expmm[:13, 13:]

        powerMats = [np.eye(13) for i in range(horizon+1)]
        for i in range(1, horizon+1):
            powerMats[i] = np.dot(Adt, powerMats[i-1])

        for r in range(horizon):
            self.A_qp[13*r:13*r+13, :] = powerMats[r+1]
            for c in range(horizon):
                if r >= c:
                    a_num = r-c
                    self.B_qp[13*r:13*r+13, 12*c:12*c+12] = np.dot(powerMats[a_num], Bdt)


        

        B_qp_trans_S = np.dot(self.B_qp.T, self.S)

        for i in range(horizon):
            for j in range(12):
                self.X_d[13*i+j, 0] = trajAll[12*i+j]

        for i in range(horizon):
            for j in range(4):
                self.U_b[i*4*5 + j*5 + 4] = mpc_table[j, i] * f_max # need to flatten mpc_table first?

        # construct optimization problem
        # these are some of the slowest operations (in cpp)

        qH = 2 * (np.dot(B_qp_trans_S, self.B_qp) + alpha * np.eye(12*horizon))
        qg = 2 * np.dot(B_qp_trans_S, np.dot(self.A_qp, x0) - self.X_d)
        lb = self.lb
        ub = self.U_b
        qA = self.fmat


        # pass the problem to qpOASES
        n_vars = qH.shape[0]
        n_cons = len(lb)

        problem = QProblem(n_vars, n_cons)
        options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.NONE
        problem.setOptions(options)

        nWSR = 100
        problem.init(qH, qg.flatten(), qA, None, None, lb, ub, nWSR)

        xOpt = np.zeros(n_vars)
        problem.getPrimalSolution(xOpt)
        objval = problem.getObjVal()

        Fr_des = xOpt[:12]
        
        return Fr_des

    def reset(self):
        pass