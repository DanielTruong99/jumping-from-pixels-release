cimport cython
import numpy as np
cimport numpy as np
import scipy
import time

from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion
from cheetahgym.utils.gait_controller import build_mpc_table_from_params

from cheetahgym.data_types.low_level_types import LowLevelCmd
from cheetahgym.data_types.wbc_level_types import WBCLevelCmd


from qpoases import PyQProblem as QProblem
from qpoases import PyBooleanType as BooleanType
from qpoases import PySubjectToStatus as SubjectToStatus
from qpoases import PyOptions as Options
from qpoases import PyPrintLevel as PrintLevel



cdef class MPCOptimizer():

    cdef np.float64_t[:, :] qH
    cdef np.float64_t[:, :] qA
    cdef np.float64_t[:, :] qg
    cdef np.float64_t[:] lb
    cdef np.float64_t[:] ub
    cdef np.float64_t[:] lbA
    cdef np.float64_t[:] ubA

    cdef np.float64_t[:, :] A
    cdef np.float64_t[:, :] B
    cdef np.float64_t[:, :] A_qp
    cdef np.float64_t[:, :] B_qp
    cdef np.float64_t[:, :] ABc
    cdef np.float64_t[:, :] Adt
    cdef np.float64_t[:, :] Bdt


    cdef np.float64_t[:, :] S
    cdef np.float64_t[:] U_b
    cdef np.float64_t[:, :] X_d


    cdef mpc_opt(self, cfg, low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations):

        cdef np.ndarray[np.double_t, ndim=2] qH
        cdef np.ndarray[np.double_t, ndim=2] qA
        cdef np.ndarray[np.double_t, ndim=2] qg
        cdef np.ndarray[np.double_t, ndim=1] lb
        cdef np.ndarray[np.double_t, ndim=1] ub
        cdef np.ndarray[np.double_t, ndim=1] lbA
        cdef np.ndarray[np.double_t, ndim=1] ubA

        cdef np.ndarray[np.double_t, ndim=2] A
        cdef np.ndarray[np.double_t, ndim=2] B
        cdef np.ndarray[np.double_t, ndim=2] A_qp
        cdef np.ndarray[np.double_t, ndim=2] B_qp
        cdef np.ndarray[np.double_t, ndim=2] ABc
        cdef np.ndarray[np.double_t, ndim=2] Adt
        cdef np.ndarray[np.double_t, ndim=2] Bdt

        cdef np.ndarray[np.double_t, ndim=2] S
        cdef np.ndarray[np.double_t, ndim=1] U_b
        cdef np.ndarray[np.double_t, ndim=2] X_d



        timer = time.time()

        base_orientation = get_quaternion_from_rpy(low_level_state.body_rpy)
        base_pos = low_level_state.body_pos
        if not (cfg is None) and not cfg.observe_corrected_vel:
            base_omega = rot_w_b.dot(low_level_state.body_angular_vel)
            base_vel = rot_w_b.dot(low_level_state.body_linear_vel)
        else:
            base_omega = low_level_state.body_angular_vel
            base_vel = low_level_state.body_linear_vel
        
        base_accel = np.zeros(3)#mpc_level_state.body_linear_accel
        
        swap_legs=True
        if swap_legs:
            q = np.concatenate((low_level_state.joint_pos[3:6], low_level_state.joint_pos[0:3], low_level_state.joint_pos[9:12], low_level_state.joint_pos[6:9]))
            qd = np.concatenate((low_level_state.joint_vel[3:6], low_level_state.joint_vel[0:3], low_level_state.joint_vel[9:12], low_level_state.joint_vel[6:9]))
        else:
            q = low_level_state.joint_pos
            qd = low_level_state.joint_vel

        print(f"Time {time.time()-timer}")

        dt = 0.026
        N_BIG_NUMBER = 5e10
        f_max = 120
        mu = 0.4
        mass = 9. # kg

        weights = [0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2] # tunable gains
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

        print(f"Time {time.time()-timer}")

        x0 = np.array([rpy[2], rpy[1], rpy[0], p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], -9.8]).reshape(-1, 1)
        R_yaw = np.array(  [[np.cos(yaw), -np.sin(yaw), 0.],
                            [np.sin(yaw), np.cos(yaw), 0,],
                            [0, 0, 1]])
        I_body = np.array(  [[0.07,     0,      0],
                             [0,        0.26,   0],
                             [0,        0,      0.242]] )

        I_world = np.dot(R_yaw, np.dot(I_body, R_yaw.T)) # compute world-frame inertia

        ## neural_ct_ss_mats
        A = np.zeros((13, 13))
        B = np.zeros((13, 12))

        A[3, 9] = 1
        A[9, 9] = 0 # x_drag?
        A[4, 10] = 1
        A[5, 11] = 1
        A[11, 12] = 1
        A[0:3, 6:9] = R_yaw.T

        I_inv = np.linalg.inv(I_world)
        for b in range(4):
            B[6:9, b*3:b*3+3] = np.cross(I_inv, r[b*3:b*3+3])
            B[9:12, b*3:b*3+3] = np.eye(3) / mass

        ## neural_c2qp
        horizon = 10 # planning horizon!
        A_qp = np.zeros((13*horizon, 13))
        B_qp = np.zeros((13*horizon, 12*horizon))

        ABc = np.zeros((25, 25))
        ABc[:13, :13] = dt * A
        ABc[:13, 13:] = dt * B
        #print("ABc", ABc)

        print(f"Time {time.time()-timer}")

        expmm = scipy.linalg.expm(ABc) # check dimension
        Adt = expmm[:13, :13]
        Bdt = expmm[:13, 13:]

        # print(np.max(expmm))

        powerMats = [np.eye(13) for i in range(horizon+1)]
        for i in range(1, horizon+1):
            powerMats[i] = np.dot(Adt, powerMats[i-1])

        for r in range(horizon):
            A_qp[13*r:13*r+13, :] = powerMats[r+1]
            for c in range(horizon):
                if r >= c:
                    a_num = r-c
                    B_qp[13*r:13*r+13, 12*c:12*c+12] = np.dot(powerMats[a_num], Bdt)


        full_weight = np.concatenate((weights, np.array([0])))
        S = np.diag(np.repeat(full_weight, horizon)) # check size of this


        B_qp_trans_S = np.dot(B_qp.T, S)

        X_d = np.zeros((horizon*13, 1))
        for i in range(horizon):
            for j in range(12):
                X_d[13*i+j, 0] = trajAll[12*i+j]

        U_b = np.zeros(5*horizon*4)

        for i in range(horizon):
            for j in range(4):
                U_b[i*4*5 + j*5:i*4*5 + j*5 + 4] = N_BIG_NUMBER
                U_b[i*4*5 + j*5 + 4] = mpc_table[j, i] * f_max # need to flatten mpc_table first?

        mu_inv = 1./mu
        fmat = np.zeros((5*horizon*4, 3*horizon*4))
        f_block = np.array([[1./mu, 0, 1.],
                            [-1./mu, 0, 1.],
                            [0, 1./mu, 1.],
                            [0, -1./mu, 1.],
                            [0, 0, 1.]])

        for i in range(horizon*4):
            fmat[i*5:(i*5+5), i*3:(i*3+3)] = f_block

        print(f"Time {time.time()-timer}")


        # construct optimization problem
        # these are some of the slowest operations (in cpp)
        #np.set_printoptions(threshold=np.inf, precision=2, linewidth=200)

        qH = 2 * (np.dot(B_qp_trans_S, B_qp) + alpha * np.eye(12*horizon))
        qg = 2 * np.dot(B_qp_trans_S, np.dot(A_qp, x0) - X_d)
        lb = np.zeros(20*horizon)
        ub = U_b
        qA = fmat

        lbA = np.array([])
        ubA = np.array([])


        # pass the problem to qpOASES
        n_vars = qH.shape[0]
        n_cons = len(lb)

        cdef problem = QProblem(n_vars, n_cons)
        cdef options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.NONE
        problem.setOptions(options)

        print(f"Time {time.time()-timer}")

        cdef int nWSR = 100
        problem.init(qH, qg.flatten(), qA, None, None, lb, ub, nWSR)

        #print(f"Time {time.time()-timer}")

        xOpt = np.zeros(n_vars)
        problem.getPrimalSolution(xOpt)
        objval = problem.getObjVal()

        Fr_des = np.zeros(12)
        for leg in range(4):
            for axis in range(3):
                Fr_des[leg*3+axis] = xOpt[leg*3+axis]

        print(f"Time {time.time()-timer}")
        
        return Fr_des


    def mpc_opt_partester(self, cfg, low_level_state, rot_w_b, wbc_level_cmd, mpc_table, iters_list, trajAll, foot_locations, num_problems=1):

        cdef np.ndarray[np.double_t, ndim=2] qH
        cdef np.ndarray[np.double_t, ndim=2] qA
        cdef np.ndarray[np.double_t, ndim=2] qg
        cdef np.ndarray[np.double_t, ndim=1] lb
        cdef np.ndarray[np.double_t, ndim=1] ub
        cdef np.ndarray[np.double_t, ndim=1] lbA
        cdef np.ndarray[np.double_t, ndim=1] ubA

        cdef np.ndarray[np.double_t, ndim=2] A
        cdef np.ndarray[np.double_t, ndim=2] B
        cdef np.ndarray[np.double_t, ndim=2] A_qp
        cdef np.ndarray[np.double_t, ndim=2] B_qp
        cdef np.ndarray[np.double_t, ndim=2] ABc
        cdef np.ndarray[np.double_t, ndim=2] Adt
        cdef np.ndarray[np.double_t, ndim=2] Bdt

        cdef np.ndarray[np.double_t, ndim=2] S
        cdef np.ndarray[np.double_t, ndim=1] U_b
        cdef np.ndarray[np.double_t, ndim=2] X_d



        timer = time.time()

        base_orientation = get_quaternion_from_rpy(low_level_state.body_rpy)
        base_pos = low_level_state.body_pos
        if not (cfg is None) and not cfg.observe_corrected_vel:
            base_omega = rot_w_b.dot(low_level_state.body_angular_vel)
            base_vel = rot_w_b.dot(low_level_state.body_linear_vel)
        else:
            base_omega = low_level_state.body_angular_vel
            base_vel = low_level_state.body_linear_vel
        
        base_accel = np.zeros(3)#mpc_level_state.body_linear_accel
        
        swap_legs=True
        if swap_legs:
            q = np.concatenate((low_level_state.joint_pos[3:6], low_level_state.joint_pos[0:3], low_level_state.joint_pos[9:12], low_level_state.joint_pos[6:9]))
            qd = np.concatenate((low_level_state.joint_vel[3:6], low_level_state.joint_vel[0:3], low_level_state.joint_vel[9:12], low_level_state.joint_vel[6:9]))
        else:
            q = low_level_state.joint_pos
            qd = low_level_state.joint_vel

        print(f"Time {time.time()-timer}")

        dt = 0.026
        N_BIG_NUMBER = 5e10
        f_max = 120
        mu = 0.4
        mass = 9. # kg

        weights = [0.25, 0.25, 10, 2, 2, 20, 0, 0, 0.3, 0.2, 0.2, 0.2] # tunable gains
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

        print(f"Time {time.time()-timer}")

        x0 = np.array([rpy[2], rpy[1], rpy[0], p[0], p[1], p[2], w[0], w[1], w[2], v[0], v[1], v[2], -9.8]).reshape(-1, 1)
        R_yaw = np.array(  [[np.cos(yaw), -np.sin(yaw), 0.],
                            [np.sin(yaw), np.cos(yaw), 0,],
                            [0, 0, 1]])
        I_body = np.array(  [[0.07,     0,      0],
                             [0,        0.26,   0],
                             [0,        0,      0.242]] )

        I_world = np.dot(R_yaw, np.dot(I_body, R_yaw.T)) # compute world-frame inertia

        ## neural_ct_ss_mats
        A = np.zeros((13, 13))
        B = np.zeros((13, 12))

        A[3, 9] = 1
        A[9, 9] = 0 # x_drag?
        A[4, 10] = 1
        A[5, 11] = 1
        A[11, 12] = 1
        A[0:3, 6:9] = R_yaw.T

        I_inv = np.linalg.inv(I_world)
        for b in range(4):
            B[6:9, b*3:b*3+3] = np.cross(I_inv, r[b*3:b*3+3])
            B[9:12, b*3:b*3+3] = np.eye(3) / mass

        ## neural_c2qp
        horizon = 10 # planning horizon!
        A_qp = np.zeros((13*horizon, 13))
        B_qp = np.zeros((13*horizon, 12*horizon))

        ABc = np.zeros((25, 25))
        ABc[:13, :13] = dt * A
        ABc[:13, 13:] = dt * B
        #print("ABc", ABc)

        print(f"Time {time.time()-timer}")

        expmm = scipy.linalg.expm(ABc) # check dimension
        Adt = expmm[:13, :13]
        Bdt = expmm[:13, 13:]

        # print(np.max(expmm))

        powerMats = [np.eye(13) for i in range(horizon+1)]
        for i in range(1, horizon+1):
            powerMats[i] = np.dot(Adt, powerMats[i-1])

        for r in range(horizon):
            A_qp[13*r:13*r+13, :] = powerMats[r+1]
            for c in range(horizon):
                if r >= c:
                    a_num = r-c
                    B_qp[13*r:13*r+13, 12*c:12*c+12] = np.dot(powerMats[a_num], Bdt)


        full_weight = np.concatenate((weights, np.array([0])))
        S = np.diag(np.repeat(full_weight, horizon)) # check size of this


        B_qp_trans_S = np.dot(B_qp.T, S)

        X_d = np.zeros((horizon*13, 1))
        for i in range(horizon):
            for j in range(12):
                X_d[13*i+j, 0] = trajAll[12*i+j]

        U_b = np.zeros(5*horizon*4)

        for i in range(horizon):
            for j in range(4):
                U_b[i*4*5 + j*5:i*4*5 + j*5 + 4] = N_BIG_NUMBER
                U_b[i*4*5 + j*5 + 4] = mpc_table[j, i] * f_max # need to flatten mpc_table first?

        mu_inv = 1./mu
        fmat = np.zeros((5*horizon*4, 3*horizon*4))
        f_block = np.array([[1./mu, 0, 1.],
                            [-1./mu, 0, 1.],
                            [0, 1./mu, 1.],
                            [0, -1./mu, 1.],
                            [0, 0, 1.]])

        for i in range(horizon*4):
            fmat[i*5:(i*5+5), i*3:(i*3+3)] = f_block

        print(f"Time {time.time()-timer}")


        # construct optimization problem
        # these are some of the slowest operations (in cpp)
        np.set_printoptions(threshold=np.inf, precision=2, linewidth=200)

        qH = 2 * (np.dot(B_qp_trans_S, B_qp) + alpha * np.eye(12*horizon))
        qg = 2 * np.dot(B_qp_trans_S, np.dot(A_qp, x0) - X_d)
        lb = np.zeros(20*horizon)
        ub = U_b
        qA = fmat
        
        lbA = np.array([])
        ubA = np.array([])

        # construct parallel problems
        qH = np.concatenate(([qH for i in range(num_problems)]))
        qg = np.concatenate(([qg for i in range(num_problems)]))
        lb = np.concatenate(([lb for i in range(num_problems)]))
        ub = np.concatenate(([ub for i in range(num_problems)]))
        qA = np.concatenate(([qA for i in range(num_problems)]))


        # pass the problem to qpOASES
        n_vars = qH.shape[0]
        n_cons = len(lb)

        problem = QProblem(n_vars, n_cons)
        options = Options()
        options.setToMPC()
        options.printLevel = PrintLevel.NONE
        problem.setOptions(options)

        print(f"Time {time.time()-timer}")

        nWSR = 100
        problem.init(qH, qg.flatten(), qA, None, None, lb, ub, nWSR)

        print(f"Time {time.time()-timer}")

        xOpt = np.zeros(n_vars)
        problem.getPrimalSolution(xOpt)
        objval = problem.getObjVal()

        Fr_des = np.zeros(12)
        for leg in range(4):
            for axis in range(3):
                Fr_des[leg*3+axis] = xOpt[leg*3+axis]

        print(f"Time {time.time()-timer}")

        input((n_vars, n_cons))
        
        return Fr_des