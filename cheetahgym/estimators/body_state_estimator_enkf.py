from cheetahgym.estimators.body_state_estimator import BodyStateEstimator
import numpy as np
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion





class BodyStateEstimatorENKF(BodyStateEstimator):
    def __init__(self, initial_pos=np.zeros(3), initial_rpy=np.zeros(3), dt=0.002):
        super().__init__(initial_pos, initial_rpy)
        self.vel = np.zeros(3)

        self.dt = dt


        # initialize matrices
        self._A = np.zeros((18, 18))
        self._A[0:3, 0:3] = np.eye(3)
        self._A[0:3, 3:6] = self.dt * np.eye(3)
        self._A[3:6, 3:6] = np.eye(3)
        self._A[6:18, 6:18] = np.eye(12)

        self._B = np.zeros((18, 3))
        self._B[3:6, 0:3] = self.dt * np.eye(3)

        self._C = np.zeros((28, 18))
        self._C[0:3, 0:3] = np.eye(3)
        self._C[3:6, 0:3] = np.eye(3)
        self._C[6:9, 0:3] = np.eye(3)
        self._C[9:12, 0:3] = np.eye(3)
        self._C[0:12, 6:18] = -1 * np.eye(12)
        self._C[12:15, 3:6] = np.eye(3)
        self._C[15:18, 3:6] = np.eye(3)
        self._C[18:21, 3:6] = np.eye(3)
        self._C[21:24, 3:6] = np.eye(3)
        self._C[27, 17] = 1
        self._C[26, 14] = 1
        self._C[25, 11] = 1
        self._C[24, 8] = 1

        self._P = np.eye(18) * 100

        self._xhat = np.zeros((18, 1))

        self._b_first_visit = True


    def update(self, accel, rpy, dt, omega=None, q=None, qd=None, foot_locations_rel=None, foot_velocities_rel=None, contactEstimate=None):

        if self._b_first_visit:
            self.rpy_ini = self.rpy
            self.rpy_ini[0] = 0
            self.rpy_ini[1] = 0
            self.ori_ini_inv = get_quaternion_from_rpy(-1 * self.rpy_ini)
            self._b_first_visit = False

        rpy_relative = rpy# - self.rpy_ini # is this OK?

        rBody_est = get_rotation_matrix_from_rpy(rpy_relative).T
        omegaBody_est = rBody_est.dot(omega)
        omegaWorld_est = rBody_est.T.dot(omegaBody_est)
        aBody_est = accel
        aWorld_est = rBody_est.T.dot(aBody_est)
        
        # replace with real values
        process_noise_pimu = 0.02
        process_noise_vimu = 0.02
        process_noise_pfoot = 0.002
        sensor_noise_pimu_rel_foot = 0.001
        sensor_noise_vimu_rel_foot = 0.1
        sensor_noise_zfoot = 0.001

        Q = np.eye(18)
        Q[0:3, 0:3] = np.eye(3) * (self.dt / 20.) * process_noise_pimu
        Q[3:6, 3:6] = np.eye(3) * (self.dt * 9.8 / 20.) * process_noise_vimu
        Q[6:18, 6:18] = np.eye(12) * (self.dt) * process_noise_pfoot

        R = np.eye(28)
        R[0:12, 0:12] = np.eye(12) * sensor_noise_pimu_rel_foot
        R[12:24, 12:24] = np.eye(12) * sensor_noise_vimu_rel_foot
        R[24:28, 24:28] = np.eye(4) * sensor_noise_zfoot

        qindex = 0
        rindex1 = 0
        rindex2 = 0
        rindex3 = 0

        g = np.array([0, 0, -9.81])
        Rbod = rBody_est.T
        a = aWorld_est# + g 
        ps = np.zeros(12)
        vs = np.zeros(12)
        pzs = np.zeros(4)
        trusts = np.zeros(4)



        for i in range(4):
            # get foot position relative to body
            #p_rel = foot_locations_rel[i]
            #dp_rel = foot_velocities_rel[i]


            p_rel, dp_rel = self.compute_leg_pos_vel(q, qd, i)
            p_rel +=  self.get_hip_location(i)
            #print(p_rel_jac, p_rel)

            p_f = Rbod.dot(p_rel)
            dp_f = Rbod.dot(np.cross(omegaBody_est, p_rel) + dp_rel)

            qindex = 6+3*i
            rindex1 = 3*i 
            rindex2 = 12+3*i
            rindex3 = 24+3*i

            # trust = 1
            # phase = min(contactEstimate[i], 1.)
            # trust_window = 0.2

            # if phase < trust_window:
            #   trust = phase / trust_window
            # elif phase > (1 - trust_window):
            #   trust = (1 - phase) / trust_window
            trust = 0#contactEstimate[i] > 0 # in original code, ramps up and ramps down according to scheduled start and end

            high_suspect_number = 100

            Q[(6+3*i):(9+3*i), (6+3*i):(9+3*i)] = (1 + (1 - trust) * high_suspect_number) * Q[(6+3*i):(9+3*i), (6+3*i):(9+3*i)]
            R[(3*i):(3+3*i), (3*i):(3+3*i)] = 1 * R[(3*i):(3+3*i), (3*i):(3+3*i)]
            R[(12+3*i):(15+3*i), (12+3*i):(15+3*i)] = (1 + (1 - trust) * high_suspect_number) * R[(12+3*i):(15+3*i), (12+3*i):(15+3*i)]
            R[(24+i), (24+i)] = (1 + (1 - trust) * high_suspect_number) * R[(24+i), (24+i)]

            trusts[i] = trust 

            #input((p_f, dp_f, trust))

            ps[(3*i):(3*i+3)] = -p_f
            vs[(3*i):(3*i+3)] = (1-trust) * self.vel + trust * (-dp_f)
            pzs[i] = (1-trust)*(self.pos[2] + p_f[2])

        y = np.concatenate((ps, vs, pzs))
        
        self._xhat = self._A.dot(self._xhat) + self._B.dot(a).reshape((18, 1))
    
        Pm = self._A.dot(self._P).dot(self._A.T) + Q 
        yModel = self._C.dot(self._xhat)
        y_err = y.reshape((28, 1)) - yModel
        #print(yModel, y_err)
        S = self._C.dot(Pm).dot(self._C.T) + R

        S_y_err = np.linalg.solve(S, y_err)
        self._xhat += Pm.dot(self._C.T).dot(S_y_err)

        S_C = np.linalg.solve(S, self._C)
        self._P = (np.eye(18) - Pm.dot(self._C.T).dot(S_C)).dot(Pm)

        self._P = (self._P + self._P.T) / 2.

        if np.linalg.det(self._P[0:2, 0:2]) > 0.000001:
            self._P[0:2, 2:18] = 0 
            self._P[2:18, 0:2] = 0 
            self._P[0:2, 0:2] /= 10

        pWorld = self._xhat[0:3, 0]
        vWorld = self._xhat[3:6, 0]
        vBody = rBody_est.dot(vWorld)

        self.pos = pWorld 
        self.vel = vWorld
        self.rpy = rpy


    def compute_leg_pos_vel(self, q, qd, idx):
        l1 = 0.062 #abadLinkLength
        l2 = 0.209 #hipLinkLength
        l3 = 0.195 #kneeLinkLength
        l4 = 0.004 #kneeLinkYOffset
        sideSign = -1 if idx in [0, 2] else 1

        c1, c2, c3 = np.cos(q[idx*3]), np.cos(q[idx*3+1]), np.cos(q[idx*3+2])
        s1, s2, s3 = np.sin(q[idx*3]), np.sin(q[idx*3+1]), np.sin(q[idx*3+2])
        c23 = c2*c3-s2*s3
        s23 = s2*c3+c2*s3

        J = np.zeros((3, 3))
        J[0, 1] = l3 * c23 + l2 * c2
        J[0, 2] = l3 * c23
        J[1, 0] = l3 * c1 * c23 + l2 * c1 * c2 - (l1+l4) * sideSign * s1
        J[1, 1] = -l3 * s1 * s23 - l2 * s1 * s2
        J[1, 2] = -l3 * s1 * s23
        J[2, 0] = l3 * s1 * c23 + l2 * c2 * s1 + (l1+l4) * sideSign * c1
        J[2, 1] = l3 * c1 * s23 + l2 * c1 * s2
        J[2, 2] = l3 * c1 * s23

        p = np.zeros(3)
        p[0] = l3 * s23 + l2 * s2
        p[1] = (l1+l4) * sideSign * c1 + l3 * (s1 * c23) + l2 * c2 * s1
        p[2] = (l1+l4) * sideSign * s1 - l3 * (c1 * c23) - l2 * c1 * c2

        v = J.dot(qd[idx*3:idx*3+3])

        return p, v

    def get_hip_location(self, idx):
        x = 0.19 if idx in [0, 1] else -0.19
        y = 0.049 if idx in [0, 2] else -0.049
        z = 0
        return np.array([x, y, z])