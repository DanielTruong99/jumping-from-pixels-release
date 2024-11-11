from cheetahgym.estimators.body_state_estimator import BodyStateEstimator
import numpy as np
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion

class BodyStateEstimatorNaive(BodyStateEstimator):
    def __init__(self, initial_pos=np.zeros(3), initial_rpy=np.zeros(3)):
        super().__init__(initial_pos, initial_rpy)
        self.vel = np.zeros(3)

        self._b_first_visit = True

    def update(self, accel, rpy, dt, **kwargs):
        # self.pos += self.vel * dt
        # self.vel += accel * dt
        # self.rpy = rpy


        if self._b_first_visit:
            self.rpy_ini = self.rpy
            self.rpy_ini[0] = 0
            self.rpy_ini[1] = 0
            self.ori_ini_inv = get_quaternion_from_rpy(-1 * self.rpy_ini)
            self._b_first_visit = False

        rpy_relative = rpy - self.rpy_ini # is this OK?

        rBody_est = get_rotation_matrix_from_rpy(rpy_relative)
        #omegaBody_est = omega
        #omegaWorld_est = rBody_est.T.dot(omegaBody_est)
        aBody_est = accel
        aWorld_est = rBody_est.T.dot(aBody_est)


        self.pos += self.vel * dt
        self.vel += aWorld_est * dt
        self.rpy = rpy