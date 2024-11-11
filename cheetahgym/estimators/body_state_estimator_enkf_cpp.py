from cheetahgym.estimators.body_state_estimator import BodyStateEstimator
import numpy as np
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion





class BodyStateEstimatorENKFCpp(BodyStateEstimator):
    def __init__(self, initial_pos=np.zeros(3), initial_rpy=np.zeros(3), dt=0.002, whole_body_controller=None):
        super().__init__(initial_pos, initial_rpy)
        
        self.whole_body_controller = whole_body_controller


    def update(self, accel, rpy, dt, omega=None, q=None, qd=None, foot_locations_rel=None, foot_velocities_rel=None, contactEstimate=None):

        quat = get_quaternion_from_rpy(rpy)[[1, 2, 3, 0]]

        self.whole_body_controller._set_joint_state(q, qd)

        self.whole_body_controller.vnavData.accelerometer = accel # + gravity!!
        self.whole_body_controller.vnavData.quat = quat
        self.whole_body_controller.vnavData.gyro = omega

        # for i in range(3):
        #     self.whole_body_controller.vnavData.accel[i] = accel[i]

        # quat = get_quaternion_from_rpy(rpy)[[1, 2, 3, 0]]
        # for i in range(4):
        #     self.whole_body_controller.vnavData.quat[i] = quat[i]
        # for i in range(3):
        #     self.whole_body_controller.vnavData.gyro[i] = omega[i]


        self.whole_body_controller.stateEstimator.setContactPhase(contactEstimate)

        self.whole_body_controller.stateEstimator.run()

        result = self.whole_body_controller.stateEstimator.getResult()

        self.pos = result.position 
        self.vel = result.vWorld
        self.rpy = result.rpy

        input((self.pos, self.rpy, self.vel))