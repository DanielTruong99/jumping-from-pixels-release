from cheetahgym.systems.pybullet_system import PyBulletSystem
import pybullet as p
import pybullet_data

import numpy as np


import pathlib

import matplotlib

try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
    pass


class PyBulletSystemMesh(PyBulletSystem):
    def __init__(self, cfg=None, gui=False, mpc_controller=None, initial_coordinates=None, fix_body=False, log_hd=True,
                 is_shell=False):
        super().__init__(cfg, gui=gui, mpc_controller=mpc_controller, initial_coordinates=initial_coordinates,
                         fix_body=fix_body, log_hd=log_hd, is_shell=is_shell)

    def _setup_pybullet(self):
        p.setGravity(0, 0, -9.8, physicsClientId=self.physicsClient)
        p.resetDebugVisualizerCamera(1.0, 0, -0, [0, 0, 0], physicsClientId=self.physicsClient)
        p.setPhysicsEngineParameter(fixedTimeStep=self.cfg.simulation_dt,
                                    numSolverIterations=50,  # 300,
                                    solverResidualThreshold=1e-30,
                                    numSubSteps=1,
                                    physicsClientId=self.physicsClient)

        robot_start_pos = self.initial_coords[0:3]
        robot_start_ori = [self.initial_coords[4], self.initial_coords[5], self.initial_coords[6],
                           self.initial_coords[3]]

        urdf_name = "mini_cheetah.urdf"
        self.robot = p.loadURDF(f"{str(pathlib.Path(__file__).parent.parent.absolute())}/urdf/{urdf_name}",
                                robot_start_pos, robot_start_ori,
                                useFixedBase=False,
                                physicsClientId=self.physicsClient)

        # self.motor_id_list = [0, 1, 2, 4, 5, 6, 8, 9, 10, 12, 13, 14] # mini_cheetah_simple
        self.motor_id_list = [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]
        # self.num_joints = 16 # mini_cheetah_simple.urdf
        self.num_joints = 12
        self.foot_frames = [2, 5, 8, 11]

        jointIds = []
        for j in range(p.getNumJoints(self.robot, physicsClientId=self.physicsClient)):
            jointIds.append(j)
