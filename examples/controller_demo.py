import numpy as np
import argparse

from easyrl.configs.command_line import cfg_from_cmd
from easyrl.configs import cfg, set_config
from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

from cheetahgym.data_types.mpc_level_types import MPCLevelCmd, MPCLevelState
from cheetahgym.utils.heightmaps import RandomizedGapGenerator
from cheetahgym.utils.rotation_utils import get_quaternion_from_rpy, get_rotation_matrix_from_quaternion, get_rpy_from_quaternion, get_rotation_matrix_from_rpy, inversion

from cheetahgym.systems.pybullet_system import PyBulletSystem
from cheetahgym.controllers.control_container_old import ControlContainer
from cheetahgym.envs.cheetah_mpc_env import CheetahMPCEnv


def trot_in_place(render=True):

    set_config('ppo')
    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(cfg.alg, parser)

    cfg.alg.terrain_cfg_file = "./terrain_config/flatworld/params.json"
    #cfg.alg.terrain_cfg_file = './terrain_config/long_platformed_10cmgaps/params.json'

    hmap_generator = RandomizedGapGenerator()
    hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    cfg.alg.render = render

    control_dt, iterationsBetweenMPC = 0.002, 15

    pybullet_simulator = PyBulletSystem(cfg.alg, gui=cfg.alg.render)
    ob = pybullet_simulator.get_obs()

    controller = ControlContainer(control_dt, iterationsBetweenMPC)
    controller.reset_ctrl()

    t=0

    random_force = np.zeros(3)

    def apply_random_force(force_magnitude):

        nonlocal random_force
    
        #todo: vary force slowly
        if t % 100 == 0:
            random_force = np.random.randn(3)*force_magnitude

        p.applyExternalForce(objectUniqueId=pybullet_simulator.robot,
                                 linkIndex=0,
                                 forceObj=random_force,
                                 posObj=np.zeros(3),
                                 flags=p.LINK_FRAME,
                                 physicsClientId=pybullet_simulator.physicsClient)


    # trotting
    contact_table = np.array([[1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [1, 0, 0, 1],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],
                              [0, 1, 1, 0],

                            ])
    # # pronking
    # contact_table = np.array([[1, 1, 1, 1],
    #                           [1, 1, 1, 1],
    #                           [1, 1, 1, 1],
    #                           [1, 1, 1, 1],
    #                           [1, 1, 1, 1],
    #                           [0, 0, 0, 0],
    #                           [0, 0, 0, 0],
    #                           [0, 0, 0, 0],
    #                           [0, 0, 0, 0],
    #                           [0, 0, 0, 0],

    #                         ])


    
    for i in range(30000):
        ob = pybullet_simulator.get_obs()

        #apply_random_force(force_magnitude=10)

        # build command
        gait_params = MPCLevelCmd()
        gait_params.vel_cmd = np.array([0.0, 0, 0])
        gait_params.vel_rpy_cmd = np.zeros(3)
        gait_params.fp_rel_cmd = np.zeros(8)
        gait_params.fh_rel_cmd = np.zeros(4)
        gait_params.footswing_height = 0.06
        gait_params.offsets_smoothed = np.zeros(4).astype(int)
        gait_params.durations_smoothed = np.zeros(4).astype(int)
        gait_params.mpc_table_update = contact_table.flatten()
        gait_params.vel_table_update = np.tile(gait_params.vel_cmd, (10, 1)).flatten()
        gait_params.vel_rpy_table_update = np.zeros((30, 1))
        gait_params.iterations_table_update = np.ones((10, 1)) * iterationsBetweenMPC
        gait_params.iterationsBetweenMPC = iterationsBetweenMPC
        gait_params.planningHorizon = 10
        gait_params.adaptationSteps = 1
        gait_params.adaptationHorizon = 10

        # compute action
        rot_w_b = inversion(get_rotation_matrix_from_rpy(ob.body_rpy))
        action = controller.step_with_mpc_table(gait_params, None, ob, rot_w_b, foot_locations=pybullet_simulator.get_foot_positions())

        # simulate action
        pybullet_simulator.step_state_low_level(action, loop_count=int(control_dt/cfg.alg.simulation_dt))

        if i % iterationsBetweenMPC == 0:
            contact_table = np.concatenate((contact_table[1:, :], contact_table[0:1, :]))
            controller.set_accept_update()


if __name__ == '__main__':
    trot_in_place(render=True)