from easyrl.configs.command_line import cfg_from_cmd
from easyrl.configs import cfg, set_config

from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
from cheetahgym.envs.cheetah_mpc_env import CheetahMPCEnv
from cheetahgym.utils.heightmaps import RandomizedGapGenerator

import argparse
import numpy as np

def trot_in_place(render=True):

    set_config('ppo')

    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(cfg.alg, parser)


    cfg.alg.render = render
    cfg.alg.linear_decay_clip_range = False
    cfg.alg.linear_decay_lr = False
    #cfg.alg.simulator_name = "PYBULLET_MESHMODEL"
    cfg.alg.simulator_name = "PYBULLET"

    from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    cfg.alg.terrain_cfg_file = "./terrain_config/flatworld/params.json"

    hmap_generator = RandomizedGapGenerator()
    hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)

    cfg.alg.adaptation_steps = 1
    cfg.alg.nonzero_gait_adaptation = False
    cfg.alg.fixed_gait_type = "trotting"
    #cfg.alg.fixed_gait_type = "pronking"


    env = CheetahMPCEnv(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    env.reset()
    action = np.zeros(19)

    for t in range(1800):

        action[0] = -5./3.
        obs, reward, done, info = env.step(action)


if __name__ == "__main__":
	trot_in_place(render=True)