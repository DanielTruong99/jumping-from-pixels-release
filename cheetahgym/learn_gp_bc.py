import gym
import torch.nn as nn
import cheetahgym
import os
import copy

from easyrl.agents.ppo_agent_hybrid import PPOAgentHybrid
from easyrl.agents.ppo_rnn_agent_hybrid import PPORNNAgentHybrid
from easyrl.agents.ppo_agent import PPOAgent
from easyrl.agents.ppo_rnn_agent import PPORNNAgent
from easyrl.configs.command_line import cfg_from_cmd
from easyrl.configs import cfg
from easyrl.configs import set_config
import argparse
from easyrl.engine.ppo_engine import PPOEngine
from easyrl.engine.ppo_rnn_engine import PPORNNEngine
from cheetahgym.engine.ppo_curriculum_engine import PPOCurriculumEngine
from cheetahgym.engine.ppo_rnn_curriculum_engine import PPORNNCurriculumEngine
from easyrl.models.diag_gaussian_policy import DiagGaussianPolicy
from easyrl.models.rnn_diag_gaussian_policy import RNNDiagGaussianPolicy


from easyrl.models.rnn_base import RNNBase
from easyrl.models.value_net import ValueNet
from easyrl.models.rnn_value_net import RNNValueNet
from easyrl.runner.nstep_runner import EpisodicRunner
from easyrl.runner.rnn_runner import RNNRunner
from easyrl.utils.common import set_random_seed
from easyrl.utils.gym_util import make_vec_env
from easyrl.utils.common import get_git_infos
from easyrl.envs.vec_normalize import VecNormalize

from pathlib import Path
import numpy as np

from dataclasses import fields
import pickle as pkl

from cheetahgym.models.gp_model_vision_categorical import GaitParamNet, MixedPolicy
from cheetahgym.models.gp_model_rnn import RNNMixedPolicy 
from cheetahgym.utils.heightmaps import FileReader, MapGenerator, RandomizedGapGenerator
from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

from cheetahgym.engine.bc_rnn_engine import BCRNNEngine
from cheetahgym.agents.bc_mlp_rnn_agent import BCMLPRNNAgent
from cheetahgym.agents.bc_mlp_rnn_agent_hybrid import BCMLPRNNAgentHybrid

from cheetahgym.engine.bc_engine import BCEngine
from cheetahgym.agents.bc_mlp_agent import BCMLPAgent
from cheetahgym.agents.bc_mlp_agent_hybrid import BCMLPAgentHybrid



from easyrl.utils.torch_util import load_torch_model
from easyrl.utils.torch_util import load_state_dict



def main():
    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    learn_bc_with_params(parser)
    


def learn_bc_with_params(parser):
    
    ## the configuration for the student is loaded from command line
    set_config('ppo')
    
    cfg_from_cmd(cfg.alg, parser)

    cfg.alg.linear_decay_clip_range = False
    cfg.alg.linear_decay_lr = False
    
    if cfg.alg.resume or cfg.alg.test:
        if cfg.alg.test:
            skip_params = [
                'test_num',
                'num_envs',
                'sample_action',
                'record_video',
                'debug_flag',
                'randomize_dynamics',
                'dataset_path',
                'render_heightmap',
                'render',
                'device',
                'episode_steps',
                'log_lcm',
                'dilation_px',
                'erosion_px',
                #'external_force_magnitude',
                #'external_torque_magnitude',
                #'latency_seconds',
                'apply_heightmap_noise',
                #'terrain_cfg_file',
                'simulator_name'
            ]
        else:
            skip_params = []
        print(cfg.alg.env_name)
        if cfg.alg.env_name != 'BlindGait-v0' and cfg.alg.env_name != 'VisionCheetah-v0':
            try:
                cfg.alg.restore_cfg(skip_params=skip_params)
            except FileNotFoundError:
                if cfg.alg.resume:
                    print("Unable to resume! Starting from scratch.")
                    cfg.alg.resume = False
                else:
                    print("Could not evaluate -- no model found.")
                    return

    if not cfg.alg.resume and not cfg.alg.test:
        root_path = Path(__file__).resolve().parents[3]
        cheetah_soft_path = Path('/Cheetah-Software')
        try:
            cheetah_soft_git = get_git_infos(cheetah_soft_path)
        except:
            cheetah_soft_git = None
        cfg.alg.extra_cfgs = dict(locomotion_git=get_git_infos(root_path),
                                  cheetah_soft_git=cheetah_soft_git)
    print(vars(cfg.alg))

    print(f"Loading expert configuration from {cfg.alg.expert_save_dir}")

    expert_cfg = copy.deepcopy(cfg.alg)
    expert_cfg.save_dir = cfg.alg.expert_save_dir
    expert_cfg.restore_cfg(skip_params=[])

    cfg.alg.num_envs = 1
    cfg.alg.device = 'cpu'

    print("Creating Environment...")

    if cfg.alg.terrain_cfg_file is not "None":
        hmap_generator = RandomizedGapGenerator()
        hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    else:
        hmap_generator = FileReader(dataset_size=1000, destination=cfg.alg.dataset_path)
        if cfg.alg.test and cfg.alg.fixed_heightmap_idx != -1:
            hmap_generator.fix_heightmap_idx(cfg.alg.fixed_heightmap_idx)

    SIM_NAME = cfg.alg.simulator_name
    use_dynamics_randomization = cfg.alg.randomize_dynamics

    set_random_seed(cfg.alg.seed)
    dummy_cfg = copy.deepcopy(cfg.alg)
    dummy_cfg.plot_state = False
    dummy_cfg.camera_source = "PYBULLET"
    dummyenv = gym.make(dummy_cfg.env_name, hmap_generator=hmap_generator, cfg=dummy_cfg, gui=False, expert_cfg=expert_cfg)
    print("make vec envs:")
    env = make_vec_env(cfg.alg.env_name,
                       cfg.alg.num_envs,
                       seed=cfg.alg.seed,
                       env_kwargs={'hmap_generator': hmap_generator, 'cfg': cfg.alg, 'expert_cfg': expert_cfg})
    print('Num envs: ' + str(cfg.alg.num_envs))
    if cfg.alg.vec_normalize:
        env = VecNormalize(env)
    env.reset()
    ob_vec_size = dummyenv.expert_ob_dim
    cont_action_dim = dummyenv.cont_action_dim
    disc_action_dim = dummyenv.disc_action_dim

    print(" action dim", cont_action_dim, disc_action_dim)

    env.reward_range = [-1, 1]
    
    print("Creating Expert Model...")

    if expert_cfg.use_rnn:
        expert_body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=expert_cfg.num_stack, 
                            im_width = expert_cfg.im_width, 
                            im_height = expert_cfg.im_height, 
                            use_one_hot_embedding=expert_cfg.use_onehot_obs, 
                            num_discrete_actions=expert_cfg.num_discrete_actions, 
                            cfg=expert_cfg)
        expert_rnn_body = RNNBase(body_net=expert_body,
                           rnn_features=128,
                           in_features=128,
                           rnn_layers=1,
                           )
        if disc_action_dim > 0:
            expert_actor = RNNMixedPolicy(expert_rnn_body, cont_action_dim=cont_action_dim, discrete_action_dim=dummyenv.num_discrete_actions, num_discrete_actions=disc_action_dim)
        else:
            expert_actor = RNNDiagGaussianPolicy(expert_rnn_body, action_dim=cont_action_dim, tanh_on_dist=expert_cfg.tanh_on_dist)

    else:
        expert_body = GaitParamNet(input_dim=ob_vec_size, num_stack=expert_cfg.num_stack, im_width = expert_cfg.im_width, im_height = expert_cfg.im_height, use_one_hot_embedding=expert_cfg.use_onehot_obs, num_discrete_actions=expert_cfg.num_discrete_actions, cfg=expert_cfg)

        if disc_action_dim > 0:
            expert_actor = MixedPolicy(expert_body, cont_action_dim=cont_action_dim, discrete_action_dim=dummyenv.num_discrete_actions, num_discrete_actions=disc_action_dim)
        else:
            expert_actor = DiagGaussianPolicy(expert_body, action_dim=cont_action_dim, tanh_on_dist=expert_cfg.tanh_on_dist)
        
    expert_ckpt_file = expert_cfg.model_dir.joinpath('model_best.pt')
    ckpt_data = load_torch_model(expert_ckpt_file, cfg.alg.device)
    load_state_dict(expert_actor, ckpt_data['actor_state_dict'])

    print("Loaded Expert Model!")

    print("Creating Student Model...")

    ob_vec_size = dummyenv.ob_dim
    cont_action_dim = dummyenv.cont_action_dim
    disc_action_dim = dummyenv.disc_action_dim

    # actor to train
    if True:#cfg.alg.use_rnn:
        body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=cfg.alg.num_stack, 
                            im_width = cfg.alg.im_width, 
                            im_height = cfg.alg.im_height, 
                            use_one_hot_embedding=cfg.alg.use_onehot_obs, 
                            num_discrete_actions=cfg.alg.num_discrete_actions, 
                            cfg=cfg.alg)
        rnn_body = RNNBase(body_net=body,
                           rnn_features=128,
                           in_features=128,
                           rnn_layers=1,
                           )

        if disc_action_dim > 0:
            actor = RNNMixedPolicy(rnn_body, cont_action_dim=cont_action_dim, discrete_action_dim=dummyenv.num_discrete_actions, num_discrete_actions=disc_action_dim)
            agent = BCMLPRNNAgentHybrid(actor=actor, expert_actor=expert_actor, state_mask=None, env=env, dim_cont=cont_action_dim, deactivate_rnn=(not cfg.alg.use_rnn)) #same_body = True
        else:
            actor = RNNDiagGaussianPolicy(rnn_body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            agent = BCMLPRNNAgent(actor=actor, expert_actor=expert_actor, state_mask=None, env=env, deactivate_rnn=(not cfg.alg.use_rnn))
        
        runner = RNNRunner(agent=agent, env=env)

        if cfg.alg.use_curriculum:
            print("ERROR: Curriculum not supported for BC at the moment")
            return
            #engine = PPORNNCurriculumEngine(agent=agent,
            #               runner=runner)
        else:
            engine = BCRNNEngine(agent=agent,
                               runner=runner)
    
    print("Created Student Model!")

    print("Training...")

    if not cfg.alg.test:
        if cfg.alg.profile:
            import cProfile
            pr = cProfile.Profile()
            pr.enable()
            try:
                engine.train()
            except KeyboardInterrupt:
                pr.disable()
                pr.print_stats(sort='cumtime')
        else:
            engine.train()
    else:
        # make sure the file doesn't fail before running eval
        
        if cfg.alg.save_traj_final:
            svdir = "./data/evaluation/raw_traj_info/" + cfg.alg.env_name + "/" + cfg.alg.dataset_path + "/seed_" + str(cfg.alg.seed)
            os.makedirs(svdir) 
            with open(svdir + "/raw_traj_info.pkl", "wb") as f:
                if cfg.alg.simulator_name == "RAISIM" and cfg.alg.render and cfg.alg.record_video: 
                    import raisimpy as raisim
                    raisim.OgreVis.get().start_recording_video("/data/video/gp_eval_vis.mp4")

                stat_info, raw_traj_info = engine.eval(render=cfg.alg.render,
                                               save_eval_traj=cfg.alg.save_test_traj,
                                               eval_num=cfg.alg.test_num,
                                               sleep_time=0.04)
        
                pkl.dump(raw_traj_info,  f, pkl.HIGHEST_PROTOCOL)
        
            print(raw_traj_info)
        else:
                if cfg.alg.simulator_name == "RAISIM" and cfg.alg.render and cfg.alg.record_video: 
                    import raisimpy as raisim
                    raisim.OgreVis.get().start_recording_video("/data/video/gp_eval_vis.mp4")

                stat_info, raw_traj_info = engine.eval(render=cfg.alg.render,
                                               save_eval_traj=cfg.alg.save_test_traj,
                                               eval_num=cfg.alg.test_num,
                                               sleep_time=0.04)
        
        
        import pprint
        pprint.pprint(stat_info)
        if cfg.alg.simulator_name == "RAISIM"  and cfg.alg.render and cfg.alg.record_video: raisim.OgreVis.get().stop_recording_video_and_save()
    env.close()




if __name__ == '__main__':
    main()
