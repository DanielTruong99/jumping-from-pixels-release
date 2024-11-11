import gym
import torch.nn as nn

import os
import copy

from cheetahgym.agents.bc_agent import BCAgent
from cheetahgym.agents.bc_mlp_rnn_agent import BCMLPRNNAgent
from cheetahgym.agents.bc_mlp_rnn_agent_hybrid import BCMLPRNNAgentHybrid
from easyrl.agents.ppo_agent_hybrid import PPOAgentHybrid
from easyrl.agents.ppo_rnn_agent_hybrid import PPORNNAgentHybrid
from easyrl.agents.ppo_agent import PPOAgent
from easyrl.agents.ppo_rnn_agent import PPORNNAgent
from easyrl.configs.command_line import cfg_from_cmd
from easyrl.configs import cfg
from easyrl.configs import set_config
#from easyrl.configs.ppo_config import cfg.alg
#from gp_config import cfg.alg
import argparse
from easyrl.engine.ppo_engine import PPOEngine
from easyrl.engine.ppo_rnn_engine import PPORNNEngine
from cheetahgym.engine.ppo_curriculum_engine import PPOCurriculumEngine
from cheetahgym.engine.ppo_rnn_curriculum_engine import PPORNNCurriculumEngine
from easyrl.models.categorical_policy import CategoricalPolicy
from easyrl.models.diag_gaussian_policy import DiagGaussianPolicy
from easyrl.models.rnn_categorical_policy import RNNCategoricalPolicy
from easyrl.models.rnn_diag_gaussian_policy import RNNDiagGaussianPolicy

#from easyrl.models.mlp import MLP
from easyrl.envs.shmem_vec_env import ShmemVecEnv
from easyrl.envs.dummy_vec_env import DummyVecEnv
#from easyrl.envs.timeout import NoTimeOutEnv
from easyrl.models.rnn_base import RNNBase
from easyrl.models.value_net import ValueNet
from easyrl.models.rnn_value_net import RNNValueNet
from easyrl.runner.nstep_runner import EpisodicRunner
from easyrl.runner.rnn_runner import RNNRunner
from easyrl.utils.common import set_random_seed
from easyrl.utils.gym_util import make_vec_env
from easyrl.utils.common import get_git_infos
#from easyrl.utils.common import check_if_run_distributed
from easyrl.envs.vec_normalize import VecNormalize
from easyrl.utils.torch_util import load_torch_model
from easyrl.utils.torch_util import load_state_dict



from pathlib import Path
import numpy as np
from tqdm import tqdm

from dataclasses import fields
import pickle as pkl

from cheetahgym.models.gp_model_vision_categorical import GaitParamNet, MixedPolicy
from cheetahgym.models.gp_model_rnn import RNNMixedPolicy 
from cheetahgym.utils.heightmaps import FileReader, MapGenerator, RandomizedGapGenerator
from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

from cheetahgym.data_types.low_level_types import LowLevelState

from gym.envs.registration import register

import copy
import time


register(id='CheetahMPCEnv-v0',
         entry_point='cheetahgym.envs.cheetah_mpc_env:CheetahMPCEnv',
         max_episode_steps=5000,
         reward_threshold=2500.0,
         kwargs={})

def load_cfg(log_path=None):
    from easyrl.configs.command_line import cfg_from_cmd
    from easyrl.configs import cfg, set_config
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

    import argparse


    set_config('ppo')


    parser = argparse.ArgumentParser()
    set_mc_cfg_defaults(parser)
    cfg_from_cmd(cfg.alg, parser)

    cfg.alg.linear_decay_clip_range = False
    cfg.alg.linear_decay_lr = False
    cfg.alg.test = True
    cfg.alg.resume = True
    cfg.alg.test_num = 10
    cfg.alg.device = 'cpu'
    cfg.alg.diff_cfg = {'test': True, 'device': 'cpu'}

    cfg.alg.test = True
    if log_path is not None:
        cfg.alg.save_dir = log_path
        cfg.alg.diff_cfg['save_dir'] = log_path
        skip_params = ['test', 'num_envs', 'device', 'record_video']
        cfg.alg.restore_cfg(skip_params=skip_params, path=Path(log_path))#skip_params=skip_params)

    return copy.deepcopy(cfg)

def build_env(env_name, terrain_cfg_file, cfg, expert_cfg=None, mpc_controller_obj=None, test_mode=False, lcm_publisher=None):
    if cfg.alg.expert_save_dir != "None":
        return build_env_bc(env_name, terrain_cfg_file, cfg, mpc_controller_obj, test_mode, lcm_publisher)

    #from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    cfg.alg.terrain_cfg_file = terrain_cfg_file
    #cfg.alg.env_name = env_name

    if cfg.alg.terrain_cfg_file is not "None":
        hmap_generator = RandomizedGapGenerator()
        #if cfg.alg.terrain_cfg_file[0] == '.':
        #    cfg.alg.terrain_cfg_file = "/workspace/cheetah-gym/cheetahgym/"+cfg.alg.terrain_cfg_file
        hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    else:
        hmap_generator = FileReader(dataset_size=1000, destination=cfg.alg.dataset_path)
        if cfg.alg.test and cfg.alg.fixed_heightmap_idx != -1:
            hmap_generator.fix_heightmap_idx(cfg.alg.fixed_heightmap_idx)


    #import cProfile
    #pr = cProfile.Profile()
    #pr.enable()

    #env = env_type(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    #env.reset()
    #print(cfg.alg.simulator_name)

    env = make_vec_env(cfg.alg.env_name,
                       cfg.alg.num_envs,
                       seed=cfg.alg.seed,
                       env_kwargs={'hmap_generator': hmap_generator, 'cfg': cfg.alg, 'expert_cfg': None, 'mpc_controller_obj': mpc_controller_obj, 'test_mode': test_mode, 'lcm_publisher': lcm_publisher})
    print('Num envs: ' + str(cfg.alg.num_envs))
    if cfg.alg.vec_normalize:
        env = VecNormalize(env)
        #print("VecNormalize!!")
    #env.reset()
    
    return env

def build_env_bc(env_name, terrain_cfg_file, cfg, mpc_controller_obj=None, test_mode=False, lcm_publisher=None):
    #input("Build BC Env")
    #from cheetahgym.utils.heightmaps import FileReader, RandomizedGapGenerator

    cfg.alg.terrain_cfg_file = terrain_cfg_file
    print("DON'T USE EGL")
    cfg.alg.use_egl=False
    #cfg.alg.env_name = env_name

    if cfg.alg.terrain_cfg_file is not "None":
        hmap_generator = RandomizedGapGenerator()
        hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    else:
        hmap_generator = FileReader(dataset_size=1000, destination=cfg.alg.dataset_path)
        if cfg.alg.test and cfg.alg.fixed_heightmap_idx != -1:
            hmap_generator.fix_heightmap_idx(cfg.alg.fixed_heightmap_idx)


    #import cProfile
    #pr = cProfile.Profile()
    #pr.enable()

    #env = env_type(hmap_generator=hmap_generator, cfg=cfg.alg, gui=cfg.alg.render)
    #env.reset()
    #print(cfg.alg.simulator_name)

    #expert_cfg = load_cfg(log_path="/data/"+cfg.alg.expert_save_dir[34:]+"CheetahMPCEnv-v0/default/seed_0/")
    expert_cfg = copy.deepcopy(cfg.alg)
    expert_cfg.save_dir = "/data/"+cfg.alg.expert_save_dir[34:]+"CheetahMPCEnv-v0/default/seed_0/"
    #expert_cfg.save_dir = cfg.alg.expert_save_dir+"CheetahMPCEnv-v0/default/seed_0/"

    expert_cfg.restore_cfg(skip_params=[])

    #input(expert_cfg)

    env = make_vec_env(cfg.alg.env_name,
                       cfg.alg.num_envs,
                       seed=cfg.alg.seed,
                       env_kwargs={'hmap_generator': hmap_generator, 'cfg': cfg.alg, 'expert_cfg': expert_cfg, 'mpc_controller_obj': mpc_controller_obj, 'test_mode': test_mode, 'lcm_publisher': lcm_publisher})
    print('Num envs: ' + str(cfg.alg.num_envs))
    if cfg.alg.vec_normalize:
        env = VecNormalize(env)
        #print("VecNormalize!!")
    #env.reset()
    
    return env

def log_lcm(filename=None):
    # NEED TO FIGURE OUT HOW TO AUTO TERMINATE
    if filename is None: filename = time.strftime("%Y%m%d-%H%M%S")
    import subprocess
    os.makedirs(f"/data/deployment_logs/hardware/{filename}/")
    subprocess.Popen(["lcm-logger", f"/data/deployment_logs/hardware/{filename}/lcm-log.00"], stdout=subprocess.DEVNULL)

def get_agent(env, cfg, lcm_publisher=None):

    if cfg.alg.expert_save_dir != "None":
        print(cfg.alg.expert_save_dir)
        return get_agent_bc(env, cfg, lcm_publisher=lcm_publisher)

    hmap_generator = RandomizedGapGenerator()
    hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    #import copy
    #cfg2 = copy.deepcopy(cfg.alg)
    #cfg2.simulator_name = "None"
    
    #dummyenv = gym.make(cfg.alg.env_name, hmap_generator=hmap_generator, cfg=cfg2, gui=False)
    ob_vec_size = env.envs[0].ob_dim # - dummyenv.env_params["im_width"] * dummyenv.env_params["im_height"]
    cont_action_dim = env.envs[0].cont_action_dim
    disc_action_dim = env.envs[0].disc_action_dim
    num_discrete_actions = env.envs[0].num_discrete_actions
    #action_dim, cont_action_dim, disc_action_dim = env.envs[0]._get_action_space_params()
    
    if cfg.alg.use_rnn:
        body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=cfg.alg.num_stack, 
                            im_width = cfg.alg.im_width, 
                            im_height = cfg.alg.im_height, 
                            use_one_hot_embedding=cfg.alg.use_onehot_obs, 
                            num_discrete_actions=cfg.alg.num_discrete_actions, 
                            cfg=cfg.alg,
                            lcm_publisher=lcm_publisher)
        rnn_body = RNNBase(body_net=body,
                           rnn_features=128,
                           in_features=128,
                           rnn_layers=1,
                           )
        if cfg.alg.use_shared_policy_body:
            critic = RNNValueNet(rnn_body)
        else:
            critic_body = RNNGaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg)
            rnn_critic_body = RNNBase(body_net=critic_body,
                                      rnn_features=128,
                                      in_features=128,
                                      rnn_layers=1,
                                      )
            critic = RNNValueNet(rnn_critic_body)

        if disc_action_dim > 0:
            actor = RNNMixedPolicy(rnn_body, cont_action_dim=cont_action_dim, discrete_action_dim=num_discrete_actions, num_discrete_actions=disc_action_dim)
            agent = PPORNNAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
            #agent = BCMLPRNNAgentHybrid(actor=actor, expert_actor=actor, env=env, dim_cont=cont_action_dim) #same_body = True
        else:
            actor = RNNDiagGaussianPolicy(rnn_body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            agent = PPORNNAgent(actor=actor, critic=critic, env=env, same_body=cfg.alg.use_shared_policy_body)
            #agent = BCMLPRNNAgent(actor=actor, expert_actor=actor, env=env)
        
        
        #runner = RNNRunner(agent=agent, env=env)

        #if cfg.alg.use_curriculum:
        #    engine = PPORNNCurriculumEngine(agent=agent,
        #                   runner=runner)
        #else:
        #    engine = PPORNNEngine(agent=agent,
        #                       runner=runner)
        
    else:
        body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg, lcm_publisher=lcm_publisher)
        if cfg.alg.use_shared_policy_body:
            critic = ValueNet(body)
        else:
            critic_body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg, lcm_publisher=lcm_publisher)
            critic = ValueNet(critic_body)

        if disc_action_dim > 0:
            actor = MixedPolicy(body, cont_action_dim=cont_action_dim, discrete_action_dim=num_discrete_actions, num_discrete_actions=disc_action_dim)
            agent = PPOAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
        else:
            actor = DiagGaussianPolicy(body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            agent = PPOAgent(actor=actor, critic=critic, env=env, same_body=cfg.alg.use_shared_policy_body)
            
        
        #runner = EpisodicRunner(agent=agent, env=env)

        #if cfg.alg.use_curriculum:
        #    engine = PPOCurriculumEngine(agent=agent,
        #                   runner=runner)
        #else:
        #    engine = PPOEngine(agent=agent,
        #                       runner=runner)
        
    
    agent.load_model(step=cfg.alg.resume_step)

    return agent

def get_agent_bc(env, cfg, lcm_publisher=None):

    #input("Get BC Agent")

    hmap_generator = RandomizedGapGenerator()
    hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)
    
    #import copy
    #cfg2 = copy.deepcopy(cfg.alg)
    #cfg2.simulator_name = "None"
    
    #dummyenv = gym.make(cfg.alg.env_name, hmap_generator=hmap_generator, cfg=cfg2, gui=False)
    ob_vec_size = env.envs[0].ob_dim # - dummyenv.env_params["im_width"] * dummyenv.env_params["im_height"]
    cont_action_dim = env.envs[0].cont_action_dim
    disc_action_dim = env.envs[0].disc_action_dim
    num_discrete_actions = env.envs[0].num_discrete_actions
    #action_dim, cont_action_dim, disc_action_dim = env.envs[0]._get_action_space_params()

    expert_cfg = copy.deepcopy(cfg.alg)
    expert_cfg.save_dir = "/data/"+cfg.alg.expert_save_dir[34:]+"CheetahMPCEnv-v0/default/seed_0/"
    #expert_cfg.save_dir = cfg.alg.expert_save_dir+"CheetahMPCEnv-v0/default/seed_0/"
    
    expert_cfg.restore_cfg(skip_params=[])
    
    if True:#cfg.alg.use_rnn:
        body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=cfg.alg.num_stack, 
                            im_width = cfg.alg.im_width, 
                            im_height = cfg.alg.im_height, 
                            use_one_hot_embedding=cfg.alg.use_onehot_obs, 
                            num_discrete_actions=cfg.alg.num_discrete_actions, 
                            cfg=cfg.alg,
                            lcm_publisher=lcm_publisher)
        rnn_body = RNNBase(body_net=body,
                           rnn_features=128,
                           in_features=128,
                           rnn_layers=1,
                           )

        expert_body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=expert_cfg.num_stack, 
                            im_width = expert_cfg.im_width, 
                            im_height = expert_cfg.im_height, 
                            use_one_hot_embedding=expert_cfg.use_onehot_obs, 
                            num_discrete_actions=expert_cfg.num_discrete_actions, 
                            cfg=expert_cfg,
                            lcm_publisher=lcm_publisher)

        # if cfg.alg.use_shared_policy_body:
        #     critic = RNNValueNet(rnn_body)
        # else:
        #     critic_body = RNNGaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg)
        #     rnn_critic_body = RNNBase(body_net=critic_body,
        #                               rnn_features=128,
        #                               in_features=128,
        #                               rnn_layers=1,
        #                               )
        #     critic = RNNValueNet(rnn_critic_body)

        if disc_action_dim > 0:
            actor = RNNMixedPolicy(rnn_body, cont_action_dim=cont_action_dim, discrete_action_dim=num_discrete_actions, num_discrete_actions=disc_action_dim)
            expert_actor = MixedPolicy(expert_body, cont_action_dim=cont_action_dim, discrete_action_dim=num_discrete_actions, num_discrete_actions=disc_action_dim)
            #agent = PPORNNAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
            agent = BCMLPRNNAgentHybrid(actor=actor, expert_actor=expert_actor, env=env, dim_cont=cont_action_dim, deactivate_rnn=(not cfg.alg.use_rnn)) #same_body = True
        else:
            actor = RNNDiagGaussianPolicy(rnn_body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            expert_actor = DiagGaussianPolicy(expert_body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            #agent = PPORNNAgent(actor=actor, critic=critic, env=env, same_body=cfg.alg.use_shared_policy_body)
            agent = BCMLPRNNAgent(actor=actor, expert_actor=expert_actor, env=env, deactivate_rnn=(not cfg.alg.use_rnn))
        
        
        #runner = RNNRunner(agent=agent, env=env)

        #if cfg.alg.use_curriculum:
        #    engine = PPORNNCurriculumEngine(agent=agent,
        #                   runner=runner)
        #else:
        #    engine = PPORNNEngine(agent=agent,
        #                       runner=runner)
        
    # else:
    #     body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg)
    #     if cfg.alg.use_shared_policy_body:
    #         critic = ValueNet(body)
    #     else:
    #         critic_body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg)
    #         critic = ValueNet(critic_body)

    #     if disc_action_dim > 0:
    #         actor = MixedPolicy(body, cont_action_dim=cont_action_dim, discrete_action_dim=num_discrete_actions, num_discrete_actions=disc_action_dim)
    #         agent = BCAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
    #     else:
    #         actor = DiagGaussianPolicy(body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
    #         agent = BCAgent(actor=actor, env=env, same_body=cfg.alg.use_shared_policy_body)
            
        
        #runner = EpisodicRunner(agent=agent, env=env)

        #if cfg.alg.use_curriculum:
        #    engine = PPOCurriculumEngine(agent=agent,
        #                   runner=runner)
        #else:
        #    engine = PPOEngine(agent=agent,
        #                       runner=runner)
        
    
   # agent.load_model(step=cfg.alg.resume_step)
    

    #expert_cfg = load_cfg(log_path="/data/"+cfg.alg.expert_save_dir[34:]+"CheetahMPCEnv-v0/default/seed_0/")

    '''
    expert_ckpt_file = expert_cfg.model_dir.joinpath('model_best.pt')
    ckpt_data = load_torch_model(expert_ckpt_file, cfg.alg.device)
    load_state_dict(expert_actor, ckpt_data['actor_state_dict'])

    input("Loaded expert model!")

    ckpt_file = cfg.alg.model_dir.joinpath('model_best.pt')
    ckpt_data = load_torch_model(ckpt_file, cfg.alg.device)
    load_state_dict(actor, ckpt_data['actor_state_dict'])

    input("Loaded student model!")
    '''
    agent.load_model(step=cfg.alg.resume_step)
    #agent.load_expert_vec_normalize()

    return agent

def setup_engine(env, cfg, lcm_publisher=None):
    '''
    cfg.alg.save_dir = log_path
    cfg.alg.diff_cfg['save_dir'] = log_path
    cfg.alg.test = True
    print(cfg.alg.test)
    skip_params = ['test', 'num_envs', 'device']
    cfg.alg.restore_cfg(skip_params=skip_params, path=Path(log_path))#skip_params=skip_params)
    '''
    hmap_generator = RandomizedGapGenerator()
    hmap_generator.load_cfg(cfg.alg.terrain_cfg_file)

    dummyenv = gym.make(cfg.alg.env_name, hmap_generator=hmap_generator, cfg=cfg.alg, gui=False)
    ob_vec_size = dummyenv.ob_dim # - dummyenv.env_params["im_width"] * dummyenv.env_params["im_height"]
    cont_action_dim = dummyenv.cont_action_dim
    disc_action_dim = dummyenv.disc_action_dim

    if cfg.alg.use_rnn:
        body = GaitParamNet(input_dim=ob_vec_size,
                            num_stack=cfg.alg.num_stack, 
                            im_width = cfg.alg.im_width, 
                            im_height = cfg.alg.im_height, 
                            use_one_hot_embedding=cfg.alg.use_onehot_obs, 
                            num_discrete_actions=cfg.alg.num_discrete_actions, 
                            cfg=cfg.alg,
                            lcm_publisher=lcm_publisher)
        rnn_body = RNNBase(body_net=body,
                           rnn_features=128,
                           in_features=128,
                           rnn_layers=1,
                           )
        if cfg.alg.use_shared_policy_body:
            critic = RNNValueNet(rnn_body)
        else:
            critic_body = RNNGaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg)
            rnn_critic_body = RNNBase(body_net=critic_body,
                                      rnn_features=128,
                                      in_features=128,
                                      rnn_layers=1,
                                      )
            critic = RNNValueNet(rnn_critic_body)

        if disc_action_dim > 0:
            actor = RNNMixedPolicy(rnn_body, cont_action_dim=cont_action_dim, discrete_action_dim=dummyenv.num_discrete_actions, num_discrete_actions=disc_action_dim)
            agent = PPORNNAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
        else:
            actor = RNNDiagGaussianPolicy(rnn_body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            agent = PPORNNAgent(actor=actor, critic=critic, env=env, same_body=cfg.alg.use_shared_policy_body)
        
        runner = RNNRunner(agent=agent, env=env)

        if cfg.alg.use_curriculum:
            engine = PPORNNCurriculumEngine(agent=agent,
                           runner=runner)
        else:
            engine = PPORNNEngine(agent=agent,
                               runner=runner)

    else:
        body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg, lcm_publisher=lcm_publisher)
        if cfg.alg.use_shared_policy_body:
            critic = ValueNet(body)
        else:
            critic_body = GaitParamNet(input_dim=ob_vec_size, num_stack=cfg.alg.num_stack, im_width = cfg.alg.im_width, im_height = cfg.alg.im_height, use_one_hot_embedding=cfg.alg.use_onehot_obs, num_discrete_actions=cfg.alg.num_discrete_actions, cfg=cfg.alg, lcm_publisher=lcm_publisher)
            critic = ValueNet(critic_body)

        if disc_action_dim > 0:
            actor = MixedPolicy(body, cont_action_dim=cont_action_dim, discrete_action_dim=dummyenv.num_discrete_actions, num_discrete_actions=disc_action_dim)
            agent = PPOAgentHybrid(actor=actor, critic=critic, env=env, dim_cont=cont_action_dim, same_body=cfg.alg.use_shared_policy_body) #same_body = True
        else:
            actor = DiagGaussianPolicy(body, action_dim=cont_action_dim, tanh_on_dist=cfg.alg.tanh_on_dist)
            agent = PPOAgent(actor=actor, critic=critic, env=env, same_body=cfg.alg.use_shared_policy_body)
            

        runner = EpisodicRunner(agent=agent, env=env)

        if cfg.alg.use_curriculum:
            engine = PPOCurriculumEngine(agent=agent,
                           runner=runner)
        else:
            engine = PPOEngine(agent=agent,
                               runner=runner)

    return engine


def evaluate_policy(env, agent, evalFunction, num_episodes=1, episode_steps=50, initial_obs=None, use_tqdm=False):

    #print("EVALPOLCI")

    #input(f"EXPERT SAVE DIR {cfg.alg.expert_save_dir}")
    if cfg.alg.expert_save_dir != "None":
        return evaluate_policy_bc(env, agent, evalFunction, num_episodes, episode_steps, initial_obs)
    
    BC = False
    if cfg.alg.expert_save_dir != "None":
        BC = True

    eval_output = [[None for i in range(episode_steps)] for i in range(num_episodes)]

    is_rnn = False

    # execute policy manually
    if initial_obs is None:
        obs = env.reset()
    else:
        obs = initial_obs
    #print(1, type(env))

    done = False
    ts = 0
    hidden_state = None
    is_rnn = True

    if use_tqdm:
        ep_range = tqdm(range(num_episodes), desc="Episode", leave=False)
    else:
        ep_range = range(num_episodes)

    try:
        #for ep in tqdm(range(num_episodes), desc="Episode", leave=False):
        for ep in ep_range:
            #for step in tqdm(range(episode_steps), desc="Step", leave=False):
            if use_tqdm:
                step_range = tqdm(range(episode_steps), desc="Step", leave=False)
            else:
                step_range = range(episode_steps)
            for step in step_range:
                #input()
                #print(step)
                #print(env.cheat_state[0])
                #input()
                if step < episode_steps:
                    #print(obs)
                    #print(f"whole step time: {time.time()-ts}")
                    ts = time.time()
                    if is_rnn:
                        if BC:
                            ret = agent.get_action(obs, sample=False, hidden_state=hidden_state, evaluation=True)
                        else:
                            ret = agent.get_action(obs, sample=False, hidden_state=hidden_state)
                    else:
                        if BC:
                            ret = agent.get_action(obs, sample=False, evaluation=True)
                        else:
                            ret = agent.get_action(obs, sample=False)
                    if len(ret) > 2:
                        is_rnn = True
                        action, action_info, hidden_state = ret
                    else:
                        action, action_info = ret
                    #print(action)
                    #print(f"inference time: {time.time()-ts}")
                    #print("STEPPING")
                    #action[0, 1:] = 0
                    #print(action)
                    
                    # # FIX ROTATION
                    # if abs(env.envs[0].low_level_ob.body_rpy[2]) > 0.3:# manually prevent big yaw
                    #     action[0][3] = 0

                    obs, reward, done, info = env.step(action)
                    #print(f"env step time: {time.time()-ts}")
                    #print("STEPPED", done)
                    obs["state"] = obs["state"] #+ np.random.random(obs["state"].shape) * 0.3

                    # evaluate
                    if evalFunction is not None:
                        #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                        #    eval_output[ep][step] = evalFunction(env.envs[0], action, obs, info)
                        #else:
                        eval_output[ep][step], done2 = evalFunction(env, action, obs, info)

                        if done2: done = True

                    #print("BP", env.low_level_ob.body_pos[0])

                    #if done or env.low_level_ob.body_pos[0] <= -50:
                    #    break
                else:
                    env.envs[0].cfg.nonzero_gait_adaptation = True
                    standstill_action = np.zeros(19)
                    standstill_action[0:3] = np.array([0.0, 0.0, 0])  # vel
                    standstill_action[0:3] =  standstill_action[0:3] / np.array([0.3, 0.3, 0.01]) - np.array([5/3, 0, 0])
                    standstill_action[3] = 0 # vel_rpy
                    standstill_action[4:8] = np.array([0, 0, 0, 0]) # timing
                    standstill_action[8:12] = np.array([10, 10, 10, 10]) # duration
                    standstill_action[12] = 15 # frequency parameter
                    obs, reward, done, info = env.step(standstill_action)

                if done:# or ep < 3:
                    #print("DONE", step, episode_steps)
                    break
            #print("R1", step)
            obs = env.reset()
            hidden_state = None
    except KeyboardInterrupt:
        pass

    return eval_output

def evaluate_policy_bc(env, agent, evalFunction, num_episodes=1, episode_steps=50, initial_obs=None):

    #input("Evaluate BC Policy")

    eval_output = [[None for i in range(episode_steps)] for i in range(num_episodes)]

    is_rnn = False

    # execute policy manually
    if initial_obs is None:
        obs = env.reset()
    else:
        obs = initial_obs
    done = False

    try:
        for ep in range(num_episodes):
            for step in range(episode_steps):
                if step < episode_steps:
                    #print(obs.keys())
                    ts = time.time()
                    if is_rnn:
                        ret = agent.get_action_student(obs, sample=False, hidden_state=hidden_state)
                    else:
                        ret = agent.get_action_student(obs, sample=False)
                    if len(ret) > 2:
                        is_rnn = True
                        action, action_info, hidden_state = ret
                    else:
                        action, action_info = ret
                    #print(action)
                    #print(f"inference time: {time.time()-ts}")
                    #print("STEPPING")
                    obs, reward, done, info = env.step(action)
                    #print("STEPPED")
                    obs["state"] = obs["state"] #+ np.random.random(obs["state"].shape) * 0.3

                    # evaluate
                    if evalFunction is not None:
                        #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                        #    eval_output[ep][step] = evalFunction(env.envs[0], action, obs, info)
                        #else:
                        eval_output[ep][step], done2 = evalFunction(env, action, obs, info)


                    #print("BP", env.low_level_ob.body_pos[0])

                    #if done or env.low_level_ob.body_pos[0] <= -50:
                    #    break
                else:
                    env.envs[0].cfg.nonzero_gait_adaptation = True
                    standstill_action = np.zeros(19)
                    standstill_action[0:3] = np.array([0.0, 0.0, 0])  # vel
                    standstill_action[0:3] =  standstill_action[0:3] / np.array([0.3, 0.3, 0.01]) - np.array([5/3, 0, 0])
                    standstill_action[3] = 0 # vel_rpy
                    standstill_action[4:8] = np.array([0, 0, 0, 0]) # timing
                    standstill_action[8:12] = np.array([10, 10, 10, 10]) # duration
                    standstill_action[12] = 15 # frequency parameter
                    obs, reward, done, info = env.step(standstill_action)

                if done:
                    print("DONE", step, episode_steps)
                    break
            obs = env.reset()
            hidden_state = None
    except KeyboardInterrupt:
        pass
        
    return eval_output



def evaluate_static_gait(env, static_action, evalFunction, num_episodes=1, episode_steps=50, initial_obs=None, test_a0_sequence=None):

    eval_output = [[None for i in range(episode_steps)] for i in range(num_episodes)]

    try:
        # execute policy manually
        if initial_obs is None:
            obs = env.reset()
            #obs = env.update_observation()
        else:
            obs = initial_obs
        done = False
        for ep in range(num_episodes):
            for step in range(episode_steps):
                #input()
                #print(obs)
                #action, action_info = agent.get_action(obs)#, sample=False)
                #print(action)
                if test_a0_sequence is not None:
                    static_action[0] = test_a0_sequence[min(step, len(test_a0_sequence)- 1)]

                if env.envs[0].cfg.env_name == "CheetahMPCEnv-v0":
                    static_action[4:8] = (static_action[4:8] + env.envs[0].cfg.adaptation_steps) % 10

                obs, reward, done, info = env.step(static_action)

                # evaluate
                #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                #    eval_output[ep][step] = evalFunction(env.envs[0], static_action, obs, info)
                #else:
                eval_output[ep][step] = evalFunction(env, static_action, obs, info)

                #if done:
                #    break
                if done:
                    print("DONE", step, episode_steps)
                    #break
            obs = env.reset()
            print('reset')

    except KeyboardInterrupt:
        pass

    return eval_output

def evaluate_playback_observations(env, agent, observations, evalFunction, num_episodes=1, episode_steps=50, initial_obs=None):

    #print("EVALPOLCI")

    #input(f"EXPERT SAVE DIR {cfg.alg.expert_save_dir}")
    #if cfg.alg.expert_save_dir != "None":
    #    return evaluate_playback_actions_bc(env, evalFunction, num_episodes, episode_steps, initial_obs)

    eval_output = [[None for i in range(episode_steps)] for i in range(num_episodes)]

    is_rnn = False

    BC = False
    if cfg.alg.expert_save_dir != "None":
        BC = True

    # execute policy manually
    if initial_obs is None:
        obs = env.reset()
    else:
        obs = initial_obs
    #print(1, type(env))

    done = False
    ts = 0
    hidden_state = None
    is_rnn = True

    try:
        #for ep in tqdm(range(num_episodes), desc="Episode", leave=False):
        for ep in range(num_episodes):
            #for step in tqdm(range(episode_steps), desc="Step", leave=False):
            for step in range(episode_steps):
                #input()
                if step < episode_steps:
                    #print(obs)
                    #print(f"whole step time: {time.time()-ts}")
                    if is_rnn:
                        if BC:
                            ret = agent.get_action(obs, sample=False, hidden_state=hidden_state, evaluation=True)
                        else:
                            ret = agent.get_action(obs, sample=False, hidden_state=hidden_state)
                    else:
                        if BC:
                            ret = agent.get_action(obs, sample=False, evaluation=True)
                        else:
                            ret = agent.get_action(obs, sample=False)
                    if len(ret) > 2:
                        is_rnn = True
                        action, action_info, hidden_state = ret
                    else:
                        action, action_info = ret
                    #print(action)
                    #print(f"inference time: {time.time()-ts}")
                    #print("STEPPING")
                    #action[0, 1:] = 0
                    #print(action)
                    
                    # # FIX ROTATION
                    # if abs(env.envs[0].low_level_ob.body_rpy[2]) > 0.3:# manually prevent big yaw
                    #     action[0][3] = 0

                    obs, reward, done, info = env.step(action)
                    #print(f"env step time: {time.time()-ts}")
                    #print("STEPPED", done)
                    print("A")
                    #print(obs["state"][0][4:16], observations[step]["state"][0][4:16])
                    obs["state"][0][4:] = observations[step]["state"][0][4:]
                    #obs["state"][0] = observations[step]["state"][0] #+ np.random.random(obs["state"].shape) * 0.3
                    #print(obs["state"][0], observations[step]["state"][0])
                    #obs["ob"] = observations[step]["ob"]

                    # evaluate
                    if evalFunction is not None:
                        #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                        #    eval_output[ep][step] = evalFunction(env.envs[0], action, obs, info)
                        #else:
                        eval_output[ep][step] = evalFunction(env, action, obs, info)

                    #print("BP", env.low_level_ob.body_pos[0])

                    #if done or env.low_level_ob.body_pos[0] <= -50:
                    #    break
                else:
                    env.envs[0].cfg.nonzero_gait_adaptation = True
                    standstill_action = np.zeros(19)
                    standstill_action[0:3] = np.array([0.0, 0.0, 0])  # vel
                    standstill_action[0:3] =  standstill_action[0:3] / np.array([0.3, 0.3, 0.01]) - np.array([5/3, 0, 0])
                    standstill_action[3] = 0 # vel_rpy
                    standstill_action[4:8] = np.array([0, 0, 0, 0]) # timing
                    standstill_action[8:12] = np.array([10, 10, 10, 10]) # duration
                    standstill_action[12] = 15 # frequency parameter
                    obs, reward, done, info = env.step(standstill_action)

                if done:# or ep < 3:
                    print("DONE", step, episode_steps)
                    break
            #print("R1", step)
            obs = env.reset()
            hidden_state = None

    except KeyboardInterrupt:
        pass

    return eval_output

def evaluate_playback_actions(env, actions, evalFunction, num_episodes=1, episode_steps=50, initial_obs=None, action_clip=None, planar=False, start_buffer=None):

    #print("EVALPOLCI")

    #input(f"EXPERT SAVE DIR {cfg.alg.expert_save_dir}")
    #if cfg.alg.expert_save_dir != "None":
    #    return evaluate_playback_actions_bc(env, evalFunction, num_episodes, episode_steps, initial_obs)

    eval_output = [[None for i in range(episode_steps)] for i in range(num_episodes)]

    is_rnn = False

    # execute policy manually
    if initial_obs is None:
        obs = env.reset()
    else:
        obs = initial_obs
    #print(1, type(env))

    done = False
    ts = 0
    hidden_state = None
    is_rnn = True

    if start_buffer is not None:
        actions = np.concatenate(([np.zeros_like(actions[0]) for i in range(start_buffer)], actions), axis=0)

    try:

        #for ep in tqdm(range(num_episodes), desc="Episode", leave=False):
        for ep in range(num_episodes):
            #for step in tqdm(range(episode_steps), desc="Step", leave=False):
            for step in range(episode_steps):
                #input()
                if step < episode_steps:
                    #print(obs)
                    #print(f"whole step time: {time.time()-ts}")
                    ts = time.time()
                    action = actions[step]
                    if action_clip is not None:
                        action[0:3] = np.clip(action[0:3], -action_clip, action_clip)
                    if planar:
                        action[1:3] = 0

                    
                    #print(action)
                    #print(f"inference time: {time.time()-ts}")
                    #print("STEPPING")
                    #action[0, 1:] = 0
                    #print(action)
                    
                    # # FIX ROTATION
                    # if abs(env.envs[0].low_level_ob.body_rpy[2]) > 0.3:# manually prevent big yaw
                    #     action[0][3] = 0

                    obs, reward, done, info = env.step(action)
                    #print(f"env step time: {time.time()-ts}")
                    #print("STEPPED", done)
                    obs["state"] = obs["state"] #+ np.random.random(obs["state"].shape) * 0.3

                    # evaluate
                    if evalFunction is not None:
                        #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                        #    eval_output[ep][step] = evalFunction(env.envs[0], action, obs, info)
                        #else:
                        eval_output[ep][step] = evalFunction(env, action, obs, info)

                    #print("BP", env.low_level_ob.body_pos[0])

                    #if done or env.low_level_ob.body_pos[0] <= -50:
                    #    break
                else:
                    env.envs[0].cfg.nonzero_gait_adaptation = True
                    standstill_action = np.zeros(19)
                    standstill_action[0:3] = np.array([0.0, 0.0, 0])  # vel
                    standstill_action[0:3] =  standstill_action[0:3] / np.array([0.3, 0.3, 0.01]) - np.array([5/3, 0, 0])
                    standstill_action[3] = 0 # vel_rpy
                    standstill_action[4:8] = np.array([0, 0, 0, 0]) # timing
                    standstill_action[8:12] = np.array([10, 10, 10, 10]) # duration
                    standstill_action[12] = 15 # frequency parameter
                    obs, reward, done, info = env.step(standstill_action)

                if done:# or ep < 3:
                    print("DONE", step, episode_steps)
                    break
            #print("R1", step)
            obs = env.reset()
            hidden_state = None

    except KeyboardInterrupt:
        pass

    return eval_output


def playback_actions(env, action_list, evalFunction, num_episodes=1, episode_steps=50):

    eval_output = [[None for i in range(len(action_list))] for i in range(num_episodes)]

    # execute policy manually
    obs = env.reset()
    done = False
    for ep in range(num_episodes):
        for step in range(len(action_list)):
            #print(obs)
            #action, action_info = agent.get_action(obs)#, sample=False)
            #print(action)
            obs, reward, done, info = env.step(action_list[step])

            # evaluate
            #if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
            #    eval_output[ep][step] = evalFunction(env.envs[0], action_list[step], obs, {})
            #else:
            eval_output[ep][step] = evalFunction(env, action_list[step], obs, {})

            if done:
                break
        obs = env.reset()
        print('reset')

    return eval_output
        

def playback_states(env, state_list, evalFunction, num_episodes=1, episode_steps=50):

    eval_output = [[None for i in range(len(state_list))] for i in range(num_episodes)]

    # execute policy manually
    obs = env.reset()
    done = False
    for ep in range(num_episodes):
        for step in range(len(state_list)):
            #print(obs)
            #action, action_info = agent.get_action(obs)#, sample=False)
            #print(action)
            
            # evaluate
            if isinstance(env, DummyVecEnv) or isinstance(env, VecNormalize):
                env.envs[0].simulator.set_state(state_list[step])
                env.envs[0].low_level_ob = state_list[step]
                env.envs[0].update_observation()
                print('body pos:', state_list[step].body_pos)
                eval_output[ep][step] = evalFunction(env, None, state_list[step], {})
            else:
                env.simulator.set_state(state_list[step])
                env.envs[0].low_level_ob = state_list[step]
                env.update_observation()
                print('body pos:', state_list[step].body_pos)
                eval_output[ep][step] = evalFunction(env, None, state_list[step], {})

            input()

            #if done:
            #    break
        obs = env.reset()
        print('reset')

    return eval_output

def log_obs_action(env, action, obs, infin):
    info = {}
    info["obs"] = copy.deepcopy(env.low_level_ob)
    info["cmd"] = copy.deepcopy(env.mpc_level_cmd)
    info["vision"] = copy.deepcopy(env.ob_scaled_vis["ob"])
    info["action"] = action
    info["info"] = infin
    #print(vars(info["obs"]))

    return info


def plot(infos):

    from matplotlib import pyplot as plt

    # plot states
    plt.figure()
    plot_state_names = ["x_pos", "y_pos", "body_height",
                        "roll", "pitch", "yaw", 
                        "roll_vel", "pitch_vel", "yaw_vel", 
                        "x_vel", "y_vel", "z_vel", 
                        "x_vel_cmd", "y_vel_cmd", "z_vel_cmd", 
                        "yaw_vel_cmd"]
    plot_state_extractors = {"x_pos": lambda x: x["obs"].body_pos[0],
                             "y_pos": lambda x: x["obs"].body_pos[1],
                             "body_height": lambda x: x["obs"].body_pos[2],
                             "roll": lambda x: x["obs"].body_rpy[0],
                             "pitch": lambda x: x["obs"].body_rpy[1],
                             "yaw": lambda x: x["obs"].body_rpy[2],
                             "roll_vel": lambda x: x["obs"].body_angular_vel[0],
                             "pitch_vel": lambda x: x["obs"].body_angular_vel[1],
                             "yaw_vel": lambda x: x["obs"].body_angular_vel[2],
                             "x_vel": lambda x: x["obs"].body_linear_vel[0],
                             "y_vel": lambda x: x["obs"].body_linear_vel[1],
                             "z_vel": lambda x: x["obs"].body_linear_vel[2],
                             "x_vel_cmd": lambda x: x["cmd"].vel_cmd[0],
                             "y_vel_cmd": lambda x: x["cmd"].vel_cmd[1],
                             "z_vel_cmd": lambda x: x["cmd"].vel_cmd[2],
                             "yaw_vel_cmd": lambda x: x["cmd"].vel_rpy_cmd[2]}
    plot_states = {state_name: [] for state_name in plot_state_extractors.keys()}

    for i in range(len(infos[0])):
        if infos[0][i] is not None:
            for state_name in plot_state_names:
                plot_states[state_name].append(plot_state_extractors[state_name](infos[0][i]))

    for i in range(len(plot_state_names)):
        plt.subplot(4, 4, i+1)
        plt.plot([j for j in range(len(plot_states[plot_state_names[i]]))], plot_states[plot_state_names[i]])
        plt.title(f"Robot {plot_state_names[i]}")


    # plot frames
    visual_frames = []
    for i in range(len(infos[0])):
        if infos[0][i] is not None:
            visual_frames.append(infos[0][i]["vision"])

    plt.figure()
    num_frames = min(16, len(visual_frames))
    #num_frames = 
    for i in range(num_frames):
        plt.subplot(4, 4, i+1)
        plt.imshow(visual_frames[i])

    #plt.show()

def get_action_list(infos):
    action_list = []
    for i in range(len(infos[0])):
        if infos[0][i] is None:
            break
        action_list += [infos[0][i]['action']]
    return action_list

def get_state_list(infos):
    state_list = []
    for i in range(len(infos[0])):
        if infos[0][i] is None:
            break
        state_list += [infos[0][i]['obs']]
    return state_list
