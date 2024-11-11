from easyrl.configs.command_line import cfg_from_cmd
from cheetahgym.learn_gp_ppo import learn_with_params
from cheetahgym.learn_gp_bc import learn_bc_with_params
from easyrl.configs import cfg
from easyrl.configs import set_config
from cheetahgym.config.mc_cfg import set_mc_cfg_defaults
import argparse

import os
import sys
import gym
from gym.envs.registration import register


def train_vloco_MPC(parser):

    # registration
    from gym.envs.registration import register

    register(
        id='CheetahMPCEnv-v0',
        entry_point='cheetahgym.envs:CheetahMPCEnv',
    )

    print(parser.parse_args())

    print("Start training!")

    learn_with_params(parser)

def clone_vloco_MPC(parser):

    # registration
    from gym.envs.registration import register

    register(
        id='CheetahMPCEnv-v0',
        entry_point='cheetahgym.envs:CheetahMPCEnv',
    )

    print(parser.parse_args())

    print("Start training!")

    learn_bc_with_params(parser)
    


def get_parser_defaults():
    parser = argparse.ArgumentParser()

    set_mc_cfg_defaults(parser)
    
    parser.set_defaults(simulator_name='PYBULLET')
    #parser.add_argument("--resume", type=bool, default=True)
    parser.set_defaults(num_envs=32)
    parser.set_defaults(log_interval=5)
    parser.set_defaults(eval_interval=5)
    parser.set_defaults(test_num=3)
    parser.add_argument('--episode_steps', type=int, default=100)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(adaptation_steps=10)
    parser.set_defaults(use_rnn=True)
    parser.set_defaults(use_raw_depth_image=True)
    parser.set_defaults(vec_normalize=True)
    parser.set_defaults(use_shared_policy_body=True)
    parser.set_defaults(vertical_body_vel_range=0.0)
    parser.set_defaults(depth_cam_height=120)
    parser.set_defaults(depth_cam_width=160)
    parser.set_defaults(use_22D_actionspace=False)
    parser.set_defaults(env_name='CheetahMPCEnv-v0')

    parser.add_argument('--device', type=str, default="cpu")

    return parser





def train_trot_controller():

    terrain_file = f'10cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    
    parser.set_defaults(fixed_gait_type="trotting")
    parser.set_defaults(nonzero_gait_adaptation=False)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/trotting/heightmaps/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cuda")
    parser.set_defaults(num_envs=32)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=True)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=False)
    parser.set_defaults(use_raw_depth_image=False)
    parser.set_defaults(use_shared_policy_body=False)
    parser.set_defaults(clip_mpc_actions=False)

    train_vloco_MPC(parser=parser)


def train_pronk_controller():

    terrain_file = f'20cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    
    parser.set_defaults(fixed_gait_type="pronking")
    parser.set_defaults(nonzero_gait_adaptation=False)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/pronking/heightmaps/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cuda")
    parser.set_defaults(num_envs=32)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=True)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=False)
    parser.set_defaults(use_raw_depth_image=False)
    parser.set_defaults(use_shared_policy_body=False)
    parser.set_defaults(clip_mpc_actions=False)

    train_vloco_MPC(parser=parser)

def train_pronk_controller_depth():

    terrain_file = f'20cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    
    parser.set_defaults(fixed_gait_type="pronking")
    parser.set_defaults(nonzero_gait_adaptation=False)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/pronking/depth_images/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cpu")
    parser.set_defaults(num_envs=32)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=True)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=True)
    parser.set_defaults(use_raw_depth_image=True)
    parser.set_defaults(use_shared_policy_body=True)
    parser.set_defaults(clip_mpc_actions=False)
    parser.set_defaults(clip_image_left_px=20)

    train_vloco_MPC(parser=parser)


def train_varpronk_controller():

    terrain_file = f'30cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    parser.set_defaults(nonzero_gait_adaptation=True)
    parser.set_defaults(pronk_actions=True)
    parser.set_defaults(num_discrete_actions=2)
    parser.set_defaults(penalize_mean_motor_vel=True)
    parser.set_defaults(mean_motor_vel_penalty_magnitude=0.04)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/varpronk/heightmaps/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cuda")
    parser.set_defaults(num_envs=32)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=True)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=False)
    parser.set_defaults(use_raw_depth_image=False)
    parser.set_defaults(use_shared_policy_body=False)
    parser.set_defaults(clip_mpc_actions=False)

    train_vloco_MPC(parser=parser)

def train_varpronk_controller_stateless():

    terrain_file = f'30cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    parser.set_defaults(nonzero_gait_adaptation=True)
    parser.set_defaults(pronk_actions=True)
    parser.set_defaults(num_discrete_actions=2)
    parser.set_defaults(penalize_mean_motor_vel=True)
    parser.set_defaults(mean_motor_vel_penalty_magnitude=0.04)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/varpronk_stateless/heightmaps/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cuda")
    parser.set_defaults(num_envs=32)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=False)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=False)
    parser.set_defaults(use_raw_depth_image=False)
    parser.set_defaults(use_shared_policy_body=False)
    parser.set_defaults(clip_mpc_actions=False)

    train_vloco_MPC(parser=parser)

def clone_varpronk_controller():
    # run after `train_varpronk_controller`!

    terrain_file = f'30cmgaps_densityCf.json'

    parser = get_parser_defaults()

    seed = 1

    parser.set_defaults(nonzero_gait_adaptation=True)
    parser.set_defaults(pronk_actions=True)
    parser.set_defaults(num_discrete_actions=2)
    parser.set_defaults(penalize_mean_motor_vel=True)
    parser.set_defaults(mean_motor_vel_penalty_magnitude=0.04)

    parser.set_defaults(terrain_cfg_file=f'../../cheetahgym/terrain_config/{terrain_file}')
    parser.set_defaults(randomize_dynamics=False)
    parser.set_defaults(nmpc_use_vel_control=True)
    parser.set_defaults(expert_save_dir=f'/data/DIC_models/varpronk/heightmaps/seed_{seed}/')
    parser.add_argument("--save_dir", type=str, default=f'/data/DIC_models/varpronk/cloned/seed_{seed}')
    parser.set_defaults(iterationsBetweenMPC=17)

    parser.set_defaults(resume=False)
    parser.set_defaults(adaptation_steps=1)
    parser.set_defaults(seed=seed)
    parser.set_defaults(device="cuda")
    parser.set_defaults(num_envs=1)
    parser.set_defaults(render=False)
    parser.set_defaults(episode_steps=500)
    parser.set_defaults(steps_between_policyupdate=100)
    parser.set_defaults(eval_interval=20)
    parser.set_defaults(log_interval=4)

    parser.set_defaults(longitudinal_body_vel_range=0.3)
    parser.set_defaults(longitudinal_body_vel_center=0.5)

    parser.set_defaults(observe_state=True)
    parser.set_defaults(only_observe_body_state=False)

    parser.set_defaults(use_rnn=True)
    parser.set_defaults(use_raw_depth_image=True)
    parser.set_defaults(use_shared_policy_body=True)
    parser.set_defaults(clip_mpc_actions=False)
    parser.set_defaults(clip_image_left_px=20)

    clone_vloco_MPC(parser=parser)





if __name__ == "__main__":
            
    #train_trot_controller()
    #train_pronk_controller()
    #train_pronk_controller_depth()
    train_varpronk_controller()
    #train_varpronk_controller_stateless()
    #clone_varpronk_controller()
