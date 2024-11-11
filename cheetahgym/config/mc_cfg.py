import numpy as np

def set_mc_cfg_defaults(parser):
    parser.add_argument('--env_name', type=str, default='CheetahMPCEnv-v0')
    parser.add_argument('--simulator_name', type=str, default='PYBULLET')
    parser.add_argument('--dataset_path', type=str, default='./terrains/challenging40cm/test')
    parser.add_argument('--terrain_cfg_file', type=str, default='None')
    parser.add_argument('--fixed_heightmap_idx', type=int, default=-1)
    
    #parser.add_argument('--no_randomize_dynamics', dest='randomize_dynamics', action='store_false')
    parser.add_argument('--num_stack', type=int, default=1)
    parser.add_argument('--use_spatial_stacking', dest='use_spatial_stacking', action='store_true')
    parser.add_argument('--record_video', dest='record_video', action='store_true')
    parser.add_argument('--render_heightmap', dest='render_heightmap', action='store_true')
    parser.add_argument('--plot_state', dest='plot_state', action='store_true')
    parser.add_argument('--debug_output', dest='debug_flag', action='store_true')
    parser.add_argument('--print_time', dest='print_time', action='store_true')
    parser.add_argument('--profile', dest='profile', action='store_true')
    parser.add_argument('--log_lcm', dest='log_lcm', action='store_true')
    parser.add_argument('--env_params_file', type=str, default='ICRA-Config')
    
    parser.add_argument('--simulation_dt', type=float, default=0.001)
    parser.add_argument('--control_dt', type=float, default=0.002)
    parser.add_argument('--tg_update_dt', type=float, default=0.0025)

    parser.add_argument('--iterationsBetweenMPC', type=int, default=15)
    parser.add_argument('--mpc_steps_per_env_step', type=int, default=10)

    parser.add_argument('--use_old_ctrl', dest='use_old_ctrl', action='store_true')
    parser.add_argument('--no_mpc_ctrl', dest='no_mpc_ctrl', action='store_true')

    parser.add_argument('--low_level_deploy', dest='low_level_deploy', action='store_true')
    parser.add_argument('--wbc_level_deploy', dest='wbc_level_deploy', action='store_true')
    parser.add_argument('--mpc_level_deploy', dest='mpc_level_deploy', action='store_true')

    parser.add_argument('--use_egl', dest='use_egl', action='store_true')

    # dynamics params
    parser.add_argument('--randomize_dynamics', dest='randomize_dynamics', action='store_true')
    parser.add_argument('--dynamics_randomization_rate', type=float, default=1.0)
    parser.add_argument('--randomize_dynamics_ood', dest='randomize_dynamics_ood', action='store_true')
    parser.add_argument('--randomize_contact_dynamics', dest='randomize_contact_dynamics', action='store_true')
    parser.add_argument('--external_force_interval_ts', type=float, default=0.1)
    parser.add_argument('--external_force_magnitude', type=float, default=0.0)
    parser.add_argument('--external_torque_magnitude', type=float, default=0.0)
    parser.add_argument('--external_force_prob', type=float, default=0.3)
    parser.add_argument('--external_torque_prob', type=float, default=0.3)

    parser.add_argument('--foot_external_force_interval_ts', type=int, default=100)
    parser.add_argument('--foot_external_force_magnitude', type=float, default=0.0)
    parser.add_argument('--foot_external_force_prob', type=float, default=0.0)
    parser.add_argument('--max_initial_yaw', type=float, default=0.0)
    parser.add_argument('--fix_body', dest='fix_body', action='store_true')
    parser.add_argument('--use_actuator_model', dest='use_actuator_model', action='store_true')


    parser.add_argument('--nominal_joint_damping', type=float, default=0.0)
    parser.add_argument('--nominal_joint_friction', type=float, default=0.0)
    parser.add_argument('--nominal_motor_strength', type=float, default=17)
    parser.add_argument('--nominal_control_latency', type=float, default=0.0)
    parser.add_argument('--nominal_pd_latency', type=float, default=0.0)
    parser.add_argument('--nominal_pdtau_command_latency', type=float, default=0.0)
    #parser.add_argument('--nominal_control_step', type=float, default=0.11)
    parser.add_argument('--nominal_ground_friction', type=float, default=0.875)
    parser.add_argument('--nominal_ground_restitution', type=float, default=0.2)
    parser.add_argument('--nominal_ground_spinning_friction', type=float, default=0.03)
    parser.add_argument('--nominal_ground_rolling_friction', type=float, default=0.03)
    parser.add_argument('--nominal_contact_processing_threshold', type=float, default=0.006)
    parser.add_argument('--force_lowpass_filter_window', type=int, default=1)
    
    parser.add_argument('--std_joint_damping', type=float, default=0.05)
    parser.add_argument('--std_joint_friction', type=float, default=0.05)
    parser.add_argument('--std_motor_strength_pct', type=float, default=0.20)
    parser.add_argument('--std_link_mass_pct', type=float, default=0.50)
    parser.add_argument('--std_link_inertia_pct', type=float, default=0.05)
    parser.add_argument('--std_control_latency', type=float, default=0.040)
    parser.add_argument('--std_pd_latency', type=float, default=0.001)
    parser.add_argument('--std_pdtau_command_latency', type=float, default=0.001)
    #parser.add_argument('--nominal_control_step', type=float, default=0.11)
    parser.add_argument('--std_ground_friction', type=float, default=0.375)
    parser.add_argument('--std_ground_restitution', type=float, default=0.2)
    parser.add_argument('--std_ground_spinning_friction', type=float, default=0.03)
    parser.add_argument('--std_ground_rolling_friction', type=float, default=0.03)
    parser.add_argument('--std_contact_processing_threshold', type=float, default=0.004)
    parser.add_argument('--kp_std', type=float, default=0.0)
    parser.add_argument('--kd_std', type=float, default=0.0)


    # action params
    parser.add_argument('--longitudinal_body_vel_range', type=float, default=0.3)
    parser.add_argument('--longitudinal_body_vel_center', type=float, default=0.5)
    parser.add_argument('--lateral_body_vel_range', type=float, default=0.01)
    parser.add_argument('--vertical_body_vel_range', type=float, default=0.01)
    parser.add_argument('--no_nonzero_gait_adaptation', dest='nonzero_gait_adaptation', action='store_false')
    parser.add_argument('--fixed_gait_type', type=str, default="trotting")
    parser.add_argument('--fixed_durations', dest='fixed_durations', action='store_true')
    parser.add_argument('--trot_only', dest='trot_only', action='store_true')
    parser.add_argument('--alt_trot_only', dest='alt_trot_only', action='store_true')
    parser.add_argument('--modulate_durations_anyway', dest='modulate_durations_anyway', action='store_true')
    parser.add_argument('--frequency_adaptation', dest='frequency_adaptation', action='store_true')
    parser.add_argument('--num_discrete_actions', type=int, default=10)
    parser.add_argument('--binary_contact_actions', dest='binary_contact_actions', action='store_true')
    parser.add_argument('--symmetry_contact_actions', dest='symmetry_contact_actions', action='store_true')
    parser.add_argument('--pronk_actions', dest='pronk_actions', action='store_true')
    parser.add_argument('--bound_actions', dest='bound_actions', action='store_true')
    parser.add_argument('--use_continuous_actions_only', dest='use_continuous_actions_only', action='store_true')
    parser.add_argument('--use_22D_actionspace', dest='use_22D_actionspace', action='store_true')
    parser.add_argument('--no_use_gait_smoothing', dest='use_gait_smoothing', action='store_false')
    parser.add_argument('--use_vel_smoothing', dest='use_vel_smoothing', action='store_true')
    parser.add_argument('--no_use_gait_wrapping_obs', dest='use_gait_wrapping_obs', action='store_false')
    parser.add_argument('--no_use_gait_cycling', dest='use_gait_cycling', action='store_false')
    parser.add_argument('--adaptation_steps', type=int, default=10)
    parser.add_argument('--adaptation_horizon', type=int, default=10)
    parser.add_argument('--adaptation_frequency', type=int, default=-1)
    parser.add_argument('--planning_horizon', type=int, default=10)
    parser.add_argument('--use_mpc_force_residuals', dest='use_mpc_force_redisuals', action='store_true')
    parser.add_argument('--clip_mpc_actions', dest='clip_mpc_actions', action='store_true')
    parser.add_argument('--clip_mpc_actions_magnitude', type=float, default=1.0)
    parser.add_argument('--obs_dim', type=int, default=55)
    parser.add_argument('--fpa_heuristic', dest='fpa_heuristic', default=False)

    # controller params for passage to C++
    parser.add_argument('--nmpc_recompute_swingduration_every_step', dest='nmpc_recompute_swingduration_every_step', action='store_true') 
    parser.add_argument("--nmpc_jump_ctrl", dest="nmpc_jump_ctrl", action='store_true') 
    parser.add_argument("--no_nmpc_adaptive_foot_placements", dest="nmpc_adaptive_foot_placements", action='store_false') 
    parser.add_argument("--no_nmpc_use_vel_control", dest="nmpc_use_vel_control", action='store_false') 
    parser.add_argument("--use_lcm_comm", dest="use_lcm_comm", action='store_true')  
    parser.add_argument("--zero_yaw", dest="zero_yaw", action="store_true") 
    parser.add_argument("--fix_body_height", dest="fix_body_height", action="store_true")
    parser.add_argument("--adjust_body_height_pb", type=float, default=0.0)

    parser.add_argument("--mpc_v0", dest="mpc_v0", action="store_true")
    
    # observation params
    parser.add_argument('--state_estimation_mode', type=str, default="cheater")
    parser.add_argument('--use_onehot_obs', dest='use_onehot_obs', action='store_true')
    parser.add_argument('--use_multihot_obs', dest='use_multihot_obs', action='store_true')
    parser.add_argument('--only_observe_body_state', dest='only_observe_body_state', action='store_true')
    parser.add_argument('--no_observe_state', dest='observe_state', action='store_false')
    parser.add_argument('--no_use_vision', dest='use_vision', action='store_false')
    parser.add_argument('--truncate_hmap', dest='truncate_hmap', action='store_true')
    parser.add_argument('--apply_ob_noise', dest='apply_ob_noise', action='store_true')
    parser.add_argument('--apply_heightmap_noise', dest='apply_heightmap_noise', action='store_true')
    parser.add_argument('--apply_motion_blur', dest='apply_motion_blur', action='store_true')
    parser.add_argument('--im_height', type=int, default=15)
    parser.add_argument('--im_width', type=int, default=48)
    parser.add_argument('--im_x_shift', type=float, default=0.65)
    parser.add_argument('--im_y_shift', type=float, default=0.0)
    parser.add_argument('--im_x_resolution', type=float, default=1./30.)
    parser.add_argument('--im_y_resolution', type=float, default=1./30.)
    parser.add_argument('--no_scale_heightmap', dest='scale_heightmap', action='store_false')
    #parser.add_argument('--hmap_resolution', type=float, default=1./30.)
    parser.add_argument('--dilation_px', type=int, default=0)
    parser.add_argument('--erosion_px', type=int, default=0)
    parser.add_argument('--no_observe_mpc_progress', dest='observe_mpc_progress', action='store_false')
    parser.add_argument('--vec_normalize', dest='vec_normalize', action='store_true')
    parser.add_argument('--contact_history_len', type=int, default=10)
    parser.add_argument('--observe_contact_history_scalar', dest='observe_contact_history_scalar', action='store_true')
    parser.add_argument('--observe_corrected_vel', dest='observe_corrected_vel', action='store_true')
    parser.add_argument('--observe_command_vel', dest='observe_command_vel', action='store_true')

    # observation noise
    parser.add_argument('--height_std', type=float, default=0.0)
    parser.add_argument('--roll_std', type=float, default=0.0)
    parser.add_argument('--pitch_std', type=float, default=0.0)
    parser.add_argument('--yaw_std', type=float, default=0.0)
    parser.add_argument('--vel_std', type=float, default=0.0)
    parser.add_argument('--vel_roll_std', type=float, default=0.0)
    parser.add_argument('--vel_pitch_std', type=float, default=0.0)
    parser.add_argument('--vel_yaw_std', type=float, default=0.0)
    parser.add_argument('--joint_pos_std', type=float, default=0.0)
    parser.add_argument('--joint_vel_std', type=float, default=0.0)
    parser.add_argument('--prev_action_std', type=float, default=0.0)
    parser.add_argument('--ob_noise_autocorrelation', type=float, default=0.0)
    parser.add_argument('--cam_rpy_std', type=float, default=0.0)
    parser.add_argument('--cam_pose_std', type=float, default=0.0)

    # sensor params
    parser.add_argument('--use_raw_depth_image', dest='use_raw_depth_image', action='store_true')
    parser.add_argument('--use_grayscale_image', dest='use_grayscale_image', action='store_true')
    parser.add_argument('--observe_gap_state', dest='observe_gap_state', action='store_true')
    parser.add_argument('--num_observed_gaps', type=int, default=1)
    parser.add_argument('--depth_cam_width', type=int, default=160)
    parser.add_argument('--depth_cam_height', type=int, default=120)
    parser.add_argument('--depth_cam_x', type=float, default=0.2773)
    parser.add_argument('--depth_cam_y', type=float, default=0.007)
    parser.add_argument('--depth_cam_z', type=float, default=-0.0085)
    parser.add_argument('--depth_cam_roll', type=float, default=0.)
    parser.add_argument('--depth_cam_pitch', type=float, default=-0.46)
    parser.add_argument('--depth_cam_yaw', type=float, default=0.)
    parser.add_argument('--depth_cam_fov', type=float, default=62.0) # d435 vertical fov
    parser.add_argument('--depth_cam_aspect', type=float, default=4./3.) # d435 aspect ratio
    parser.add_argument('--depth_cam_nearVal', type=float, default=0.1)
    parser.add_argument('--depth_cam_farVal', type=float, default=1.0)
    parser.add_argument('--gimbal_camera', dest='gimbal_camera', action='store_true')
    parser.add_argument('--camera_source', type=str, default="PYBULLET")
    parser.add_argument('--body_pose_source', type=str, default="SIMULATOR")
    parser.add_argument('--clip_image_left_px', type=int, default=0)

    # reward scaling
    parser.add_argument('--command_conditioned', dest='command_conditioned', action="store_true")
    parser.add_argument('--progress_reward_coef', type=float, default=1.0)
    parser.add_argument('--vel_penalty_coef', type=float, default=0.5)
    parser.add_argument('--vel_ceiling', type=float, default=1.0)
    parser.add_argument('--ang_vel_ceiling', type=float, default=0.6)
    parser.add_argument('--roll_penalty_coef', type=float, default=0.02)
    parser.add_argument('--pitch_penalty_coef', type=float, default=0.05)
    parser.add_argument('--yaw_penalty_coef', type=float, default=0.05)
    parser.add_argument('--torque_penalty_coef', type=float, default=0.0)
    parser.add_argument('--terminal_penalty_coef', type=float, default=0.0)
    parser.add_argument('--mpc_loss_penalty_min', type=float, default=-2)
    parser.add_argument('--wbc_loss_penalty_min', type=float, default=-2)
    parser.add_argument('--mpc_loss_penalty_coef', type=float, default=0.0)
    parser.add_argument('--wbc_loss_penalty_coef', type=float, default=0.0)
    parser.add_argument('--leg_inversion_penalty_coef', type=float, default=0.10)
    parser.add_argument('--height_penalty_coef', type=float, default=0.0)
    parser.add_argument('--foot_clearance_reward_coef', type=float, default=0.0)
    parser.add_argument('--foot_clearance_limit', type=float, default=0.20)
    parser.add_argument('--height_floor', type=float, default=0.25)
    parser.add_argument('--penalize_action_change', dest='penalize_action_change', action='store_true')
    parser.add_argument('--action_change_penalty', type=float, default=0.0)
    parser.add_argument('--act_with_accel', dest='act_with_accel', action='store_true')
    parser.add_argument('--reward_ontime_contacts', dest='reward_ontime_contacts', action='store_true')
    parser.add_argument('--no_terminate_on_bad_step', dest='terminate_on_bad_step', action='store_false')
    parser.add_argument('--penalize_contact_change', dest='penalize_contact_change', action='store_true')
    parser.add_argument('--reward_phase_duration', dest='reward_phase_duration', action='store_true')
    parser.add_argument('--contact_change_penalty_magnitude', type=float, default=0.02)
    parser.add_argument('--phase_duration_reward_coef', type=float, default=0.02)
    parser.add_argument('--reward_gap_crossing', dest='reward_gap_crossing', action='store_true')
    parser.add_argument('--gap_crossing_reward_coef', type=float, default=1.0)
    parser.add_argument('--penalize_mean_motor_vel', dest='penalize_mean_motor_vel', action='store_true')
    parser.add_argument('--mean_motor_vel_penalty_magnitude', type=float, default=0.01)
   

    # network params
    parser.add_argument('--use_shared_policy_body', dest='use_shared_policy_body', action='store_true')
    parser.add_argument('--use_rnn', dest='use_rnn', action='store_true')
    parser.add_argument('--use_resnet', dest='use_resnet', action='store_true')

    # default ppo config params
    parser.add_argument('--rew_discount', type=float, default=0.99)
    parser.add_argument('--log_interval', type=int, default=10)
    parser.add_argument('--opt_epochs', type=int, default=2)
    parser.add_argument('--eval_interval', type=int, default=50)
    parser.add_argument('--max_steps', type=int, default=1000000000)
    parser.add_argument('--max_decay_steps', type=int, default=1000000000)
    parser.add_argument('--ent_coef', type=float, default=0.0)
    parser.add_argument('--batch_size', type=int, default=256)
    parser.add_argument('--max_grad_norm', type=int, default=10)
    parser.add_argument('--num_envs', type=int, default=36)
    parser.add_argument('--save_traj_final', dest='save_traj_final', action='store_true')
    parser.add_argument('--steps_between_policyupdate', type=int, default=100)

    parser.add_argument('--use_curriculum', dest='use_curriculum', action='store_true')
    parser.add_argument('--mesg', type=str, default='flat_ground')

    # PMTG related
    parser.add_argument('--pmtg_gait_type', type=str, default="TROT")
    parser.add_argument('--pmtg_beta', type=float, default=0.5)
    parser.add_argument('--pmtg_kp', type=float, default=60)
    parser.add_argument('--pmtg_kd', type=float, default=2.5)
    parser.add_argument('--residual_hk', type=float, default=0.3)
    parser.add_argument('--residual_a', type=float, default=0.1)


    #TG Parameters
    parser.add_argument('--alpha_tg', type=float, default=0.25)
    parser.add_argument('--alphak_tg', type=float, default=0.3)
    parser.add_argument('--alphak_center', type=float, default=0.5)
    parser.add_argument('--alpha_center', type=float, default=0.6)
    parser.add_argument('--f_tg_center', type=float, default=5.0)
    parser.add_argument('--f_tg_scale', type=float, default=0.5)
    parser.add_argument('--hip_center', type=float, default=-0.8086)
    parser.add_argument('--max_stride', type=float, default=2.1)
    parser.add_argument('--add_joint_limits', dest='add_joint_limits', action='store_true')
    parser.add_argument('--joint_limit_termination', dest='joint_limit_termination', action='store_true')
    parser.add_argument('--hip_low', type=float, default=79)
    parser.add_argument('--hip_high', type=float, default=76)
    parser.add_argument('--vel_termination', dest='vel_termination', action='store_true')
    parser.add_argument('--rx', type=float, default=0.1)
    parser.add_argument('--ry', type=float, default=0.03)
    parser.add_argument('--rz', type=float, default=0.1)
    parser.add_argument('--step_in_gap', type=float, default= -0.02)

    # Behavioral Cloning Params
    parser.add_argument('--expert_save_dir', type=str, default="None")
    parser.add_argument('--train_rollout_steps', type=int, default=100)

    parser.add_argument('--pretrain_cnn_dir', type=str, default="None")
    parser.add_argument('--preload_vision_vecnormalize', dest="preload_vision_vecnormalize", action="store_true")
    
    # Texture Params
    parser.add_argument('--texture_scale', type=float, default='0.001')
    parser.add_argument('--texture', type=str, default='./img_2.png')
    parser.add_argument('--pertub_h', type=float, default='0.04')

    #visualize_traj_params
    parser.add_argument('--visualize_traj', dest='visualize_traj', action='store_true')

    #rough terrain_params
    parser.add_argument('--rough_terrain', dest='rough_terrain', action='store_true')

    parser.add_argument('--x_cpg', dest='x_cpg', action='store_true')

def load_cfg(log_path=None):
    from easyrl.configs.command_line import cfg_from_cmd
    from easyrl.configs import cfg, set_config
    from cheetahgym.config.mc_cfg import set_mc_cfg_defaults

    import argparse, copy

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
