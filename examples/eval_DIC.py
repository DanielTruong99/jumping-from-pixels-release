from cheetahgym.evaluation.evaluation_utils import load_cfg, build_env, get_agent, evaluate_policy, evaluate_static_gait, build_env_bc, get_agent_bc
from cheetahgym.evaluation.evaluation_utils import playback_actions, playback_states
from cheetahgym.evaluation.evaluation_utils import plot, log_obs_action, get_action_list, get_state_list
import numpy as np
import copy
import time
import matplotlib
matplotlib.use("TkAgg")
from matplotlib import pyplot as plt
from tqdm import tqdm

#plt.style.use("./plotting/plot_style.yml")

def log_traj(env, action, obs, info):

    body_height = env.simulator.body_heights
    body_xpos = env.simulator.body_xpos
    body_vel = env.simulator.body_vels

    torques = np.abs(env.simulator.torques)
    min_torque = np.min(np.array(torques), axis=1)
    median_torque = np.median(np.array(torques), axis=1)
    max_torque = np.max(np.array(torques), axis=1)
    motor_velocities = np.abs(env.simulator.motor_velocities)
    min_motor_vel = np.min(np.array(motor_velocities), axis=1)
    median_motor_vel = np.median(np.array(motor_velocities), axis=1)
    max_motor_vel = np.max(np.array(motor_velocities), axis=1)
    power = np.sum(np.multiply(torques, motor_velocities), axis=1)
    max_power = np.max(np.multiply(torques, motor_velocities), axis=1)
    #print(power.shape, body_vel.shape)
    CoT = power / np.abs(9 * np.array(body_vel) + 0.01)

    foot_positions = env.simulator.foot_positions

    return [body_height,
            body_xpos,
            body_vel,
            min_torque,
            median_torque,
            max_torque,
            min_motor_vel,
            median_motor_vel,
            max_motor_vel,
            power,
            CoT,
            foot_positions,
            max_power
            ]


def evaluate(model_path, num_episodes = 1, terrain_cfg_file = None, use_static = False):
    # run on hardware, but check against the prediction from pybullet

    # load agent and env
    cfg = load_cfg(log_path=model_path)
    cfg.alg.num_envs = 1
    cfg.alg.randomize_dynamics = False
    cfg.alg.randomize_contact_dynamics = False
    cfg.alg.foot_external_force_magnitude = 0.0
    cfg.alg.external_force_magnitude = 0.0
    cfg.alg.external_torque_magnitude = 0.0
    cfg.alg.render = True
    #cfg.alg.device = "cuda"
    #cfg.alg.state_estimation_mode = "enkf"; input("SIMULATING WITH ENKF STATE ESTIMATOR")
    if terrain_cfg_file is not None:
        cfg.alg.terrain_cfg_file = terrain_cfg_file
   

    env = build_env(env_name=cfg.alg.env_name, terrain_cfg_file=cfg.alg.terrain_cfg_file, cfg=cfg)
    
    env.simulator.log_hd = True
    
    env.reset()

    agent = get_agent(env=env, cfg=cfg)
    
    infos = []
    gap_size = 60 # cm
    num_episodes = 5
    ep_steps = 5000
    
    initial_obs = None

    infos = [[None for i in range(cfg.alg.episode_steps)] for i in range(num_episodes)]


    obs = env.reset()

    done = False
    ts = 0
    hidden_state = None

    ep_range = tqdm(range(num_episodes), desc="Episode", leave=False)
    step_range = tqdm(range(cfg.alg.episode_steps), desc="Step", leave=False)

    for ep in ep_range:
        for step in step_range:
            if hidden_state is not None:
                ret = agent.get_action(obs, sample=False, hidden_state=hidden_state)
            else:
                ret = agent.get_action(obs, sample=False)
            if len(ret) > 2:
                is_rnn = True
                action, action_info, hidden_state = ret
            else:
                action, action_info = ret

            obs, reward, done, info = env.step(action)

            infos[ep][step] = log_traj(env, action, obs, info)

            if done:
                break
        obs = env.reset()
        hidden_state = None




    end = len(infos[0]) - 1
    for i in range(len(infos[0])):
        if infos[0][i] is None:
            end = i
            break

    body_heights = np.concatenate([infos[0][i][0] for i in range(end)])
    body_xposs = np.concatenate([infos[0][i][1] for i in range(end)])
    body_vels = np.concatenate([infos[0][i][2] for i in range(end)])
    min_torques = np.concatenate([infos[0][i][3] for i in range(end)])
    median_torques = np.concatenate([infos[0][i][4] for i in range(end)])
    max_torques = np.concatenate([infos[0][i][5] for i in range(end)])
    min_motor_vels = np.concatenate([infos[0][i][6] for i in range(end)])
    median_motor_vels = np.concatenate([infos[0][i][7] for i in range(end)])
    max_motor_vels = np.concatenate([infos[0][i][8] for i in range(end)])
    powers = np.concatenate([infos[0][i][9] for i in range(end)])
    CoTs = np.concatenate([infos[0][i][10] for i in range(end)])
    foot_positions = np.concatenate([infos[0][i][11] for i in range(end)])
    max_powers = np.concatenate([infos[0][i][12] for i in range(end)])

    times = [i * cfg.alg.simulation_dt for i in range(len(min_torques))]

    f1 = plt.figure(figsize=(5,8))

    plt.subplot(5, 2, 1)
    plt.plot(body_xposs, body_heights)
    plt.title("Robot CoM Trajectory")
    plt.ylabel("Z coordinate")
    plt.xlabel("X coordinate (m)")

    plt.subplot(5, 2, 2)
    plt.plot([i * 0.3 for i in range(len(body_vels))], body_vels)
    plt.title("Robot CoM Velocity")
    plt.ylabel("Velocity (m/s)")
    plt.xlabel("Time (s)")

    plt.subplot(5, 1, 2)
    plt.plot(times, min_torques)
    plt.plot(times, median_torques)
    plt.plot(times, max_torques)
    plt.title("Motor Torques")
    plt.ylabel("Torque (Nm)")
    plt.xlabel("Time (s)")
    plt.legend(["Min", "Median", "Max"], ncol=3)

    plt.subplot(5, 1, 3)
    plt.plot(times, min_motor_vels)
    plt.plot(times, median_motor_vels)
    plt.plot(times, max_motor_vels)
    plt.title("Motor Angular Velocities")
    plt.ylabel("Angular Vel (rad/s)")
    plt.xlabel("Time (s)")
    plt.legend(["Min", "Median", "Max"], ncol=3)

    plt.subplot(5, 1, 4)
    plt.plot(times, max_powers)
    plt.title("Max power output of all 12 motors")
    plt.ylabel("Power (W)")
    plt.xlabel("Time (s)")

    plt.subplot(5, 1, 5)
    plt.plot(times, CoTs)
    plt.title("Instantaneous Cost of Transport (CoT) (J/(Nm))")
    plt.ylabel("CoT (J/(Nm))")
    plt.xlabel("Time (s)")

    plt.subplots_adjust(hspace=0.8)

    print(len(foot_positions))

    print(foot_positions)
    print(foot_positions[0])
    print(foot_positions[0][0])

    f2, ax = plt.subplots()
    plt.xlabel("Forward Progress (m)")
    plt.ylabel("Robot Height (m)")
    ax.xaxis.label.set_fontsize(14)
    ax.yaxis.label.set_fontsize(14)

    plt.plot(body_xposs, body_heights, 'k')
    colors = ['green', 'blue', 'red', 'orange']
    for f in range(4):
        foot_xp = [foot_positions[i][f][0] for i in range(len(foot_positions)) if foot_positions[i][f][0] != foot_positions[0][f][0]]
        foot_zp = [foot_positions[i][f][2] for i in range(len(foot_positions)) if foot_positions[i][f][0] != foot_positions[0][f][0]]
        plt.plot(foot_xp, foot_zp, '--', color=colors[f])
    plt.plot(body_xposs, np.zeros_like(body_xposs), 'g', linewidth=5)
    plt.legend(["Body", "LF Foot", "RF Foot", "LR Foot", "RR Foot"], prop={'size': 14})

    ax.title.set_fontsize(20)
    ax.xaxis.label.set_fontsize(16)
    ax.yaxis.label.set_fontsize(16)
    for tick in ax.xaxis.get_major_ticks():
        tick.label.set_fontsize(12)
    for tick in ax.yaxis.get_major_ticks():
        tick.label.set_fontsize(12)

    if cfg.alg.render:
        plt.show()



if __name__ == "__main__":


    # with heightmaps
    #model_path = '/data/DIC_models/trotting/heightmaps/seed_1/CheetahMPCEnv-v0/default/seed_0/'
    #model_path = '/data/DIC_models/pronking/heightmaps/seed_1/CheetahMPCEnv-v0/default/seed_0/'
    model_path = '/data/DIC_models/varpronk/heightmaps/seed_1/CheetahMPCEnv-v0/default/seed_0/'
    
    evaluate(model_path, num_episodes = 1)
