from easyrl.engine.ppo_engine import PPOEngine

from itertools import count
from easyrl.configs import cfg

class PPOCurriculumEngine(PPOEngine):
    def __init__(self, agent, runner):
        super().__init__(agent=agent, runner=runner)

        self.terrain_parameters = {"min_gap_width": 5, "max_gap_width": 8, "min_gap_dist": 1.0, "max_gap_dist": 1.5}


    def train(self):
        for iter_t in count():
            if iter_t % cfg.alg.eval_interval == 0:
                det_log_info, _ = self.eval(eval_num=cfg.alg.test_num,
                                            sample=False, smooth=True)
                sto_log_info, _ = self.eval(eval_num=cfg.alg.test_num,
                                            sample=True, smooth=False)

                det_log_info = {f'det/{k}': v for k, v in det_log_info.items()}
                sto_log_info = {f'sto/{k}': v for k, v in sto_log_info.items()}
                eval_log_info = {**det_log_info, **sto_log_info}
                self.agent.save_model(is_best=self._eval_is_best,
                                      step=self.cur_step)
            else:
                eval_log_info = None
            traj, rollout_time = self.rollout_once(sample=True,
                                                   get_last_val=True,
                                                   time_steps=cfg.alg.episode_steps,
                                                   reset_kwargs={"cfgs": [{"terrain_parameters": self.terrain_parameters} for i in range(cfg.alg.num_envs)]})
            train_log_info = self.train_once(traj)
            if iter_t % cfg.alg.log_interval == 0:
                train_log_info['train/rollout_time'] = rollout_time
                if eval_log_info is not None:
                    train_log_info.update(eval_log_info)
                if cfg.alg.linear_decay_lr:
                    train_log_info.update(self.agent.get_lr())
                if cfg.alg.linear_decay_clip_range:
                    train_log_info.update(dict(clip_range=cfg.alg.clip_range))
                scalar_log = {'scalar': train_log_info}
                self.tf_logger.save_dict(scalar_log, step=self.cur_step)
            if self.cur_step > cfg.alg.max_steps:
                break
            if cfg.alg.linear_decay_lr:
                self.agent.decay_lr()
            if cfg.alg.linear_decay_clip_range:
                self.agent.decay_clip_range()

    def adapt_parameters(self, train_log_info, eval_log_info):

        DISTANCE_THRESHOLD = 20.0 #8.0 #meters
        mean_episode_forward_distance = train_log_info["train/rollout_final/forward_distance/mean"]

        if mean_episode_forward_distance > DISTANCE_THRESHOLD:
            self.terrain_parameters["min_gap_width"] += 1
            self.terrain_parameters["max_gap_width"] += 1