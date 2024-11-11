from dataclasses import dataclass
from functools import partial

import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from easyrl.agents.base_agent import BaseAgent
from easyrl.configs import cfg
from easyrl.utils.common import linear_decay_percent
from easyrl.utils.rl_logger import logger
from easyrl.utils.torch_util import clip_grad
from easyrl.utils.torch_util import freeze_model
from easyrl.utils.torch_util import load_ckpt_data
from easyrl.utils.torch_util import load_state_dict
from easyrl.utils.torch_util import move_to
from easyrl.utils.torch_util import save_model
from torch.distributions import Independent
from torch.distributions import Normal
from torch.distributions import Categorical
from torch.distributions.kl import kl_divergence
from torch.optim.lr_scheduler import LambdaLR


@dataclass
class BCAgent(BaseAgent):
    actor: nn.Module
    expert_actor: nn.Module
    state_mask: torch.Tensor

    def __post_init__(self):
        move_to([self.actor, self.expert_actor],
                device=cfg.alg.device)
        freeze_model(self.expert_actor)

        if (cfg.alg.linear_decay_lr or cfg.alg.linear_decay_clip_range) and \
                cfg.alg.max_steps > cfg.alg.max_decay_steps:
            logger.warning('max_steps should not be greater than max_decay_steps.')
            cfg.alg.max_decay_steps = int(cfg.alg.max_steps * 1.5)
            logger.warning(f'Resetting max_decay_steps to {cfg.alg.max_decay_steps}!')
        total_epochs = int(np.ceil(cfg.alg.max_decay_steps / (cfg.alg.num_envs *
                                                              cfg.alg.train_rollout_steps)))
        if cfg.alg.linear_decay_clip_range:
            self.clip_range_decay_rate = cfg.alg.clip_range / float(total_epochs)

        p_lr_lambda = partial(linear_decay_percent,
                              total_epochs=total_epochs)

        self.optimizer = optim.Adam(self.actor.parameters(),
                                    lr=cfg.alg.policy_lr,
                                    weight_decay=cfg.alg.weight_decay,
                                    amsgrad=cfg.alg.use_amsgrad)

        self.lr_scheduler = LambdaLR(optimizer=self.optimizer,
                                     lr_lambda=[p_lr_lambda])

    @torch.no_grad()
    def get_action(self, ob, sample=True, get_action_only=False, *args, **kwargs):
        raise NotImplementedError

    def optim_preprocess(self, data):
        raise NotImplementedError

    def optimize(self, data, **kwargs):
        pre_res = self.optim_preprocess(data)
        processed_data = pre_res
        kl_loss = self.cal_loss(**processed_data)
        self.optimizer.zero_grad()
        kl_loss.backward()
        grad_norm = clip_grad(self.actor.parameters(),
                              cfg.alg.max_grad_norm)
        self.optimizer.step()
        optim_info = dict(
            kl_loss=kl_loss.item(),
            grad_norm=grad_norm,
            entropy=processed_data['entropy'].mean().item(),
        )
        return optim_info

    def cal_loss(self, **kwargs):
        if "act_dist" in kwargs:
            with torch.no_grad():
                exp_act_dist = Independent(Normal(loc=kwargs['exp_act_loc'],
                                                  scale=kwargs['exp_act_scale']), 1)
            kl_div_loss = kl_divergence(exp_act_dist, kwargs["act_dist"]).mean()
            #print(exp_act_dist, kwargs["act_dist"])

        else: # hybrid action space
            with torch.no_grad():
                #print(kwargs['exp_act_loc'].shape)
                exp_act_dist_cont = Independent(Normal(loc=kwargs['exp_act_loc'].squeeze(2),
                                                  scale=kwargs['exp_act_scale'].squeeze(2)), 1)
                exp_act_dist_disc = Categorical(logits=kwargs['exp_act_logits'])
            #print(exp_act_dist_cont, kwargs["act_dist_cont"])
            kl_div_loss_cont = kl_divergence(exp_act_dist_cont, kwargs["act_dist_cont"]).mean()
            kl_div_loss_disc = kl_divergence(exp_act_dist_disc, kwargs["act_dist_disc"]).mean()
            kl_div_loss = kl_div_loss_cont + kl_div_loss_disc

        return kl_div_loss

    def eval_mode(self):
        self.actor.eval()

    def train_mode(self):
        self.actor.train()

    def decay_lr(self):
        self.lr_scheduler.step()

    def get_lr(self):
        cur_lr = self.lr_scheduler.get_lr()
        lrs = {'policy_lr': cur_lr[0]}
        return lrs

    def save_model(self, is_best=False, step=None, best_name=None):
        #if is_dist_not_root_rank(cfg.alg):
        #    return
        self.save_env(cfg.alg.model_dir)
        data_to_save = {
            'step': step,
            'actor_state_dict': self.actor.state_dict(),
            'optim_state_dict': self.optimizer.state_dict(),
            'lr_scheduler_state_dict': self.lr_scheduler.state_dict()
        }
        save_model(data_to_save, cfg.alg, is_best=is_best, step=step)

    def load_expert_vec_normalize(self):
        self.load_env_expert(cfg.alg.expert_save_dir+"CheetahMPCEnv-v0/default/seed_0/model/")

    def load_model(self, step=None, pretrain_model=None):
        #if is_dist_not_root_rank(cfg.alg):
        #    return
        #input(f"load_model: {cfg.alg.model_dir}")
        self.load_env(cfg.alg.model_dir)
        ckpt_data = load_ckpt_data(cfg.alg, step=step,
                                   pretrain_model=pretrain_model)
        load_state_dict(self.actor,
                        ckpt_data['actor_state_dict'])

        if pretrain_model is not None:
            return
        self.optimizer.load_state_dict(ckpt_data['optim_state_dict'])
        self.lr_scheduler.load_state_dict(ckpt_data['lr_scheduler_state_dict'])
        return ckpt_data['step']

    @torch.no_grad()
    def get_val(self, ob, *args, **kwargs):
        return None