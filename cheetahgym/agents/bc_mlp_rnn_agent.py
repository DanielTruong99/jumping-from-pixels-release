from dataclasses import dataclass

import torch
import numpy as np
from easyrl.configs import cfg
from easyrl.utils.torch_util import action_entropy
from easyrl.utils.torch_util import action_from_dist
from easyrl.utils.torch_util import action_log_prob
from easyrl.utils.torch_util import torch_to_np
from easyrl.utils.torch_util import torch_float

from cheetahgym.agents.bc_agent import BCAgent


@dataclass
class BCMLPRNNAgent(BCAgent):
    def __init__(self, env, actor, expert_actor, state_mask=None, deactivate_rnn=False):
        super().__init__(env, actor, expert_actor, state_mask)
        self.expert_hidden_state = None
        self.deactivate_rnn = deactivate_rnn

    def extract_expert_state(self, ob):
        #tob = torch_float(ob, device=cfg.alg.device)
        #tob = tob.unsqueeze(dim=1)
        #tob = {key: torch_float(ob[key], device=cfg.alg.device).unsqueeze(dim=1) for key in ob} 
        tob = {}
        tob["ob"] = torch_float(ob["expert_ob"], device=cfg.alg.device).unsqueeze(dim=1)
        tob["state"] = torch_float(ob["expert_state"], device=cfg.alg.device).unsqueeze(dim=1)
        return tob

    def extract_student_state(self, ob):
        tob = {}
        tob["ob"] = torch_float(ob["ob"], device=cfg.alg.device).unsqueeze(dim=1)
        tob["state"] = torch_float(ob["state"], device=cfg.alg.device).unsqueeze(dim=1)
        return tob

    @torch.no_grad()
    def get_action(self, ob, sample=True, hidden_state=None, get_action_only=False, evaluation=False, *args, **kwargs):
        if evaluation:
            return self.get_action_student(ob, sample=sample, hidden_state=hidden_state, get_action_only=get_action_only)

        self.eval_mode()
        t_ob = self.extract_student_state(ob)

        if self.deactivate_rnn and hidden_state is not None:
            hidden_state = torch.zeros_like(hidden_state)

        act_dist, val, out_hidden_state = self.actor(x=t_ob,
                                                   hidden_state=hidden_state,
                                                   done=None)

        if self.deactivate_rnn:
            out_hidden_state = torch.zeros_like(out_hidden_state)

        t_ob_exp = self.extract_expert_state(ob)
        #print(t_ob_exp["ob"].shape)
        if self.expert_hidden_state is not None:
            ret = self.expert_actor(t_ob_exp, hidden_state=self.expert_hidden_state)
        else:
            ret = self.expert_actor(t_ob_exp)

        exp_act_dist = ret[0]
        if len(ret) == 3: # teacher is an rnn
            self.expert_hidden_state = ret[2]

        action = action_from_dist(exp_act_dist,
                                  sample=sample)

        student_action = action_from_dist(act_dist, sample=sample)

        #print(action, student_action)


        if not get_action_only:
            log_prob = action_log_prob(action, act_dist)
            entropy = action_entropy(act_dist, log_prob)
            in_hidden_state = torch_to_np(hidden_state) if hidden_state is not None else hidden_state
            action_info = dict(
                log_prob=torch_to_np(log_prob),
                entropy=torch_to_np(entropy),
                val=val,
                in_hidden_state=in_hidden_state
            )

            action_info['exp_act_loc'] = exp_act_dist.base_dist.loc
            action_info['exp_act_scale'] = exp_act_dist.base_dist.scale
        else:
            action_info = dict()
        return torch_to_np(action.squeeze(1)), action_info, out_hidden_state

    def get_action_student(self, ob, sample=True, hidden_state=None, get_action_only=False, *args, **kwargs):
        self.eval_mode()
        t_ob = self.extract_student_state(ob)
        #print(expert_ob.keys(), t_ob["ob"].shape, t_ob["state"].shape, ob["expert_ob"].shape, ob["expert_state"].shape)

        #print(torch.min(t_ob["ob"]), torch.max(t_ob["ob"]), torch.min(t_ob["state"]), torch.max(t_ob["state"]))

        if self.deactivate_rnn and hidden_state is not None:
            hidden_state = torch.zeros_like(hidden_state)

        act_dist, val, out_hidden_state = self.actor(x=t_ob,
                                                   hidden_state=hidden_state,
                                                   done=None)
        if self.deactivate_rnn:
            out_hidden_state = torch.zeros_like(out_hidden_state)
        

        #print(t_ob.keys(), t_ob["ob"].shape, t_ob["state"].shape)
        action = action_from_dist(act_dist,
                                  sample=sample)

        #print(act_dist.base_dist.loc, act_dist.base_dist.scale)

        if not get_action_only:
            log_prob = action_log_prob(action, act_dist)
            entropy = action_entropy(act_dist, log_prob)
            
            in_hidden_state = torch_to_np(hidden_state) if hidden_state is not None else hidden_state
            action_info = dict(
                log_prob=torch_to_np(log_prob),
                entropy=torch_to_np(entropy),
                #val=torch_to_np(val),
                val=val,
                in_hidden_state=in_hidden_state
            )
            
            #action_info['exp_act_cont_loc'] = exp_act_dist_cont.base_dist.loc
            #action_info['exp_act_cont_scale'] = exp_act_dist_cont.base_dist.scale
            #action_info['exp_act_disc_loc'] = exp_act_dist_disc.base_dist.loc
            #action_info['exp_act_disc_scale'] = exp_act_dist_disc.base_dist.scale
            #action_info['exp_act_loc'] = exp_act_dist_cont.base_dist.loc
            #action_info['exp_act_scale'] = exp_act_dist_cont.base_dist.scale
            #action_info['exp_act_logits'] = exp_act_dist_disc.logits
            #action_info['expert_in_hidden_state'] = self.expert_hidden_state
        else:
            action_info = dict()

        expert_action, expert_action_info, expert_hidden_state = self.get_action(ob, sample=sample, hidden_state=hidden_state, get_action_only=get_action_only)

        #print(expert_action, action)
        #return expert_action, expert_action_info, expert_hidden_state

        #return torch_to_np(action.squeeze(1)), action_info, out_hidden_state
        return torch_to_np(action.squeeze(1)), action_info, out_hidden_state

    def optim_preprocess(self, data):
        self.train_mode()
        for key, val in data.items():
            if val is not None:
                data[key] = torch_float(val, device=cfg.alg.device)
        ob = data['ob']
        state = data['state']
        action = data['action']
        #ret = data['ret']
        #adv = data['adv']
        old_log_prob = data['log_prob']
        #old_val = data['val']
        exp_act_loc = data['exp_act_loc']
        exp_act_scale = data['exp_act_scale']
        hidden_state = data['hidden_state']
        hidden_state = hidden_state.permute(1, 0, 2)
        done = data['done']

        act_dist, _, out_hidden_state = self.actor(x={'ob': ob, 'state': state},
                                                   hidden_state=hidden_state,
                                                   done=done)

        #print("STUDENTLOC, EXPLOC", act_dist.base_dist.loc, exp_act_loc)

        log_prob = action_log_prob(action, act_dist)
        entropy = action_entropy(act_dist)
        processed_data = dict(
            #val=val,
            #old_val=old_val,
            #ret=ret,
            log_prob=log_prob,
            old_log_prob=old_log_prob,
            #adv=adv,
            act_dist=act_dist,
            exp_act_loc=exp_act_loc,
            exp_act_scale=exp_act_scale,
            entropy=entropy
        )
        return processed_data

    @torch.no_grad()
    def get_val(self, ob, *args, **kwargs):
        return None, None