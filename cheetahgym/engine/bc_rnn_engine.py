import numpy as np
from torch.utils.data import DataLoader

from easyrl.configs import cfg
from easyrl.engine.ppo_engine import PPOEngine
from easyrl.utils.torch_util import DictDataset
from easyrl.utils.torch_util import torch_to_np

from cheetahgym.engine.bc_engine import BCEngine


class BCRNNEngine(BCEngine):
    def __init__(self, agent, runner):
        super().__init__(agent=agent,
                         runner=runner)

    def traj_preprocess(self, traj):
        action_infos = traj.action_infos
        #vals = np.array([ainfo['val'].flatten() for ainfo in action_infos])
        #print('vs', vals.shape)
        log_prob = np.array([ainfo['log_prob'] for ainfo in action_infos])
        hidden_state = action_infos[0]['in_hidden_state']
        #expert_hidden_state = action_infos[0]['expert_in_hidden_state']
        if hidden_state is not None:
            hidden_state = hidden_state.swapaxes(0, 1)
        else:
            hidden_state_shape = self.runner.hidden_state_shape
            hidden_state = np.zeros((log_prob.shape[1], hidden_state_shape[0], hidden_state_shape[2]))

        #if expert_hidden_state is not None:
        #    expert_hidden_state = expert_hidden_state.swapaxes(0, 1)
        #else:
        #    expert_hidden_state_shape = self.runner.expert_hidden_state_shape
        #    expert_hidden_state = np.zeros((log_prob.shape[1], expert_hidden_state_shape[0], expert_hidden_state_shape[2]))


        #adv = None#self.cal_advantages(traj)
        #ret = None#adv + vals
        #if cfg.alg.normalize_adv:
        #    adv = adv.astype(np.float64)
        #    adv = (adv - np.mean(adv)) / (np.std(adv) + 1e-8)
        # TxN --> NxT
        data = dict(
            ob=traj.obs.swapaxes(0, 1),
            state=traj.states.swapaxes(0, 1),
            action=traj.actions.swapaxes(0, 1),
            #ret=ret,#.swapaxes(0, 1),
            #adv=adv,#.swapaxes(0, 1),
            log_prob=log_prob.swapaxes(0, 1),
            #val=vals.swapaxes(0, 1),
            done=traj.step_extras.swapaxes(0, 1), # we use the mask here instead of true_done
            hidden_state=hidden_state,
            #expert_hidden_state=expert_hidden_state,
        )

        if 'exp_act_logits' in action_infos[0]: # hybrid action space
            #print(action_infos[0]['exp_act_logits'])
            exp_act_logits = np.array([torch_to_np(ainfo['exp_act_logits']) for ainfo in action_infos])
            data["exp_act_logits"] = exp_act_logits.swapaxes(0, 1)

        exp_act_loc = np.array([torch_to_np(ainfo['exp_act_loc']) for ainfo in action_infos])
        exp_act_scale = np.array([torch_to_np(ainfo['exp_act_scale']) for ainfo in action_infos])
        data["exp_act_loc"] = exp_act_loc.swapaxes(0, 1)
        data["exp_act_scale"] = exp_act_scale.swapaxes(0, 1)

        rollout_dataset = DictDataset(**data)
        rollout_dataloader = DataLoader(rollout_dataset,
                                        batch_size=cfg.alg.batch_size,
                                        shuffle=True)
        return rollout_dataloader
