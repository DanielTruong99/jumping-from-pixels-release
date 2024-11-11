import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical, Independent, Normal 

'''
import numpy as np
import matplotlib
try:
    matplotlib.use("GTK3Agg")
except Exception:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
'''

'''
class RNNGaitParamNet(nn.Module):
    def __init__(self, input_dim=58, 
                       num_stack=1, 
                       im_height=32, 
                       im_width=32, 
                       use_one_hot_embedding=False, 
                       num_discrete_actions=10, 
                       cfg=None):
        # input height = width = 32
        super().__init__()
        self.input_dim = input_dim
        self.num_stack = cfg.num_stack
        self.im_width = cfg.im_width
        self.im_height = cfg.im_height
        self.use_one_hot_embedding = cfg.use_onehot_obs
        self.num_discrete_actions = num_discrete_actions
        self.cfg = cfg

        if self.use_one_hot_embedding:
            self.offsets_embedding = nn.Euse_mbedding(self.num_discrete_actions, 10)
            self.durations_embedding = nn.Embedding(self.num_discrete_actions, 10)
            self.iterations_embedding = nn.Embedding(self.num_discrete_actions, 10)
        
        self.convs = nn.Sequential(
            nn.Conv2d(in_channels=self.num_stack, out_channels=16, kernel_size=(4, 5), stride=2),
            nn.ReLU(),
            nn.Conv2d(in_channels=16, out_channels=32, kernel_size=(3, 4), stride=2),
            nn.ReLU(),
            nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(2, 3), stride=1),
            nn.ReLU()
        )
        
        if self.cfg.im_width == 48:
            self.fcs1 = nn.Sequential(
                nn.Linear(512, 128),
                #nn.Linear(32*6*6, 128),
                nn.ReLU(),
            )
        elif self.cfg.im_width == 144:
            self.fcs1 = nn.Sequential(
                nn.Linear(2048, 128),
                #nn.Linear(32*6*6, 128),
                nn.ReLU(),
            )
        elif self.cfg.im_width == 72:
            self.fcs1 = nn.Sequential(
                nn.Linear(896, 128),
                #nn.Linear(32*6*6, 128),
                nn.ReLU(),
            )
        elif self.cfg.im_width == 25:
            self.fcs1 = nn.Sequential(
                nn.Linear(512, 128),
                #nn.Linear(32*6*6, 128),
                nn.ReLU(),
            )

        if self.use_one_hot_embedding:
            self.fcs = nn.Sequential(
                nn.Linear(256, 256),
                nn.ReLU(),
                nn.Linear(256, 128),
                nn.ReLU(),
                #nn.Linear(128, 12),
                #nn.ReLU()
            )
        else:
            self.fcs = nn.Sequential(
                nn.Linear(128+self.input_dim * self.num_stack, 256),
                nn.ReLU(),
                nn.Linear(256, 128),
                nn.ReLU(),
                #nn.Linear(128, 12),
                #nn.ReLU()
            )


    def forward(self, x):
        #if self.num_stack == 1:
        #    batch_size=x.shape[0]
        #    vec = x[:, 0:self.input_dim]
        #    img = x[:, self.input_dim:].reshape(-1, 1, 32, 32)
        #else:
        
        #batch_size = x.shape[0]
       
        state = x["state"]
        img = x["ob"]
        
        if len(img.shape) == 2:
            img = img.reshape(-1, img.shape[0], img.shape[1])
            batch_size=1
        else:
            batch_size=img.shape[0]
        img = img.reshape(img.shape[0], -1, img.shape[1], img.shape[2])

        # pass image through conv
        x = self.convs(img)
        x = torch.flatten(x, start_dim=1)
        x = self.fcs1(x)

        # pass discrete through embedding
        if self.use_one_hot_embedding:
            offs_vec = self.offsets_embedding(torch.cuda.LongTensor(state[:, :, -9:-5].long()))
            durs_vec = self.durations_embedding(torch.cuda.LongTensor(state[:, :,-5:-1].long()))
            iters_vec = self.iterations_embedding(torch.cuda.LongTensor(state[:, :, -1:].long()))
            state = torch.cat((state[:, :, :-9], offs_vec.flatten(start_dim=2), durs_vec.flatten(start_dim=2), iters_vec.flatten(start_dim=2)), dim=2)


        x = torch.cat((x.flatten().reshape(batch_size, -1), state.reshape(batch_size, -1)), axis=1)
        x = self.fcs(x)

        #x_continuous = self.discrete_head(x)
        #x_discrete_mid = self.discrete_head(x)
        #x_discrete = [Categorical(logits=x_discrete_mid[10*i:10*(i+1)]) for i in range(8)]
       # x = [x_continuous] + x_discrete        

        return x
'''

class RNNMixedPolicy(nn.Module):
    def __init__(self,
                 body_net,
                 cont_action_dim,
                 discrete_action_dim,
                 num_discrete_actions,
                 init_log_std=-0.51,
                 std_cond_in=False,
                 tanh_on_dist=False,
                 in_features=None,
                 clamp_log_std=False):  # add tanh on the action distribution
        super().__init__()
        self.cont_action_dim = cont_action_dim
        self.discrete_action_dim = discrete_action_dim
        self.num_discrete_actions = num_discrete_actions
        self.std_cond_in = std_cond_in
        self.tanh_on_dist = tanh_on_dist
        self.body = body_net
        self.clamp_log_std = clamp_log_std

        if in_features is None:
            for i in reversed(range(len(self.body.fcs))):
                layer = self.body.fcs[i]
                if hasattr(layer, 'out_features'):
                    in_features = layer.out_features
                    break

        self.head_mean_cont = nn.Linear(in_features, self.cont_action_dim)
        if self.std_cond_in:
            self.head_logstd = nn.Linear(in_features, self.cont_action_dim)
        else:
            self.head_logstd = nn.Parameter(torch.full((self.cont_action_dim,),
                                                       init_log_std))

        self.head_discrete = nn.Linear(in_features, self.discrete_action_dim * self.num_discrete_actions)

    def forward(self, x=None, body_x=None, hidden_state=None, **kwargs):
        # body part

        if x is None and body_x is None:
            raise ValueError('One of [x, body_x] should be provided!')
        if body_x is None:
            body_x, hidden_state = self.body(x,
                                             hidden_state=hidden_state,
                                             **kwargs)
        
        # continuous part
        #print(body_x)
        #body_out = body_x[:, 0, :] if len(body_x.shape) == 3 else body_x
        b = body_x.shape[0]
        t = body_x.shape[1]
        body_out = body_x.view(b*t, -1) if t == 1 else body_x
        #body_out = body_x[0] if isinstance(body_x, tuple) else body_x
        #print('shapecontinp', body_out.shape)
        #print('bxs', body_x.shape)
        mean_cont = self.head_mean_cont(body_out)
        #print('shapecontinp', mean_cont.shape)
        if self.std_cond_in:
            log_std = self.head_logstd(body_out)
        else:
            log_std = self.head_logstd.expand_as(mean_cont)
        if self.clamp_log_std:
            log_std = torch.clamp(log_std, LOG_STD_MIN, LOG_STD_MAX)
        std = torch.exp(log_std)
        action_dist_cont = Independent(Normal(loc=mean_cont, scale=std), 1)
        if self.tanh_on_dist:
            action_dist_cont = TransformedDistribution(action_dist_cont,
                                                  [TanhTransform(cache_size=1)])
        # discrete part
        if t == 1:
            discrete_x_inp = self.head_discrete(body_x).reshape(-1, self.num_discrete_actions, self.discrete_action_dim)
        else:
            discrete_x_inp = self.head_discrete(body_x).reshape(b, t, self.num_discrete_actions, self.discrete_action_dim)
        #print('shapecatinp', discrete_x_inp.shape)
        action_dist_discrete = Categorical(logits=discrete_x_inp)

        #print('shapes', action_dist_cont, action_dist_discrete)

        return action_dist_cont, action_dist_discrete, body_x, hidden_state


