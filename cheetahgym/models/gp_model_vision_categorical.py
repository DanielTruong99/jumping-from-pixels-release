import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.distributions import Categorical, Independent, Normal 


import numpy as np
import matplotlib
try:
    matplotlib.use("GTK3Agg")
    import matplotlib.pyplot as plt
except Exception:
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt



class GaitParamNet(nn.Module):
    def __init__(self, input_dim=58, num_stack=1, im_height=32, im_width=32, use_one_hot_embedding=False, num_discrete_actions=10, cfg=None, lcm_publisher=None):
        # input height = width = 32
        super().__init__()
        #print("input dim:", input_dim)
        self.input_dim = input_dim
        self.num_stack = cfg.num_stack
        self.im_width = cfg.im_width
        self.im_height = cfg.im_height
        self.use_one_hot_embedding = cfg.use_onehot_obs
        self.num_discrete_actions = num_discrete_actions
        self.cfg = cfg
        self.lcm_publisher = lcm_publisher

        if self.use_one_hot_embedding:
            self.offsets_embedding = nn.Embedding(self.num_discrete_actions, 10)
            self.durations_embedding = nn.Embedding(self.num_discrete_actions, 10)
            self.iterations_embedding = nn.Embedding(self.num_discrete_actions, 10)
        
        if not self.cfg.use_vision or self.cfg.observe_gap_state:
            self.convs = None

        elif self.cfg.use_resnet:
            import torchvision
            self.convs = torchvision.models.resnet18(pretrained=False, progress=False)
            self.convs.conv1 = torch.nn.Conv2d(1, 64, kernel_size=(7, 7), stride=(2, 2), padding=(3, 3), bias=False)
            #self.convs.fc = torch.nn.Linear(128, 1)

        elif self.cfg.use_raw_depth_image or self.cfg.use_grayscale_image:
            if cfg.depth_cam_width == 224:
                self.convs = nn.Sequential(
                    nn.Conv2d(in_channels=self.num_stack, out_channels=16, kernel_size=(9, 9), stride=3),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=16, out_channels=32, kernel_size=(6, 6), stride=2),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(3, 3), stride=1),
                    nn.ReLU()
                )
            elif cfg.depth_cam_width == 160:
                self.convs = nn.Sequential(
                    nn.Conv2d(in_channels=self.num_stack, out_channels=16, kernel_size=(6, 6), stride=2),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=16, out_channels=32, kernel_size=(5, 5), stride=2),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(3, 3), stride=1),
                    nn.ReLU()
                )
            elif cfg.depth_cam_width == 30:
                self.convs = nn.Sequential(
                    nn.Conv2d(in_channels=self.num_stack, out_channels=16, kernel_size=(5, 5), stride=2),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=16, out_channels=32, kernel_size=(4, 4), stride=2),
                    nn.ReLU(),
                    nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(3, 3), stride=1),
                    nn.ReLU()
                )
        else: # heightmap dimension
            self.convs = nn.Sequential(
                nn.Conv2d(in_channels=self.num_stack, out_channels=16, kernel_size=(4, 5), stride=2),
                nn.ReLU(),
                nn.Conv2d(in_channels=16, out_channels=32, kernel_size=(3, 4), stride=2),
                nn.ReLU(),
                nn.Conv2d(in_channels=32, out_channels=64, kernel_size=(2, 3), stride=1),
                nn.ReLU()
            )

        if self.cfg.use_resnet:
            self.fcs1 = nn.Sequential(nn.Linear(1000, 128),
                                      nn.ReLU())
        
        elif self.cfg.use_raw_depth_image or self.cfg.use_grayscale_image:
            if self.cfg.depth_cam_width == 224:
                self.fcs1 = nn.Sequential(
                    nn.Linear(65536, 128),
                    #nn.Linear(32*6*6, 128),
                    nn.ReLU(),
                )
            elif self.cfg.depth_cam_width == 160:
                self.fcs1 = nn.Sequential(
                    nn.Linear(56000, 128),
                    #nn.Linear(32*6*6, 128),
                    nn.ReLU(),
                )
            elif self.cfg.depth_cam_width == 30:
                self.fcs1 = nn.Sequential(
                    nn.Linear(576, 128),
                    #nn.Linear(32*6*6, 128),
                    nn.ReLU(),
                )

        else: # heightmap dimension
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


        if not self.cfg.use_vision or self.cfg.observe_gap_state:
            self.fcs = nn.Sequential(
                nn.Linear(self.input_dim * self.num_stack, 256),
                nn.ReLU(),
                nn.Linear(256, 128),
                nn.ReLU(),
                #nn.Linear(128, 12),
                #nn.ReLU()
            )
        else:

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

        '''
        plt.ion()
        plt.imshow(torch.zeros((32, 32)))
        plt.show()
        '''
        #self.continuous_head = nn.Sequential(
        #    nn.Linear(128, 4)
        #)
        #self.discrete_head = nn.Sequential(
        #    nn.Linear(128, 8*10),
        #)

    def forward(self, x):
        #if self.num_stack == 1:
        #    batch_size=x.shape[0]
        #    vec = x[:, 0:self.input_dim]
        #    img = x[:, self.input_dim:].reshape(-1, 1, 32, 32)
        #else:
        
        #batch_size = x.shape[0]
        '''
        if self.num_stack == 1 and len(x.shape) == 2:
             x = x.reshape(x.shape[0], 1, x.shape[1])
        vec = x[:, :, 0:self.input_dim]
        img = x[:, :, self.input_dim:].reshape(-1, self.num_stack, self.im_height, self.im_width)
        '''

        '''
        plt.imshow(x["ob"][0, :, :].cpu().numpy())
        plt.draw()
        print("show ob")
        plt.pause(0.01)
        print(np.min(x["ob"][0, :, :].cpu().numpy()), np.max(x["ob"][0, :, :].cpu().numpy()))
        input()
        '''
        state = x["state"]
        img = x["ob"]

        if self.cfg.use_vision and not self.cfg.observe_gap_state:    
            #print('ingp', img.shape)

            if len(img.shape) == 2:
                img = img.reshape(-1, img.shape[0], img.shape[1])
                batch_size=1
            elif len(img.shape) == 3:
                batch_size=img.shape[0]
                img = img.reshape(img.shape[0], -1, img.shape[1], img.shape[2])
            elif len(img.shape) == 4:
                batch_size = img.shape[0]
            #elif len(img.shape) == 4:
            #    batch_size = img.shape[1]
            #    img = img.reshape(img.shape[])

            
            x = self.convs(img)
            #if not self.cfg.use_resnet:
            x = torch.flatten(x, start_dim=1)
            #print(x.shape)
            x = self.fcs1(x)

            if self.lcm_publisher is not None:
                self.lcm_publisher.broadcast_cnn_features(x.detach().numpy())

        # pass discrete through embedding
        if self.use_one_hot_embedding:
            offs_vec = self.offsets_embedding(torch.cuda.LongTensor(state[:, :, -9:-5].long()))
            durs_vec = self.durations_embedding(torch.cuda.LongTensor(state[:, :,-5:-1].long()))
            iters_vec = self.iterations_embedding(torch.cuda.LongTensor(state[:, :, -1:].long()))
            state = torch.cat((state[:, :, :-9], offs_vec.flatten(start_dim=2), durs_vec.flatten(start_dim=2), iters_vec.flatten(start_dim=2)), dim=2)

        if self.cfg.use_vision and not self.cfg.observe_gap_state:
            x = torch.cat((x.flatten().reshape(batch_size, -1), state.reshape(batch_size, -1)), axis=1)
        else:
            x = state


        #print(x.shape)
        x = self.fcs(x)

        #x_continuous = self.discrete_head(x)
        #x_discrete_mid = self.discrete_head(x)
        #x_discrete = [Categorical(logits=x_discrete_mid[10*i:10*(i+1)]) for i in range(8)]
       # x = [x_continuous] + x_discrete        

        return x


class MixedPolicy(nn.Module):
    def __init__(self,
                 body_net,
                 cont_action_dim,
                 discrete_action_dim,
                 num_discrete_actions,
                 init_log_std=-0.51,
                 std_cond_in=False,
                 tanh_on_dist=False,
                 in_features=None):  # add tanh on the action distribution
        super().__init__()
        self.cont_action_dim = cont_action_dim
        self.discrete_action_dim = discrete_action_dim
        self.num_discrete_actions = num_discrete_actions
        self.std_cond_in = std_cond_in
        self.tanh_on_dist = tanh_on_dist
        self.body = body_net

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

    def forward(self, x=None, body_x=None, **kwargs):
        # body part
        #if body_x is not None: print(f'max, min, isnan body_x: {torch.max(body_x["state"]), torch.min(body_x["state"]), torch.isnan(torch.sum(body_x["state"]))}')
        #if x is not None: print(f'max, min, isnan x: {torch.max(x["state"]), torch.min(x["state"]), torch.isnan(torch.sum(x["state"]))}')

        if x is None and body_x is None:
            raise ValueError('One of [x, body_x] should be provided!')
        if body_x is None:
            body_x = self.body(x, **kwargs)
        
        # continuous part
        #print('bxs', body_x.shape)
        mean_cont = self.head_mean_cont(body_x)
        if self.std_cond_in:
            log_std = self.head_logstd(body_x)
        else:
            log_std = self.head_logstd.expand_as(mean_cont)
        std = torch.exp(log_std)
        
        #print(f'max, min, isnan mean_cont: {torch.max(mean_cont), torch.min(mean_cont), torch.isnan(torch.sum(mean_cont))}')
        #print(f'max, min, isnan std: {torch.max(std), torch.min(std), torch.isnan(torch.sum(std))}')


        action_dist_cont = Independent(Normal(loc=mean_cont, scale=std), 1)
        if self.tanh_on_dist:
            action_dist_cont = TransformedDistribution(action_dist_cont,
                                                  [TanhTransform(cache_size=1)])
        # discrete part
        discrete_x_inp = self.head_discrete(body_x).reshape(-1, self.num_discrete_actions, self.discrete_action_dim)
        action_dist_discrete = Categorical(logits=discrete_x_inp)

        return action_dist_cont, action_dist_discrete, body_x


