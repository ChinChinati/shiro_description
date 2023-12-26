#!/usr/bin/env python3
from state_estimator import Shiro
import pickle as pkl
import torch
import numpy as np
import rospy

def load_policy(logdir):
    body = torch.jit.load(logdir + '/checkpoints/body_latest.jit', map_location=torch.device('cpu'))
    import os
    adaptation_module = torch.jit.load(logdir + '/checkpoints/adaptation_module_latest.jit', map_location=torch.device('cpu'))

    def policy(obs, info):
        i = 0
        latent = adaptation_module.forward(obs["obs_history"].to('cpu'))
        action = body.forward(torch.cat((obs["obs_history"].to('cpu'), latent), dim=-1))
        info['latent'] = latent
        return action

    return policy


logdir = "/home/chinchinati/catkin_ws/src/shiro_description/model/125648.167244"

with open(logdir+"/parameters.pkl", 'rb') as file:
        pkl_cfg = pkl.load(file)
        print(pkl_cfg.keys())
        cfg = pkl_cfg["Cfg"]
        print(cfg.keys())

# This is policy
policy = load_policy(logdir) 

# Env
env = Shiro()

# Initially
actions = torch.tensor([[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])


        
policy_info = {}
while not rospy.is_shutdown():
    obs = env.compute_observation(actions)
    actions = policy(obs,policy_info)
    env.step(actions)
    # rospy.loginfo(f'Obs:  {obs}')
    # rospy.loginfo(f'Actions:  {actions}')