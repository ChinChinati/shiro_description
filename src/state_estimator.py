#!/usr/bin/env python3
import rospy
from gazebo_msgs.msg import LinkStates
from sensor_msgs.msg import JointState
from joint_controller import JointsPublisher
from std_msgs.msg import Float64
import numpy as np
import torch
import time

torch.set_printoptions(threshold=2500)

def quat_rotate_inverse(q, v):
    shape = q.shape
    q_w = q[:, -1]
    q_vec = q[:, :3]
    a = v * (2.0 * q_w ** 2 - 1.0).unsqueeze(-1)
    b = torch.cross(q_vec, v, dim=-1) * q_w.unsqueeze(-1) * 2.0
    c = q_vec * \
        torch.bmm(q_vec.view(shape[0], 1, 3), v.view(
            shape[0], 3, 1)).squeeze(-1) * 2.0
    return a - b + c

class Shiro():
    def __init__(self):
        rospy.init_node('Shiro', anonymous=False)
        self.pub = JointsPublisher()
        for i in range(200):
            self.pub.PublishJoints([[0.1,-0.2,0.9,
                                    -0.1,0.2,-0.9,
                                    -0.1,0.2,-0.9,
                                    0.1,-0.2,0.9]])
            self.pub.rate.sleep()
        self.rate = rospy.Rate(50) 
        self.time = time.time()
        
        self.action_scale = 0.25
        self.hip_scale_reduction = 0.5
        
        
        #Gravity Vector
        self.base_quat = torch.tensor([[0,0,0,0]], dtype=torch.float)
        self.gravity_vector = torch.tensor([[0, 0, -1]], dtype=torch.float)
        self.projected_gravity = torch.tensor([[0,0,0]], dtype=torch.float)

        #Commands
        self.x_vel_cmd, self.y_vel_cmd, self.yaw_vel_cmd = 1.5, 0.0, 0.0
        self.body_height_cmd = 0.0
        self.step_frequency_cmd = 3.0
        self.gaits = [0.5, 0., 0., 0.5] #best with cmd vel 1.5
        self.footswing_height_cmd = 0.08
        self.pitch_cmd = 0.0
        self.roll_cmd = 0.0
        self.stance_width_cmd = 0.25
        self.commands = torch.tensor([[self.x_vel_cmd, self.y_vel_cmd, self.yaw_vel_cmd, self.body_height_cmd, self.step_frequency_cmd, self.gaits[0],
                                        self.gaits[1], self.gaits[2], self.gaits[3], self.footswing_height_cmd, self.pitch_cmd, self.roll_cmd,
                                        self.stance_width_cmd, 4.2803e-01, 2.1376e-04]], dtype=torch.float)
        self.commands_scale = torch.tensor([2.0000, 2.0000, 0.2500, 2.0000, 1.0000, 1.0000, 1.0000, 1.0000, 1.0000,
                                            0.1500, 0.3000, 0.3000, 1.0000, 1.0000, 1.0000])

        #Joints Pos and Vel
        '''Gazebo order
        - FL_calf_joint     0
        - FL_hip_joint      1
        - FL_thigh_joint    2
        
        - FR_calf_joint     3
        - FR_hip_joint      4
        - FR_thigh_joint    5
        
        - RL_calf_joint     6
        - RL_hip_joint      7
        - RL_thigh_joint    8
        
        - RR_calf_joint     9
        - RR_hip_joint      10
        - RR_thigh_joint    11  '''
        
        
        '''Model Order
        - FL_hip_joint      1
        - FL_thigh_joint    2
        - FL_calf_joint     0
        
        - FR_hip_joint      4
        - FR_thigh_joint    5
        - FR_calf_joint     3
        
        - RL_hip_joint      7
        - RL_thigh_joint    8
        - RL_calf_joint     6
        
        - RR_hip_joint      10
        - RR_thigh_joint    11 
        - RR_calf_joint     9
        '''
        self.default_dof_pos = torch.tensor([[ 0.1000, -0.2000,  0.9000, -0.1000,  0.2000, -0.9000, -0.1000,  0.2000,
                                                -0.9000,  0.1000, -0.2000,  0.9000]])
        self.dof_pos = torch.tensor([[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.dof_vel = torch.tensor([[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.obs_scales_dof_vel = 0.05
        
        #Actions
        self.actions = torch.tensor([[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        self.last_actions = torch.tensor([[ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]])
        
        # Clock Indices
        self.dt = 0.02
        self.num_envs = 1
        self.clock_inputs = torch.zeros(self.num_envs, 4, dtype=torch.float)
        self.gait_indices = torch.tensor([0.])

        # Noise Scale vec
        self.noise_scale_vec = torch.tensor([0.0500, 0.0500, 0.0500, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                0.0100, 0.0100, 0.0100, 0.0100, 0.0100, 0.0100, 0.0100, 0.0100, 0.0100,
                                                0.0100, 0.0100, 0.0100, 0.0750, 0.0750, 0.0750, 0.0750, 0.0750, 0.0750,
                                                0.0750, 0.0750, 0.0750, 0.0750, 0.0750, 0.0750, 0.0000, 0.0000, 0.0000,
                                                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000,
                                                0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0000])
        # Observation History
        self.obs_history_length = 30
        self.num_obs = 70
        self.num_envs = 1
        self.num_obs_history = self.obs_history_length * self.num_obs
        self.obs_history = torch.zeros(self.num_envs, self.num_obs_history, dtype=torch.float,
                                       requires_grad=False)
        
        # Lag buffer
        self.lag_buffer = [torch.zeros_like(self.dof_pos) for i in range(7)]
        
        #Callback Functions
        def link_state_cb(data:LinkStates):
            self.base_quat = torch.tensor([[data.pose[1].orientation.x, data.pose[1].orientation.y, data.pose[1].orientation.z, data.pose[1].orientation.w]], dtype=torch.float)
            self.projected_gravity =  quat_rotate_inverse(self.base_quat, self.gravity_vector)
        
        def joint_state_cb(data:JointState):
            self.dof_pos = torch.tensor([[data.position[1],data.position[2],data.position[0],
                                         data.position[4],data.position[5],data.position[3],
                                         data.position[7],data.position[8],data.position[6],
                                         data.position[10],data.position[11],data.position[9]]])
            
            self.dof_vel = torch.tensor([[data.velocity[1],data.velocity[2],data.velocity[0],
                                         data.velocity[4],data.velocity[5],data.velocity[3],
                                         data.velocity[7],data.velocity[8],data.velocity[6],
                                         data.velocity[10],data.velocity[11],data.velocity[9]]])
            
            #   self.dof_pos = torch.tensor([[data.position[1],data.position[2],data.position[0],
            #                              data.position[4],data.position[5],data.position[3],
            #                              data.position[7],data.position[8],data.position[6],
            #                              data.position[10],data.position[11],data.position[9]]])
            
            #   self.dof_vel = torch.tensor([[data.velocity[1],data.velocity[2],data.velocity[0],
            #                              data.velocity[4],data.velocity[5],data.velocity[3],
            #                              data.velocity[7],data.velocity[8],data.velocity[6],
            #                              data.velocity[10],data.velocity[11],data.velocity[9]]])
        
        #Subscribers
        self.link_statets= rospy.Subscriber('/gazebo/link_states', LinkStates, callback=link_state_cb)
        self.joint_statets= rospy.Subscriber('/shiro/joint_states', JointState, callback=joint_state_cb)
        # while 1:
        #     rospy.loginfo(self.dof_pos)
        
                
    #Clock_inputs
    def compute_clock_inputs(self):
        frequencies = self.commands[:, 4]
        phases = self.commands[:, 5]
        offsets = self.commands[:, 6]
        bounds = self.commands[:, 7]
        durations = self.commands[:, 8]
        
        self.gait_indices = torch.remainder(self.gait_indices + self.dt * frequencies, 1.0)
        
        foot_indices = [self.gait_indices + phases + offsets + bounds,
                            self.gait_indices + offsets,
                            self.gait_indices + bounds,
                            self.gait_indices + phases]
        
        self.foot_indices = torch.remainder(torch.cat([foot_indices[i].unsqueeze(1) for i in range(4)], dim=1), 1.0)
        
        self.clock_inputs[:, 0] = torch.sin(2 * np.pi * foot_indices[0])
        self.clock_inputs[:, 1] = torch.sin(2 * np.pi * foot_indices[1])
        self.clock_inputs[:, 2] = torch.sin(2 * np.pi * foot_indices[2])
        self.clock_inputs[:, 3] = torch.sin(2 * np.pi * foot_indices[3])

        return self.clock_inputs

    def compute_observation(self,actions):
        self.actions = actions
        self.obs_buf = torch.cat((self.projected_gravity, 
                                  self.commands * self.commands_scale,
                                  self.dof_pos - self.default_dof_pos,
                                  self.dof_vel * self.obs_scales_dof_vel,
                                  self.actions
                                  ), dim=-1)
        
        self.obs_buf = torch.cat((self.obs_buf,
                                    self.last_actions), dim=-1)
        
        self.compute_clock_inputs()
        self.obs_buf = torch.cat((self.obs_buf,
                                      self.clock_inputs), dim=-1)
        
        self.obs_buf += (2 * torch.rand_like(self.obs_buf) - 1) * self.noise_scale_vec

        # Formatting
        privileged_obs = torch.tensor([[1. , -1.]], dtype=torch.float)
        self.obs_history = torch.cat((self.obs_history[:, self.num_obs:], self.obs_buf), dim=-1)
        obs = {'obs': self.obs_buf, 'privileged_obs': privileged_obs, 'obs_history': self.obs_history}
        
        self.last_actions = self.actions
        self.rate.sleep()
        return obs
    
    def step(self, actions, hard_reset=False):
        clip_actions = 100
        # self.last_actions = self.actions[:]
        self.actions = torch.clip(actions[0:1, :], -clip_actions, clip_actions)
        # self.publish_action(self.actions, hard_reset=hard_reset)
        time.sleep(max(self.dt - (time.time() - self.time), 0))
        # if self.timestep % 100 == 0: print(f'frq: {1 / (time.time() - self.time)} Hz');
        self.time = time.time()
        
        actions_scaled = actions[:, :12] * self.action_scale
        actions_scaled[:, [0, 3, 6, 9]] *= self.hip_scale_reduction
        
        # rospy.loginfo(f'scaled \n{actions_scaled*180/3.14}')
        # rospy.loginfo(f'unscaled \n{self.lag_buffer}')
        
        #Adding Lag Buffer
        self.lag_buffer = self.lag_buffer[1:] + [actions_scaled.clone()]
        self.joint_pos_target = self.lag_buffer[0] + self.default_dof_pos
        
        # Without Lag Buffer
        # self.joint_pos_target = actions_scaled + self.default_dof_pos
        
        self.joint_pos_target = self.joint_pos_target.detach().cpu().numpy()
        self.pub.PublishJoints(self.joint_pos_target)
        self.pub.rate.sleep()
    

    
