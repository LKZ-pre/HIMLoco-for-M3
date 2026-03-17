# SPDX-FileCopyrightText: Copyright (c) 2021 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Copyright (c) 2021 ETH Zurich, Nikita Rudin

from legged_gym.envs.base.legged_robot_config import LeggedRobotCfg,LeggedRobotCfgPPO

class M3RoughCfg(LeggedRobotCfg):

    # 训练
    class env(LeggedRobotCfg.env):
        num_envs = 100
        num_one_step_observations = 3 + 3 + 3 + 16 + 16 + 16
        num_observations = num_one_step_observations * 6
        num_one_step_privileged_obs = num_one_step_observations + 3 + 3 + 11 * 17 + 12
        num_privileged_obs = num_one_step_privileged_obs * 1
        num_actions = 16          
    
    # 地形
    class terrain(LeggedRobotCfg.terrain):
        mesh_type = 'trimesh' # "heightfield" # none, plane, heightfield or trimesh
        static_friction  = 0.8  # 静摩擦
        dynamic_friction = 0.8  # 动摩擦
        # 地形: [光滑坡, 粗糙坡, 上楼梯, 下楼梯, 随机离散地形]
        # terrain types: [smooth slope, rough slope, stairs up, stairs down, discrete]
        terrain_proportions = [0.1, 0.1, 0.35, 0.2, 0.25]


    # 指令
    class commands(LeggedRobotCfg.commands):
        curriculum = True
        max_curriculum = 1.5
        num_commands = 4 # default: lin_vel_x, lin_vel_y, ang_vel_yaw, heading (in heading mode ang_vel_yaw is recomputed from heading error)
        resampling_time = 10. # time before command are changed[s]
        heading_command = True # if true: compute ang vel command from heading error
        class ranges:
            lin_vel_x = [-1.0, 1.0]      # min max [m/s] 
            lin_vel_y = [-0.6, 0.6]      # min max [m/s] 
            ang_vel_yaw = [-1.0, 1.0]  # min max [rad/s] 
            heading = [-3.14, 3.14] 

    # 初始状态
    class init_state( LeggedRobotCfg.init_state ):
        pos = [0.0, 0.0, 0.45] # x,y,z [m] 
        default_joint_angles = {  # = target angles [rad] when action = 0.0 [rad]
       'FL_hip_joint': 0.0,
       'RL_hip_joint': 0.0,
       'FR_hip_joint': 0.0,
       'RR_hip_joint': 0.0,

       'FL_thigh_joint': 1.0,
       'RL_thigh_joint': 1.0,
       'FR_thigh_joint': 1.0,
       'RR_thigh_joint': 1.0,

       'FL_calf_joint': -1.5,
       'RL_calf_joint': -1.5,
       'FR_calf_joint': -1.5,
       'RR_calf_joint': -1.5,

       'FL_foot_joint': 0.0,
       'RL_foot_joint': 0.0,
       'FR_foot_joint': 0.0,
       'RR_foot_joint': 0.0,}

    # 执行
    class control( LeggedRobotCfg.control ):
        # PD Drive parameters:
        control_type = 'P' 
        stiffness = {'hip_joint': 40.,'thigh_joint': 40.,'calf_joint': 40.,"foot_joint":0}  
        damping =   {'hip_joint': 1,'thigh_joint': 1,'calf_joint': 1,"foot_joint":0.5}     
        action_scale = 0.25
        vel_scale = 10.0
        decimation = 4
        wheel_speed = 1

    # URDF
    class asset( LeggedRobotCfg.asset ):
        file = '{LEGGED_GYM_ROOT_DIR}/resources/robots/M3/urdf/M3.urdf'
        name = "M3"
        foot_name = "foot"
        wheel_name =["foot"] 
        penalize_contacts_on = ["thigh", "calf", "base"] 
        terminate_after_contacts_on = ['base']
        priviledge_contacts_on = ["thigh", "calf", "base"] 
        self_collisions = 1 
        replace_cylinder_with_capsule = False 
        flip_visual_attachments = False
    
    #域随机化
    class domain_rand:
        randomize_payload_mass = True
        payload_mass_range = [-1, 2]

        randomize_com_displacement = True
        com_displacement_range = [-0.1, 0.1]

        randomize_link_mass = True
        link_mass_range = [0.8, 1.2]
        
        randomize_friction = True
        friction_range = [0.25, 2.25]
        
        randomize_restitution = True
        restitution_range = [0., 1.0]
        
        randomize_motor_strength = True
        motor_strength_range = [0.8, 1.2]
        
        randomize_kp = True
        kp_range = [0.8, 1.2]
        
        randomize_kd = True
        kd_range = [0.8, 1.2]
        
        randomize_initial_joint_pos = True
        initial_joint_pos_range = [0.5, 1.5]
        
        disturbance = True
        disturbance_range = [-30.0, 30.0]
        disturbance_interval = 8
        
        push_robots = True
        push_interval_s = 15
        max_push_vel_xy = 1.

        delay = True
    
    # 奖励函数
    class rewards( LeggedRobotCfg.rewards ):
        class scales:

            tracking_lin_vel = 1.5
            tracking_ang_vel = 0.75
            lin_vel_z = -1.0
            ang_vel_xy = -0.05
            orientation = -0.5
            base_height = -10.0
            hip_default = -0.5 
            stand_still = -0.5
            collision = -1.0
            feet_stumble = -0.1
            action_rate = -0.01
            torques = -5.0e-4
            dof_vel = -1e-7
            dof_acc = -1e-7
            run_still = -0.05

        only_positive_rewards = True # if true negative total rewards are clipped at zero (avoids early termination problems)
        tracking_sigma = 0.25 # tracking reward = exp(-error^2/sigma)
        soft_dof_pos_limit = 1. # percentage of urdf limits, values above this limit are penalized
        soft_dof_vel_limit = 1.
        soft_torque_limit = 1.
        base_height_target = 0.4
        max_contact_force = 100. # forces above this value are penalized
       
       
class M3RoughCfgPPO( LeggedRobotCfgPPO ):
    class algorithm( LeggedRobotCfgPPO.algorithm ):
        entropy_coef = 0.005
    class runner( LeggedRobotCfgPPO.runner ):
        save_interval = 2500 # check for potential saves every this many iterations
        num_steps_per_env = 48 # per iteration
        max_iterations = 10000
        # load and resume
        experiment_name = 'M3'
        run_name = ''
        resume = None
        load_run = -1 # -1 = last run
        checkpoint = -1 # -1 = last saved model
        resume_path = None # updated from load_run and chkpt

  
