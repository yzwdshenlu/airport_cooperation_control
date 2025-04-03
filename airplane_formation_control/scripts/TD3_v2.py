#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 使用位置和方向作为状态空间，用于单智能体路径规划

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import BaseCallback
import gym
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import *
from std_srvs.srv import Empty
from tqdm import tqdm
from gym import spaces
import torch
import gc
import os
import time

# 清理内存
torch.cuda.empty_cache()
gc.collect()

# 定义目标路径点（二维坐标）
PATH_POINTS = [
    (-67, -72), (-72, -98), (-52, -85), (-52, -75),
    (-68, -60), (-90, -60), (-90, -68), (-106, -75),
    (-106, -85), (-86, -96), (-86, -104), (-85, -112)
]

TIME_DELTA = 1
os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
class TrackingEnv(gym.Env):
    def __init__(self):
        super(TrackingEnv, self).__init__()
        # self.action_space = spaces.Box(low=np.array([-2.0, -3.0]), high=np.array([6.0, 3.0]), dtype=np.float32)
        # self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(20 + 4,), dtype=np.float32)
        self.current_point_index = 0
        self.robot_position = (-60, -60)
        self.current_z = -1.57
        self.previous_z = -1.57


      
        rospy.init_node('drl_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/robot0/airport_robot/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback, queue_size=1)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)  
        # Create a SetModelStateRequest object
        objstate = SetModelStateRequest()
        
        objstate.model_state.model_name = "robot0"      
        objstate.model_state.pose.position.x = -60.0      
        objstate.model_state.pose.position.y = -60.0     
        objstate.model_state.pose.position.z = 0.0     
        objstate.model_state.pose.orientation.x = 0.0    
        objstate.model_state.pose.orientation.y = 0.0    
        objstate.model_state.pose.orientation.z = -0.7068252    
        objstate.model_state.pose.orientation.w = 0.7073883
        objstate.model_state.twist.linear.x = 0.0
        objstate.model_state.twist.linear.y = 0.0
        objstate.model_state.twist.linear.z = 0.0
        objstate.model_state.twist.angular.x = 0.0
        objstate.model_state.twist.angular.y = 0.0
        objstate.model_state.twist.angular.z = 0.0

                        
        response = self.set_state_service(objstate)
        if response.success:
            print("Model state set successfully")
        else:
            print("Failed to set model state:", response.status_message)



    def model_state_callback(self, model_states_msg):
        model_index = model_states_msg.name.index('robot0')
        self.current_x = model_states_msg.pose[model_index].position.x
        self.current_y = model_states_msg.pose[model_index].position.y
        self.robot_position = (self.current_x, self.current_y)

        self.orientation_x = model_states_msg.pose[model_index].orientation.x
        self.orientation_y = model_states_msg.pose[model_index].orientation.y
        self.orientation_z = model_states_msg.pose[model_index].orientation.z
        self.orientation_w = model_states_msg.pose[model_index].orientation.w
        quaternion = (
            self.orientation_x,
            self.orientation_y,
            self.orientation_z,
            self.orientation_w
        )
        euler = euler_from_quaternion(quaternion)
        
        self.current_z = euler[2] # 当前偏航角
        
        if np.cos(self.current_z - self.previous_z) < 0.5:
            self.current_z += np.pi


        n = 0
        while self.current_z < -np.pi:
            n += 1
            self.current_z = self.current_z + n * 2 * np.pi
        n = 0
        while self.current_z > np.pi:
            n -= 1
            self.current_z = self.current_z + n * 2 * np.pi

        self.previous_z = self.current_z # 上一帧偏航角


    # def unpause_physics(self):
    #     rospy.wait_for_service('/gazebo/unpause_physics')
    #     try:
    #         unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
    #         unpause()
    #         rospy.loginfo("Unpaused the physics in Gazebo.")
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")


    # def pause_physics(self):
    #     rospy.wait_for_service('/gazebo/pause_physics')
    #     try:
    #         pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
    #         pause()
    #         rospy.loginfo("Paused the physics in Gazebo.")
    #     except rospy.ServiceException as e:
    #         rospy.logerr(f"Service call failed: {e}")

    # 计算距离当前目标点的距离和角度
    def _calculate_PositionAndPose(self):

        # 计算当前位置与目标点的距离
        target_point = PATH_POINTS[self.current_point_index]
        distance = np.linalg.norm(np.array([
            target_point[0] - self.robot_position[0],
            target_point[1] - self.robot_position[1]
        ]))

        # 计算当前位置与目标点的夹角
        robot_to_target = np.array([
            target_point[0] - self.robot_position[0],
            target_point[1] - self.robot_position[1]
        ])
        robot_heading = np.array([np.cos(self.current_z), np.sin(self.current_z)])  # 假设 self.current_z 是机器人的朝向角
        
        # 使用 arctan2 计算夹角
        cross_product = np.cross(robot_heading, robot_to_target)
        dot_product = np.dot(robot_heading, robot_to_target)  
        angle_diff = np.arctan2(cross_product, dot_product) 
        # angle_diff = np.arccos(np.clip(np.dot(robot_to_target, robot_heading) / (np.linalg.norm(robot_to_target) * np.linalg.norm(robot_heading)), -1.0, 1.0))
        angle_diff = round(angle_diff,4)

        
        return distance,angle_diff
    
    
    def reset(self):
        # 重置仿真世界
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world_service()
            print("World reset successfully")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # Create a SetModelStateRequest object
        objstate = SetModelStateRequest()
        
        objstate.model_state.model_name = "robot0"      
        objstate.model_state.pose.position.x = -60.0      
        objstate.model_state.pose.position.y = -60.0     
        objstate.model_state.pose.position.z = 0.0     
        objstate.model_state.pose.orientation.x = 0.0    
        objstate.model_state.pose.orientation.y = 0.0    
        objstate.model_state.pose.orientation.z = -0.7068252    
        objstate.model_state.pose.orientation.w = 0.7073883
        objstate.model_state.twist.linear.x = 0.0
        objstate.model_state.twist.linear.y = 0.0
        objstate.model_state.twist.linear.z = 0.0
        objstate.model_state.twist.angular.x = 0.0
        objstate.model_state.twist.angular.y = 0.0
        objstate.model_state.twist.angular.z = 0.0
                        
        response = self.set_state_service(objstate)
        if response.success:
            print("Model state reset successfully")
        else:
            print("Failed to set model state:", response.status_message)
        self.robot_position = (-60, -60)
        self.current_z = -1.57
        self.previous_z = -1.57
        cur_location_x = self.robot_position[0]
        cur_location_y = self.robot_position[1]
        state = self._get_obs(cur_location_x,cur_location_y)
        return state
    

    
    def _get_obs(self,cur_location_x,cur_location_y):
        
        robot_position_x = cur_location_x
        robot_position_y = cur_location_y
        robot_position_x = np.array([robot_position_x], dtype=np.float32)
        robot_position_y = np.array([robot_position_y], dtype=np.float32)
        
        robot_angle = np.array([self.current_z], dtype=np.float32)
        

        # 状态空间
        obs = np.concatenate([
            robot_position_x,
            robot_position_y,
            robot_angle
        ])
        obs = self.normalize_state(obs)
        print(obs)
        return obs
        
    def normalize_state(self,state):
        # 输入归一化
        min_state = np.array([-80,-80,-np.pi], dtype=np.float32)
        max_state = np.array([-40,-50,np.pi], dtype=np.float32)
        state_normalized = (state - min_state) / (max_state - min_state + 1e-8)
        return state_normalized
    def step(self, action):
        distance,angle_diff = self._calculate_PositionAndPose()

         # 计算持续时间，动作的模越大，持续时间越长
        if distance > 10:
            duration = distance * 0.01
        else:
            duration = 0.1
        # 发布速度命令并保持一段时间
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]  # 调整线速度比例
        cmd_vel.angular.z = action[1]  # 调整角速度比例
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            time.sleep(0.05)  # 以一定频率发布速度命令

        cur_location_x = self.robot_position[0]
        cur_location_y = self.robot_position[1]
        done = False
        # 判断是否到达目标点
        is_Arrived = self._is_at_target(cur_location_x,cur_location_y)
        # done表示任务是否完成
        
        # 判断是否出界
        is_OutOfRange = False
        if np.abs(cur_location_x + 60) > 20 or cur_location_y + 60 < -20 or cur_location_y > -50:
            is_OutOfRange = True
        
        if is_OutOfRange:
            done = True
        else:
            if is_Arrived:
                done = True
            else:
                done = False

        reward = self._calculate_reward(is_Arrived,is_OutOfRange)

        # 获取下一个状态
        next_obs = self._get_obs(cur_location_x,cur_location_y)
        print(distance, np.degrees(angle_diff), reward, cur_location_x, cur_location_y,action[0],action[1])
        return next_obs, reward, done, {}

    def _calculate_reward(self,is_Arrived,is_OutOfRange):
        # 计算距离当前目标点的距离和角度
        distance,angle_diff = self._calculate_PositionAndPose()

        # 奖励函数
        # 到达目标点
        if is_Arrived:
            reward = 50
        # 超出边界
        elif is_OutOfRange:
            reward = -50
        # 其他情况
        else:
            reward =  -distance - 10 * np.abs(angle_diff)
        
        min_reward = -50
        max_reward = 50
        reward_normalized = (reward - min_reward) / (max_reward - min_reward + 1e-8)
        return reward_normalized

    def _is_at_target(self,cur_location_x,cur_location_y):
        # 判断是否到达目标点（距离小于某个阈值）
        target_point = PATH_POINTS[self.current_point_index]
        distance = np.linalg.norm(np.array([
            target_point[0] - cur_location_x,
            target_point[1] - cur_location_y
        ]))
        if distance < 1.0:
            print("Target Reached!")
            return True  # 阈值为1米

# 自定义回调类
# class ProgressBarCallback(BaseCallback):
#     def __init__(self, total_timesteps, verbose=0):
#         super(ProgressBarCallback, self).__init__(verbose)
#         self.total_timesteps = total_timesteps
#         self.pbar = None
#     def _on_training_start(self):
#         self.pbar = tqdm(total=self.total_timesteps, desc="Training Progress")
#     def _on_step(self):
#         self.pbar.update(1)
#         return True
#     def _on_training_end(self):
#         self.pbar.close()

# # 创建日志目录
# log_dir = "/home/shenlu/ubuntu/consensus_control_ws/src/airplane_formation_control/parameters/.logs"
# os.makedirs(log_dir, exist_ok=True)

# env = make_vec_env(TrackingEnv, n_envs=1)
# # 创建评估回调
# env_callback = EvalCallback(env, best_model_save_path=log_dir, log_path=log_dir, eval_freq=100, deterministic=True, render=False)
# # 创建PPO模型，设置学习率和经验回放的参数
# # model = PPO('MlpPolicy', env, verbose=1, learning_rate=0.03, n_steps=2048, batch_size=64, n_epochs=10, clip_range=0.2)
# # 这里的噪声是一个正态分布噪声，标准差设为0.1
# n_actions = env.action_space.shape[-1]
# # 添加动作噪声
# action_noise = NormalActionNoise(mean=np.zeros(n_actions), sigma=0.1 * np.ones(n_actions))
# model = TD3('MlpPolicy', env, verbose=1, learning_rate=0.03,action_noise=action_noise,train_freq=1000,gradient_steps=1,batch_size=128)
# # 创建自定义进度条回调
# progress_bar_callback = ProgressBarCallback(total_timesteps=1000)
# # 训练模型，传递回调函数
# model.learn(total_timesteps=1000, callback=[env_callback,progress_bar_callback])
# # 保存模型
# model.save("/home/shenlu/ubuntu/consensus_control_ws/src/airplane_formation_control/parameters/ppo_tracking_model")

# 加载模型并进行测试
# model = PPO.load("ppo_tracking_model")
# obs = env.reset()
# while not rospy.is_shutdown():
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     if dones:
#         obs = env.reset()




