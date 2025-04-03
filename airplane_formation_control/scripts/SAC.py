#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from stable_baselines3 import SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.vec_env import DummyVecEnv
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import BaseCallback
from nav_msgs.msg import Odometry
import gym
from gazebo_msgs.msg import ModelStates
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
    (-60, -112), (-72, -98), (-52, -85), (-52, -75),
    (-68, -60), (-90, -60), (-90, -68), (-106, -75),
    (-106, -85), (-86, -96), (-86, -104), (-85, -112)
]



class TrackingEnv(gym.Env):
    def __init__(self):
        super(TrackingEnv, self).__init__()
        self.action_space = spaces.Box(low=np.array([-1.0, -3.0]), high=np.array([3.0, 3.0]), dtype=np.float32)
        self.observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(2,), dtype=np.float32)
        self.current_point_index = 0
        self.robot_position = (0, 0)
        self.current_z = -1.57
        self.previous_z = -1.57
        self.reward = 0
        
        rospy.init_node('drl_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/robot0/airport_robot/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback, queue_size=1)



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
        # print(self.current_z)

        n = 0
        while self.current_z < 0:
            n += 1
            self.current_z = self.current_z + n * 2 * np.pi
        while self.current_z > 2 * np.pi:
            n -= 1
            self.current_z = self.current_z + n * 2 * np.pi

        self.previous_z = self.current_z # 上一帧偏航角

    def reset(self):
        self.current_point_index = 0
        self.robot_position = (0, 0)
        return self._get_obs()


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
        angle_diff = np.arccos(np.clip(np.dot(robot_to_target, robot_heading) / (np.linalg.norm(robot_to_target) * np.linalg.norm(robot_heading)), -1.0, 1.0))
        angle_diff = round(angle_diff,4)
        return distance,angle_diff

    
    def _get_obs(self):
        # 计算距离当前目标点的距离和角度
        target_distance,target_angle_diff = self._calculate_PositionAndPose()

        target_distance = np.array([target_distance], dtype=np.float32)
        target_angle_diff = np.array([target_angle_diff], dtype=np.float32)
        # 合并机器人当前位置和与目标点的距离，作为状态空间
        obs = np.concatenate([
            target_distance,
            target_angle_diff,
        ])
        return obs
        

    def step(self, action):
        # 限制动作范围
        action = np.clip(action, self.action_space.low, self.action_space.high)

        distance,angle_diff = self._calculate_PositionAndPose()
        
        # 计算持续时间，动作的模越大，持续时间越长
        if distance > 10:
            duration = distance * 0.005 
        else:
            duration = 0.0001

        # 发布速度命令并保持一段时间
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0] * 2  # 调整线速度比例
        cmd_vel.angular.z = action[1] * 2  # 调整角速度比例
        # self.cmd_vel_pub.publish(cmd_vel)
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            rospy.sleep(0.1)  # 以一定频率发布速度命令

        

        # 判断是否到达目标点
        is_Arrived = self._is_at_target()
        # done表示任务是否完成(到达所有的目标点)
        if is_Arrived:
            self.current_point_index += 1
            if self.current_point_index >= len(PATH_POINTS):
                done = True
            else:
                done = False
        else:
            done = False

        reward = self._calculate_reward(action,is_Arrived)

        # 获取下一个状态
        next_obs = self._get_obs()
        print(self.current_z)

        return next_obs, reward, done, {}

    def _calculate_reward(self,action,is_Arrived):
        # 计算距离当前目标点的距离和角度
        distance,angle_diff = self._calculate_PositionAndPose()

        # 奖励函数

        # 到达目标点
        if is_Arrived:
            reward = 100
        # 其他情况
        reward =  -distance - 10 * angle_diff
        print(distance, np.degrees(angle_diff), reward)
        # print(self.current_z)
        return reward

    def _is_at_target(self):
        # 判断是否到达目标点（距离小于某个阈值）
        target_point = PATH_POINTS[self.current_point_index]
        distance = np.linalg.norm(np.array([
            target_point[0] - self.robot_position[0],
            target_point[1] - self.robot_position[1]
        ]))
        if distance < 1.0:
            return True  # 阈值为1米

# 自定义回调类
class ProgressBarCallback(BaseCallback):
    def __init__(self, total_timesteps, verbose=0):
        super(ProgressBarCallback, self).__init__(verbose)
        self.total_timesteps = total_timesteps
        self.pbar = None
    def _on_training_start(self):
        self.pbar = tqdm(total=self.total_timesteps, desc="Training Progress")
    def _on_step(self):
        self.pbar.update(1)
        return True
    def _on_training_end(self):
        self.pbar.close()

# 创建日志目录
log_dir = "/home/shenlu/ubuntu/consensus_control_ws/src/airplane_formation_control/parameters/.logs"
os.makedirs(log_dir, exist_ok=True)

env = make_vec_env(TrackingEnv, n_envs=1)
# 创建评估回调
env_callback = EvalCallback(env, best_model_save_path=log_dir, log_path=log_dir, eval_freq=800, deterministic=True, render=False)
# 创建SAC模型，设置学习率和经验回放的参数
model = SAC('MlpPolicy', env, verbose=1, learning_rate=0.03, buffer_size=1000, batch_size=64, ent_coef='auto')
# 创建自定义进度条回调
progress_bar_callback = ProgressBarCallback(total_timesteps=800)
# 训练模型，传递回调函数
model.learn(total_timesteps=800, callback=[env_callback, progress_bar_callback])
# 保存模型
model.save("/home/shenlu/ubuntu/consensus_control_ws/src/airplane_formation_control/parameters/ppo_tracking_model")

# 加载模型并进行测试
# model = PPO.load("ppo_tracking_model")
# obs = env.reset()
# while not rospy.is_shutdown():
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     if dones:
#         obs = env.reset()




