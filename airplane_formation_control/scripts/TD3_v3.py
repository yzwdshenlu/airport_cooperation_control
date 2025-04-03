#!/usr/bin/env python
# -*- coding: utf-8 -*-

# 基于TD3算法的一个飞机和一个小车的编队控制

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from tf.transformations import euler_from_quaternion
import sensor_msgs.point_cloud2 as pc2
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


TIME_DELTA = 1
os.environ['CUDA_LAUNCH_BLOCKING'] = '1'
class TrackingEnv():
    def __init__(self):
        self.robot0_position = (-60, -60)
        self.robot0_current_angle = -1.57
        self.robot0_previous_angle = -1.57
        self.airplane_position = (-60,-100)
        self.airplane_current_angle = -1.57
        self.airplane_previous_angle = -1.57


      
        rospy.init_node('drl_controller', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/robot0/airport_robot/cmd_vel', Twist, queue_size=1)
        self.plane_cmd_vel_pub = rospy.Publisher('/airplane/airport_robot/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_state_callback, queue_size=1)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_state_service = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # 复原小车
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
            print("Robot model state set successfully")
        else:
            print("Failed to set model state:", response.status_message)
        
        
        # 复原飞机
        # Create a SetModelStateRequest object
        objstate = SetModelStateRequest()
        
        objstate.model_state.model_name = "airplane"      
        objstate.model_state.pose.position.x = -60.0      
        objstate.model_state.pose.position.y = -100.0     
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
            print("Airplane model state set successfully")
        else:
            print("Failed to set model state:", response.status_message)


    def model_state_callback(self, model_states_msg):
        # 读取小车的位置和姿态
        model_index = model_states_msg.name.index('robot0')
        self.robot0_position = (model_states_msg.pose[model_index].position.x, model_states_msg.pose[model_index].position.y)

        quaternion = (
            model_states_msg.pose[model_index].orientation.x,
            model_states_msg.pose[model_index].orientation.y,
            model_states_msg.pose[model_index].orientation.z,
            model_states_msg.pose[model_index].orientation.w
        )
        euler = euler_from_quaternion(quaternion)
        
        self.robot0_current_angle = self.process_angle(euler[2],self.robot0_previous_angle) # 将偏航角转换到[-pi,pi]
        self.robot0_previous_angle = self.robot0_current_angle # 上一帧偏航角
        
        # 读取飞机的位置和姿态
        model_index = model_states_msg.name.index('airplane')
        self.airplane_position = (model_states_msg.pose[model_index].position.x, model_states_msg.pose[model_index].position.y)

        quaternion = (
            model_states_msg.pose[model_index].orientation.x,
            model_states_msg.pose[model_index].orientation.y,
            model_states_msg.pose[model_index].orientation.z,
            model_states_msg.pose[model_index].orientation.w
        )
        euler = euler_from_quaternion(quaternion) # 四元数转换为欧拉角
        
        self.airplane_current_angle = self.process_angle(euler[2],self.airplane_previous_angle) # 将偏航角转换到[-pi,pi]
        self.airplane_previous_angle = self.airplane_current_angle # 上一帧偏航角
        



    def process_angle(self,angle,pre_angle):
        if np.cos(angle - pre_angle) < 0.5:
            angle += np.pi

        n = 0
        while angle < -np.pi:
            n += 1
            angle = angle + n * 2 * np.pi
        n = 0
        while angle > np.pi:
            n -= 1
            angle = angle + n * 2 * np.pi
        return angle
    
    
    # 计算距离当前目标点的距离和角度
    def _calculate_PositionAndPose(self):

        # 计算小车与飞机的位置差
        distance_xy = [self.robot0_position[0] - self.airplane_position[0],self.robot0_position[1] - self.airplane_position[1]]
        # print(self.robot0_position[0],self.airplane_position[0],self.robot0_position[1],self.airplane_position[1])
        # 计算小车与飞机的角度差
        angle_diff = self.robot0_current_angle - self.airplane_current_angle
        
        n = 0
        while angle_diff < -np.pi:
            n += 1
            angle_diff = angle_diff + n * 2 * np.pi
        n = 0
        while angle_diff > np.pi:
            n -= 1
            angle_diff = angle_diff + n * 2 * np.pi
        
        
        return distance_xy,angle_diff
    
    
    def reset(self):
        # 重置仿真世界
        rospy.wait_for_service("/gazebo/reset_world")
        try:
            reset_world_service = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_world_service()
            print("World reset successfully")
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)

        # 复原小车
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
            print("Robot model state set successfully")
        else:
            print("Failed to set model state:", response.status_message)
        
        
        # 复原飞机     
        # Create a SetModelStateRequest object
        objstate = SetModelStateRequest()
        
        objstate.model_state.model_name = "airplane"      
        objstate.model_state.pose.position.x = -60.0      
        objstate.model_state.pose.position.y = -100.0     
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
            print("Airplane model state set successfully")
        else:
            print("Failed to set model state:", response.status_message)
            
        # 重置小车和飞机的位置与姿态
        self.robot0_position = (-60, -60)
        self.robot0_current_angle = -1.57
        self.robot0_previous_angle = -1.57
        self.airplane_position = (-60,-100)
        self.airplane_current_angle = -1.57
        self.airplane_previous_angle = -1.57
        
        state = self._get_obs()
        return state
    

    # 状态空间为x方向距离差，y方向距离差，方向角差
    def _get_obs(self):
        
        distance_xy,angle_diff = self._calculate_PositionAndPose()
        x_diff = np.array([distance_xy[0]], dtype=np.float32)
        y_diff = np.array([distance_xy[1]], dtype=np.float32)
        angle_diff = np.array([angle_diff], dtype=np.float32)
              
        # 状态空间
        obs = np.concatenate([
            x_diff,
            y_diff,
            angle_diff
        ])
        # print(obs)
        obs = self.normalize_state(obs)
        
        return obs
        
    def normalize_state(self,state):
        # 输入归一化
        min_state = np.array([-40,-70,-np.pi], dtype=np.float32)
        max_state = np.array([40,70,np.pi], dtype=np.float32)
        state_normalized = (state - min_state) / (max_state - min_state + 1e-8)
        return state_normalized
    
    def step(self, action):
        distance_xy,angle_diff = self._calculate_PositionAndPose()
        distance = np.linalg.norm(np.array([
            distance_xy[0],
            distance_xy[1]
        ]))
        duration = 0.3
        # 发布小车的速度命令并保持一段时间
        cmd_vel = Twist()
        cmd_vel.linear.x = action[0]  # 调整线速度比例
        cmd_vel.angular.z = action[1]  # 调整角速度比例
        
        plane_cmd_vel = Twist()
        plane_cmd_vel.linear.x = 0.5
        plane_cmd_vel.angular.z = 0
        
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_vel_pub.publish(cmd_vel)
            self.plane_cmd_vel_pub.publish(plane_cmd_vel)
            time.sleep(0.05)  # 以一定频率发布速度命令

        cur_location_x = self.robot0_position[0]
        cur_location_y = self.robot0_position[1]
        
        done = False
        # done表示任务是否完成
        
        # 判断是否出界
        is_OutOfRange = False
        if np.abs(cur_location_x + 60) > 20 or cur_location_y  < -120 or cur_location_y > -50:
            is_OutOfRange = True
        
        is_OutOfLength = False
        if np.abs(distance_xy[0]) > 2 or np.abs((np.abs(distance_xy[1]) - 40)) > 2:
            is_OutOfLength = True
        
        if is_OutOfRange:
            done = True
        # 飞机超出范围
        # elif np.abs(self.airplane_position[0] + 60) > 20  or  self.airplane_position[1] < -120 or self.airplane_position[1] > -50:
        #     done = True
        elif is_OutOfLength:
            done = True
        else:
            done = False

        reward = self._calculate_reward(is_OutOfRange,is_OutOfLength,distance_xy,angle_diff)

        # 获取下一个状态
        next_obs = self._get_obs()
        # print(distance_xy)
        print(distance, np.degrees(angle_diff), reward, cur_location_x, cur_location_y,action[0],action[1])
        return next_obs, reward, done, {}

    def _calculate_reward(self,is_OutOfRange,is_OutOfLength,distance_xy,angle_diff):
        # 奖励函数设计
        # 超出边界
        if is_OutOfRange or is_OutOfLength:
            reward = -80
        else:
            reward = - 2 * np.abs(distance_xy[0]) - 2 * np.abs(distance_xy[1] - 40) - 15 * np.abs(angle_diff)
        # print(reward)
        min_reward = -80
        max_reward = 0
        reward_normalized = (reward - min_reward) / (max_reward - min_reward + 1e-8)
        
        return reward_normalized


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




