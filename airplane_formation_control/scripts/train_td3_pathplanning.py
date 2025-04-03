import os
import time

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from numpy import inf
from torch.utils.tensorboard import SummaryWriter

from replay_buffer import ReplayBuffer
from TD3_v2 import TrackingEnv

# 评价神经网络模型在环境中的表现
def evaluate(network, epoch, eval_episodes=10):
    avg_reward = 0.0
    col = 0
    for _ in range(eval_episodes):
        count = 0
        state = env.reset()
        done = False
        # 限制每个episode的最大步数为500，以防止无限循环
        while not done and count < 501:
            
            
            action = network.get_action(np.array(state))
            # 线速度[0,4] 角速度[-2,2]
            a_in = [(action[0] + 1) * 2, action[1] * 2]
            state, reward, done, _ = env.step(a_in)
            avg_reward += reward
            count += 1
            # if reward < -90:
            #     col += 1
    avg_reward /= eval_episodes
    # avg_col = col / eval_episodes
    print("..............................................")
    # print(
    #     "Average Reward over %i Evaluation Episodes, Epoch %i: %f, %f"
    #     % (eval_episodes, epoch, avg_reward, avg_col)
    # )
    print(
    "Average Reward over %i Evaluation Episodes, Epoch %i: %f"
    % (eval_episodes, epoch, avg_reward)
    )
    print("..............................................")
    return avg_reward


class Actor(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Actor, self).__init__()
        
        # 输入层到第一个隐藏层
        self.layer_1 = nn.Linear(state_dim, 32)
        # self.ln1 = nn.LayerNorm(400)
        # 第一个隐藏层到第二个隐藏层
        self.layer_2 = nn.Linear(32, action_dim)
        # self.ln2 = nn.LayerNorm(action_dim)
        # 第二个隐藏层到输出层，输出维度为动作的维度
        self.layer_3 = nn.Linear(32, action_dim)
        # tanh 激活函数，用于将输出限制在 (-1, 1) 之间
        self.tanh = nn.Tanh()
        
    def forward(self, s):
        s = F.relu(self.layer_1(s)) # 第一个隐藏层的输出
        s = self.layer_2(s) # 第二个隐藏层的输出
        a = self.tanh(s) # 输出层的输出
        return a


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()

        self.layer_1 = nn.Linear(state_dim, 32)
        self.ln1 = nn.LayerNorm(32)
        self.layer_2_s = nn.Linear(32, 32)
        self.layer_2_a = nn.Linear(action_dim, 32)
        self.layer_3 = nn.Linear(32, 1)

        self.layer_4 = nn.Linear(state_dim, 32)
        self.ln2 = nn.LayerNorm(32)
        self.layer_5_s = nn.Linear(32, 32)
        self.layer_5_a = nn.Linear(action_dim, 32)
        self.layer_6 = nn.Linear(32, 1)

 
    
    def forward(self, s, a):
        s1 = F.relu(self.ln1(self.layer_1(s)))  
        # 直接使用线性层进行计算
        s11 = self.layer_2_s(s1)
        s12 = self.layer_2_a(a)     
        s1 = F.relu(s11 + s12) # 计算新的 s1   
        q1 = self.layer_3(s1) # 计算 q1

        s2 = F.relu(self.ln2(self.layer_4(s)))
        s21 = self.layer_5_s(s2)
        s22 = self.layer_5_a(a)
        s2 = F.relu(s21 + s22)
        q2 = self.layer_6(s2)
        return q1, q2


# TD3 network
class TD3(object):
    def __init__(self, state_dim, action_dim, max_action):
        # Initialize the Actor network
        self.actor = Actor(state_dim, action_dim).to(device) # 主 Actor 网络，用于生成动作
        self.actor_target = Actor(state_dim, action_dim).to(device) # 目标 Actor 网络，用于稳定训练过程。它的权重初始化为与主 Actor 网络相同
        self.actor_target.load_state_dict(self.actor.state_dict()) # 将主 Actor 网络的参数复制到目标网络
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(),lr=0.0005) # 用于优化 Actor 网络参数的优化器

        # Initialize the Critic networks
        self.critic = Critic(state_dim, action_dim).to(device) # 主 Critic 网络，用于评估 Actor 生成的动作的价值
        self.critic_target = Critic(state_dim, action_dim).to(device) # 目标 Critic 网络，类似地用于稳定训练
        self.critic_target.load_state_dict(self.critic.state_dict()) # 将主 Critic 网络的参数复制到目标网络
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters()) # 用于优化 Critic 网络参数的优化器

        self.max_action = max_action # 动作的最大值，用于确保动作输出在有效范围内
        self.writer = SummaryWriter() # 初始化了一个 TensorBoard 的 SummaryWriter，用于记录和可视化训练过程中的重要指标
        self.iter_count = 0 # 一个计数器，用于跟踪训练过程中的迭代次数

    def get_action(self, state):
        # Function to get the action from the actor
        state = torch.Tensor(state.reshape(1, -1)).to(device) # 将输入的状态转换为 PyTorch 的张量
        
        return self.actor(state).cpu().data.numpy().flatten() # 将状态输入到 Actor 网络，生成相应的动作

    # training cycle
    def train(
        self,
        replay_buffer,
        iterations,
        batch_size=128,
        discount=0.99,
        tau=0.005,
        policy_noise=0.2,  
        noise_clip=0.2,
        policy_freq=2,
    ):
        av_Q = 0
        max_Q = -inf
        av_loss = 0
        for it in range(iterations):
            # sample a batch from the replay buffer
            (
                batch_states,
                batch_actions,
                batch_rewards,
                batch_dones,
                batch_next_states,
            ) = replay_buffer.sample_batch(batch_size)
            state = torch.Tensor(batch_states).to(device)
            next_state = torch.Tensor(batch_next_states).to(device)
            action = torch.Tensor(batch_actions).to(device)
            reward = torch.Tensor(batch_rewards).to(device)
            done = torch.Tensor(batch_dones).to(device)

            # Obtain the estimated action from the next state by using the actor-target
            next_action = self.actor_target(next_state) # 预测下一步动作

            # Add noise to the action
            noise = torch.Tensor(batch_actions).data.normal_(0, policy_noise).to(device) # 生成噪声，噪声服从均值为0、标准差为policy_noise的正态分布
            noise = noise.clamp(-noise_clip, noise_clip) # 噪声截断在[-noise_clip,noise_clip]之间，防止噪声过大
            next_action = (next_action + noise).clamp(-self.max_action, self.max_action) 

            # Calculate the Q values from the critic-target network for the next state-action pair
            target_Q1, target_Q2 = self.critic_target(next_state, next_action)

            # Select the minimal Q value from the 2 calculated values
            target_Q = torch.min(target_Q1, target_Q2)
            av_Q += torch.mean(target_Q)
            max_Q = max(max_Q, torch.max(target_Q))
            # Calculate the final Q value from the target network parameters by using Bellman equation
            target_Q = reward + ((1 - done) * discount * target_Q).detach()

            # Get the Q values of the basis networks with the current parameters
            current_Q1, current_Q2 = self.critic(state, action)

            # Calculate the loss between the current Q value and the target Q value
            loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

            # Perform the gradient descent
            self.critic_optimizer.zero_grad()
            loss.backward()
            self.critic_optimizer.step()

            # 每隔 policy_freq 次更新，执行一次策略网络的更新
            if it % policy_freq == 0:
                # Maximize the actor output value by performing gradient descent on negative Q values
                # (essentially perform gradient ascent)
                actor_grad, _ = self.critic(state, self.actor(state))
                actor_grad = -actor_grad.mean()
                self.actor_optimizer.zero_grad()
                actor_grad.backward()
                self.actor_optimizer.step()

                # Use soft update to update the actor-target network parameters by
                # infusing small amount of current parameters
                for param, target_param in zip(
                    self.actor.parameters(), self.actor_target.parameters()
                ):
                    target_param.data.copy_(
                        tau * param.data + (1 - tau) * target_param.data
                    )
                # Use soft update to update the critic-target network parameters by infusing
                # small amount of current parameters
                for param, target_param in zip(
                    self.critic.parameters(), self.critic_target.parameters()
                ):
                    target_param.data.copy_(
                        tau * param.data + (1 - tau) * target_param.data
                    )

            av_loss += loss
        self.iter_count += 1
        # Write new values for tensorboard
        self.writer.add_scalar("loss", av_loss / iterations, self.iter_count)
        self.writer.add_scalar("Av. Q", av_Q / iterations, self.iter_count)
        self.writer.add_scalar("Max. Q", max_Q, self.iter_count)

    def save(self, filename, directory):
        torch.save(self.actor.state_dict(), "%s/%s_actor.pth" % (directory, filename))
        torch.save(self.critic.state_dict(), "%s/%s_critic.pth" % (directory, filename))

    def load(self, filename, directory):
        self.actor.load_state_dict(
            torch.load("%s/%s_actor.pth" % (directory, filename))
        )
        self.critic.load_state_dict(
            torch.load("%s/%s_critic.pth" % (directory, filename))
        )


# Set the parameters for the implementation
device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
seed = 0  # Random seed number
eval_freq = 5000  # After how many steps to perform the evaluation
max_ep = 200  # maximum number of steps per episode
eval_ep = 10  # number of episodes for evaluation
max_timesteps = 10000 # Maximum number of steps to perform
expl_noise = 0.2  # Initial exploration noise starting value in range [expl_min ... 1]
expl_decay_steps = (
    500000  # Number of steps over which the initial exploration noise will decay over
)
expl_min = 0.1  # Exploration noise after the decay in range [0...expl_noise]
batch_size = 64  # Size of the mini-batch
discount = 0.99  # Discount factor to calculate the discounted future reward (should be close to 1)
tau = 0.005  # Soft target update variable (should be close to 0)
policy_noise = 0.2  # Added noise for exploration
noise_clip = 0.2  # Maximum clamping values of the noise
policy_freq = 2  # Frequency of Actor network updates
buffer_size = 1e6  # Maximum size of the buffer
file_name = "TD3_train"  # name of the file to store the policy
save_model = True  # Weather to save the model or not
load_model = False  # Weather to load a stored model
random_near_obstacle = False  # To take random actions near obstacles or not

# Create the network storage folders
if not os.path.exists("./results"):
    os.makedirs("./results")
if save_model and not os.path.exists("./pytorch_models"):
    os.makedirs("./pytorch_models")

# Create the training environment
robot_dim = 3
env = TrackingEnv()
time.sleep(2)
torch.manual_seed(seed)
np.random.seed(seed)
state_dim = robot_dim
action_dim = 2
max_action = 1

# Create the network
network = TD3(state_dim, action_dim, max_action)
# Create a replay buffer
replay_buffer = ReplayBuffer(buffer_size, seed)
# 是否训练已有模型
if load_model:
    try:
        network.load(file_name, "./pytorch_models")
    except:
        print(
            "Could not load the stored model parameters, initializing training with random parameters"
        )

# Create evaluation data store
evaluations = []

timestep = 0
timesteps_since_eval = 0
episode_num = 0
done = True
epoch = 1

count_rand_actions = 0
random_action = []

# 开始训练，最多训练max_timesteps个时间步
while timestep < max_timesteps:

    # On termination of episode
    if done:
        if timestep != 0:
            print("%i episode finished!" % episode_num)
            network.train(
                replay_buffer,
                episode_timesteps,
                batch_size,
                discount,
                tau,
                policy_noise,
                noise_clip,
                policy_freq,
            )

        if timesteps_since_eval >= eval_freq:
            print("Validating")
            timesteps_since_eval %= eval_freq
            evaluations.append(
                evaluate(network=network, epoch=epoch, eval_episodes=eval_ep)
            )
            network.save(file_name, directory="./pytorch_models")
            np.save("./results/%s" % (file_name), evaluations)
            epoch += 1

        state = env.reset()
        done = False

        episode_reward = 0
        episode_timesteps = 0
        episode_num += 1

    # add some exploration noise
    # 噪声随时间递减
    if expl_noise > expl_min:
        expl_noise = expl_noise - ((1 - expl_min) / expl_decay_steps)

    # 预测动作并添加噪声
    action = network.get_action(np.array(state))
    print(action)
    action_noise = np.random.normal(0, expl_noise, size=action_dim)
    action_noise = action_noise.clip(-noise_clip, noise_clip) # 噪声截断在[-noise_clip,noise_clip]之间，防止噪声过大
    action = (action + action_noise).clip(
        -max_action, max_action
    )

    # If the robot is facing an obstacle, randomly force it to take a consistent random action.
    # This is done to increase exploration in situations near obstacles.
    # Training can also be performed without it
    if random_near_obstacle:
        if (
            np.random.uniform(0, 1) > 0.85
            and min(state[4:-8]) < 0.6
            and count_rand_actions < 1
        ):
            count_rand_actions = np.random.randint(8, 15)
            random_action = np.random.uniform(-1, 1, 2)

        if count_rand_actions > 0:
            count_rand_actions -= 1
            action = random_action
            action[0] = -1

    # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
    a_in = [(action[0] + 1) * 2, action[1] * 2]
    next_state, reward, done, _ = env.step(a_in)
    done_bool = 0 if episode_timesteps + 1 == max_ep else int(done) # 如果当前时间步数加1达到max_ep，则设置done_bool为 0，否则将done_bool转换为0或1
    done = 1 if episode_timesteps + 1 == max_ep else int(done)
    episode_reward += reward # 将当前步骤获得的奖励 reward 累加到 episode_reward 中，用于记录整个回合的累计奖励。

    # Save the tuple in replay buffer
    replay_buffer.add(state, action, reward, done_bool, next_state)

    # Update the counters
    state = next_state
    episode_timesteps += 1
    timestep += 1
    timesteps_since_eval += 1

# After the training is done, evaluate the network and save it
evaluations.append(evaluate(network=network, epoch=epoch, eval_episodes=eval_ep))
if save_model:
    network.save("%s" % file_name, directory="./models")
np.save("./results/%s" % file_name, evaluations)
