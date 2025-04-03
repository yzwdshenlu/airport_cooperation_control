#!/usr/bin/env python
# -*- coding: utf-8 -*-

import matplotlib.pyplot as plt

class TrainingVisualizer:
    def __init__(self):
        # 初始化数据存储
        self.iterations = []
        self.avg_loss = []
        self.avg_q_value = []
        self.max_q_value = []

        # 创建图形和子图
        self.fig, self.axs = plt.subplots(3, 1, figsize=(8, 12))
        self.fig.suptitle("Reinforcement Learning Training Metrics")

        # 子图标题
        self.axs[0].set_title("Average Loss vs Iterations")
        self.axs[1].set_title("Average Q-Value vs Iterations")
        self.axs[2].set_title("Max Q-Value vs Iterations")

        # 坐标标签
        for ax in self.axs:
            ax.set_xlabel("Iterations")
            ax.set_ylabel("Value")

        # 开启交互模式
        plt.ion()

    def update(self, iter_count, av_loss, av_Q, max_Q, iterations):
        """更新图表"""
        # 添加新数据
        self.iterations.append(iter_count)
        self.avg_loss.append(av_loss / iterations)
        self.avg_q_value.append(av_Q / iterations)
        self.max_q_value.append(max_Q)

        # 清除子图并重新绘制
        for ax in self.axs:
            ax.cla()

        # 更新子图标题和标签
        self.axs[0].set_title("Average Loss vs Iterations")
        self.axs[0].set_xlabel("Iterations")
        self.axs[0].set_ylabel("Loss")

        self.axs[1].set_title("Average Q-Value vs Iterations")
        self.axs[1].set_xlabel("Iterations")
        self.axs[1].set_ylabel("Q-Value")

        self.axs[2].set_title("Max Q-Value vs Iterations")
        self.axs[2].set_xlabel("Iterations")
        self.axs[2].set_ylabel("Q-Value")

        # 绘制数据
        self.axs[0].plot(self.iterations, self.avg_loss, label="Avg Loss", color="blue")
        self.axs[1].plot(self.iterations, self.avg_q_value, label="Avg Q-Value", color="orange")
        self.axs[2].plot(self.iterations, self.max_q_value, label="Max Q-Value", color="green")

        # 添加图例
        for ax in self.axs:
            ax.legend()

        # 刷新显示
        plt.pause(0.1)
        
    def finish(self):
        """关闭交互模式并显示最终结果"""
        plt.ioff()
        plt.show()