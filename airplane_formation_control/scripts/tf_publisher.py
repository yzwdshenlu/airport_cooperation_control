#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros # 创建TF广播器
import tf # 欧拉角转换四元数
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped # 创建广播的数据：坐标系关系







# 用于存储TF广播器实例
broadcaster = None

def odom_callback(msg):

    # 处理并广播TF位置关系
    doPose(msg.pose.pose)

def doPose(pose): # 函数的消息和接收到的消息相同
    # 使用全局变量 broadcaster
    global broadcaster
    
    # 4.2 创建广播的数据
    tfs = TransformStamped()
    tfs.header.frame_id = "map"
    tfs.header.stamp = rospy.Time.now()
    tfs.child_frame_id = "{}/odom".format(robot_name)

    # 坐标系原点偏移量
    tfs.transform.translation.x = pose.position.x
    tfs.transform.translation.y = pose.position.y
    tfs.transform.translation.z = pose.position.z
    
    # 直接使用收到的四元数
    tfs.transform.rotation = pose.orientation
    
    # 4.3 广播器发送数据
    broadcaster.sendTransform(tfs)

if __name__ == "__main__":
    # 2.初始化ros节点
    rospy.init_node("tf_publiser")
    robot_name = rospy.get_param('~tf_name') 

    # 创建TF广播器实例
    broadcaster = tf2_ros.TransformBroadcaster()
    
    # 3.订阅话题消息：/turtle1/pose
    sub = rospy.Subscriber("odom", Odometry, odom_callback)
    
    # 5.spin
    rospy.spin()