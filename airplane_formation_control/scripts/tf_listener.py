#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped



# 回调函数，用于处理接收到的TF消息
def tf_callback(msg):
    # 打印接收到的TF变换信息
    rospy.loginfo("Received TF from {} to {} at time {}:".format(
        msg.header.frame_id, msg.child_frame_id, msg.header.stamp))
    rospy.loginfo("  Translation: x={}, y={}, z={}".format(
        msg.transform.translation.x,
        msg.transform.translation.y,
        msg.transform.translation.z))
    rospy.loginfo("  Rotation (quaternion): x={}, y={}, z={}, w={}".format(
        msg.transform.rotation.x,
        msg.transform.rotation.y,
        msg.transform.rotation.z,
        msg.transform.rotation.w))

if __name__ == "__main__":
    # 初始化ROS节点
    rospy.init_node('tf_subscriber', anonymous=True)
    robot_name = rospy.get_param('~tf_name')
    # 创建TF缓存和监听器对象
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    
    # 定义你想监听的父坐标系和子坐标系
    parent_frame = "map"
    child_frame = "{}/odom".format(robot_name) # 这里的robot_1可能需要替换成实际robot_name

    rate = rospy.Rate(10.0) # 设置循环速率为10Hz
    while not rospy.is_shutdown():
        # 循环尝试获取最新的变换信息
        try:
            # 现在是直接查询最近的一次变换关系，可以替换为需要的时间戳
            trans = tf_buffer.lookup_transform(parent_frame, child_frame, rospy.Time(0))
            # 如果查找到变换关系，会进入这个回调处理函数
            tf_callback(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            # 如果查找变换遇到错误，则输出错误信息，并继续循环
            rospy.loginfo("Could not find transform from {} to {}".format(parent_frame, child_frame))
        
        # 根据前面定义的频率休眠相应的时间
        rate.sleep()