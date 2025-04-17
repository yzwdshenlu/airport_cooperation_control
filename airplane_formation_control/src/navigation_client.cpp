#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// 保存上一次发送的目标点
geometry_msgs::PoseStamped last_sent_pose;
bool has_last_sent = false;
bool new_pose_available = false;

// 位姿变化检测的阈值（单位：米、弧度）
const double POSITION_THRESHOLD = 0.1;
const double YAW_THRESHOLD = 0.2;

// 提取偏航角
double getYawFromQuaternion(const geometry_msgs::Quaternion& q)
{
    tf2::Quaternion quat(q.x, q.y, q.z, q.w);
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    return yaw;
}

// 检查是否与上一个目标有明显差异
bool hasPoseChanged(const geometry_msgs::Pose& pose1, const geometry_msgs::Pose& pose2)
{
    double dx = pose1.position.x - pose2.position.x;
    double dy = pose1.position.y - pose2.position.y;
    double dyaw = getYawFromQuaternion(pose1.orientation) - getYawFromQuaternion(pose2.orientation);
    return std::hypot(dx, dy) > POSITION_THRESHOLD || std::abs(dyaw) > YAW_THRESHOLD;
}

// 根据原始四元数设置翻滚角和俯仰角为 0，保持偏航角不变
tf2::Quaternion modifyYawOnly(const tf2::Quaternion& original_quat)
{
    double roll, pitch, yaw;
    tf2::Matrix3x3(original_quat).getRPY(roll, pitch, yaw);
    tf2::Quaternion new_quat;
    new_quat.setRPY(0, 0, yaw);
    return new_quat;
}

// 订阅目标位姿回调
geometry_msgs::PoseStamped raw_target_pose;
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    raw_target_pose = *msg;
    new_pose_available = true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_client");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);

    ros::Subscriber sub = nh.subscribe("/robot1/dlio/odom_node/pose", 10, poseCallback);
    tf::TransformListener listener;

    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    ros::Rate rate(2); // 稍微高一些的频率用于监听，但不频繁发送目标

    while (ros::ok())
    {
        if (new_pose_available)
        {
            try
            {
                geometry_msgs::PoseStamped target_map_pose;
                listener.waitForTransform("map", raw_target_pose.header.frame_id, ros::Time(0), ros::Duration(0.5));
                listener.transformPose("map", raw_target_pose, target_map_pose);

                tf2::Quaternion quat(target_map_pose.pose.orientation.x,
                                     target_map_pose.pose.orientation.y,
                                     target_map_pose.pose.orientation.z,
                                     target_map_pose.pose.orientation.w);
                tf2::Quaternion new_quat = modifyYawOnly(quat);

                target_map_pose.pose.orientation.x = new_quat.x();
                target_map_pose.pose.orientation.y = new_quat.y();
                target_map_pose.pose.orientation.z = new_quat.z();
                target_map_pose.pose.orientation.w = new_quat.w();

                // 只有在目标有明显变化且未在执行中时才发送新目标
                if (!has_last_sent || hasPoseChanged(target_map_pose.pose, last_sent_pose.pose))
                {
                    if (ac.getState().isDone())  // move_base 空闲时才发
                    {
                        move_base_msgs::MoveBaseGoal goal;
                        goal.target_pose.header.frame_id = "map";
                        goal.target_pose.header.stamp = ros::Time::now();
                        goal.target_pose.pose = target_map_pose.pose;
                        goal.target_pose.pose.position.z = 0;

                        ac.sendGoal(goal);
                        last_sent_pose = target_map_pose;
                        has_last_sent = true;
                        new_pose_available = false;
                        ROS_INFO("robot1 raw location: x=%lf y=%lf", raw_target_pose.pose.position.x, raw_target_pose.pose.position.y);
                        ROS_INFO("robot1 location: x=%lf y=%lf", target_map_pose.pose.position.x, target_map_pose.pose.position.y);
                        ROS_INFO("New goal sent: x=%lf y=%lf", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                    }
                }
            }
            catch (tf::TransformException& ex)
            {
                ROS_WARN("Transform failed: %s", ex.what());
            }
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
