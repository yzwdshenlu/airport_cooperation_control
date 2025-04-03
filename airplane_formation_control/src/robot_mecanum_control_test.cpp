#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

// 麦克纳姆轮小车行走测试

int main(int argc, char** argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "cmd_vel_publisher");

    // 创建 ROS 节点句柄
    ros::NodeHandle nh;

    // 创建发布器，发布 /robot0/cmd_vel 话题
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("robot0/airport_robot/cmd_vel", 10);

    // 设置循环发布的频率
    ros::Rate loop_rate(10);  // 10Hz

    while (ros::ok())
    {
        // 创建一个 Twist 消息
        geometry_msgs::Twist cmd_vel_msg;

        // 设置线速度 vx 和 vy
        cmd_vel_msg.linear.x = 0.0;  // vx = 1
        cmd_vel_msg.linear.y = 0.0;  // vy = 1
        cmd_vel_msg.linear.z = 0.0;

        // 设置角速度
        cmd_vel_msg.angular.x = 0.0;
        cmd_vel_msg.angular.y = 0.0;
        cmd_vel_msg.angular.z = 0.5;

        // 发布消息
        cmd_vel_pub.publish(cmd_vel_msg);

        // 按照指定的频率休眠
        loop_rate.sleep();
    }

    return 0;
}
