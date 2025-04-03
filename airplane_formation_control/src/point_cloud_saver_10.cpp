#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <queue>
#include <iomanip>
#include <sstream>
#include <boost/filesystem.hpp>
#include <iostream>
#include <ctime>

// 定义保存的最大文件数
const int MAX_FILES = 10;

// 文件保存路径
const std::string save_directory = "/home/shenlu/airports_ws/src/airport_robot_gazebo/config/";

// 循环队列用于保存文件名
std::queue<std::string> file_queue;

// 用于存储最新接收到的点云数据
pcl::PointCloud<pcl::PointXYZ> latest_cloud;
bool new_cloud_available = false;

// 函数声明
void savePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud);

void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS的PointCloud2消息转换为PCL点云
    pcl::fromROSMsg(*cloud_msg, latest_cloud);
    new_cloud_available = true;
}

void savePointCloud(const pcl::PointCloud<pcl::PointXYZ>& cloud) {
    // 获取当前时间戳
    std::time_t t = std::time(nullptr);
    std::tm* now = std::localtime(&t);

    // 格式化时间戳
    std::stringstream ss;
    ss << std::put_time(now, "%Y%m%d_%H%M%S");

    // 构建保存文件路径
    std::string file_name = save_directory + "lidar_scan_" + ss.str() + ".pcd";

    // 保存点云到PCD文件
    pcl::io::savePCDFileASCII(file_name, cloud);
    ROS_INFO("Point cloud saved to %s", file_name.c_str());

    // 将文件名加入队列
    file_queue.push(file_name);

    // 检查队列大小，如果超过最大文件数，删除最旧的文件
    while (file_queue.size() > MAX_FILES) {
        std::string oldest_file = file_queue.front();
        file_queue.pop();
        
        // 删除最旧的文件
        if (boost::filesystem::exists(oldest_file)) {
            boost::filesystem::remove(oldest_file);
            ROS_INFO("Removed oldest file: %s", oldest_file.c_str());
        }
    }
}

void timerCallback(const ros::TimerEvent&) {
    if (new_cloud_available) {
        savePointCloud(latest_cloud);
        new_cloud_available = false;
    }
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "point_cloud_saver");
    ros::NodeHandle nh;

    // 订阅激光雷达点云消息
    ros::Subscriber sub = nh.subscribe("/robot0/points_raw", 1, pointCloudCallback);

    // 创建一个定时器，每秒钟触发一次回调
    ros::Timer timer = nh.createTimer(ros::Duration(5.0), timerCallback);

    // 进入ROS事件循环
    ros::spin();

    return 0;
}

