#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ros/ros.h>   
#include <sensor_msgs/PointCloud2.h>   
#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/io/pcd_io.h>   
#include <pcl/point_types.h>   
#include <mutex>
   
pcl::PointCloud<pcl::PointXYZ> cloud;   
std::mutex cloud_mutex;   
bool cloud_received = false;
   
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    std::lock_guard<std::mutex> lock(cloud_mutex);
    pcl::fromROSMsg(*cloud_msg, cloud);
    cloud_received = true;
    pcl::io::savePCDFileASCII("/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_pointcloud.pcd", cloud);
    ROS_INFO("PointCloud saved to file.");   
}
   
int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_saver");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/robot1/points_raw", 1, pointCloudCallback);

    ros::spin();

    return 0;   
}

