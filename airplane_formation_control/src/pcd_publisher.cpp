#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcd_publisher");
    ros::NodeHandle nh;

    // 设置话题名
    ros::Publisher pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("pointcloud", 1);

    // 加载 PCD 文件
    std::string pcd_path = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_dlio_map_without_ground.pcd";
    pcl::PointCloud<pcl::PointXYZ> cloud;

    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, cloud) == -1)
    {
        PCL_ERROR("Couldn't read file %s \n", pcd_path.c_str());
        return -1;
    }

    ROS_INFO("Loaded PCD file with %zu points.", cloud.points.size());

    // 转换为ROS消息
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud, cloud_msg);
    cloud_msg.header.frame_id = "robot1/base_link"; 

    ros::Rate loop_rate(1.0);  // 每秒发布一次

    while (ros::ok())
    {
        cloud_msg.header.stamp = ros::Time::now();  // 实时更新时间戳
        pcl_pub.publish(cloud_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
