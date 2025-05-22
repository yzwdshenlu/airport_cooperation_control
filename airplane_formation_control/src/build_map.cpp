#include <ros/ros.h>   
#include <nav_msgs/OccupancyGrid.h>   
#include <nav_msgs/GetMap.h>   
#include <sensor_msgs/PointCloud2.h>   
#include <pcl/io/pcd_io.h>   
#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>
#include <pcl/common/transforms.h>

/* DLIO生成的点云地图经过预处理之后，将这个点云地图转二维栅格地图 */
std::string pcd_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_dlio_map_without_ground.pcd"; 
// std::string pcd_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot0_dlio_map_without_ground.pcd"; 
std::string map_topic_name = "map";   
 
nav_msgs::OccupancyGrid map_topic_msg;   
double map_resolution = 0.05;
   
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);
bool TransformPointCloudToWorldWithTF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      const std::string& source_frame,
                                      const std::string& target_frame,
                                      tf2_ros::Buffer& tf_buffer);


   
int main(int argc, char** argv)
{
    ros::init(argc, argv, "pcl_filters");
    ros::NodeHandle nh;

    ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

    // 加载 PCD 文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file, *pcd_cloud) == -1)
    {
        PCL_ERROR("Couldn't read file: %s \n", pcd_file.c_str());
        return -1;
    }

    std::cout << "输入点云点数：" << pcd_cloud->points.size() << std::endl;

    // TF监听器
    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener(tf_buffer);

    ros::Rate rate(1.0);
    bool transformed = false;

    while (ros::ok() && !transformed)
    {
        // 尝试获取变换并转换点云
        transformed = TransformPointCloudToWorldWithTF(pcd_cloud, "robot1/velodyne_base_link", "robot1/base_link", tf_buffer);
        if (!transformed)
        {
            ROS_WARN("wait for TF...");
            ros::spinOnce();
            rate.sleep();
            continue;
        }

        // 成功转换后，生成地图消息
        SetMapTopicMsg(pcd_cloud, map_topic_msg);
        break;
    }

    while (ros::ok())
    {
        map_topic_pub.publish(map_topic_msg);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
   
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)   
{
   msg.header.seq = 0;
   msg.header.stamp = ros::Time::now();  // 设置时间戳
  //  msg.header.frame_id = "map"; // 设置坐标系
   msg.header.frame_id = "robot1/base_link"; // 设置坐标系
  
   msg.info.map_load_time = ros::Time::now();
   msg.info.resolution = map_resolution;

   double x_min, x_max, y_min, y_max;

   if(cloud->points.empty())
   {
    ROS_WARN("pcd is empty!\n");
    return;
   }

   // 计算点云的最小最大值（用于确定栅格地图的大小）
   for(int i = 0; i < cloud->points.size(); i++)
   {
    if(i == 0)
    {
      x_min = x_max = cloud->points[i].x;
      y_min = y_max = cloud->points[i].y;
    }

    double x = cloud->points[i].x;
    double y = cloud->points[i].y;

    if(x < x_min) x_min = x;
    if(x > x_max) x_max = x;

    if(y < y_min) y_min = y;
    if(y > y_max) y_max = y;
   }

   msg.info.origin.position.x = x_min;
   msg.info.origin.position.y = y_min;
   msg.info.origin.position.z = 0.0;
   msg.info.origin.orientation.x = 0.0;
   msg.info.origin.orientation.y = 0.0;
   msg.info.origin.orientation.z = 0.0;
   msg.info.origin.orientation.w = 1.0;

   // 计算地图的宽度和高度，使用点云的最小最大值来确定大小
   msg.info.width = int((x_max - x_min) / map_resolution) + 1; // 必须要+1，否则会漏掉一整行加一整列的点(或许可以改成round函数试一试！！！！)
   msg.info.height = int((y_max - y_min) / map_resolution) + 1;
   ROS_INFO("width: %d, height: %d", msg.info.width, msg.info.height);

   msg.data.resize(msg.info.width * msg.info.height);
   msg.data.assign(msg.info.width * msg.info.height, 0);  // 默认设置为不占据


   // 设置 z 阈值（例如：z_threshold）
   double z_threshold = 10.5;  // 可以根据实际需求设置该阈值

   // 遍历点云，设置栅格占据信息
   for(int iter = 0; iter < cloud->points.size(); iter++)
   {
    int i = int((cloud->points[iter].x - x_min) / map_resolution);
    if(i < 0 || i >= msg.info.width) continue;

    int j = int((cloud->points[iter].y - y_min) / map_resolution);
    if(j < 0 || j >= msg.info.height) continue;

    // 根据 z 坐标判断栅格是否占据
    if(cloud->points[iter].z < z_threshold)  // 如果 z 小于阈值，设置为占据
    {
      msg.data[i + j * msg.info.width] = 100;  // 占据
    }
    else  // 如果 z 大于阈值，设置为不占据
    {
      msg.data[i + j * msg.info.width] = 0;  // 不占据
    }
   }   
}

// 将点云转换到世界坐标系(输入参数为机器人相对于目标坐标系的位姿)
bool TransformPointCloudToWorldWithTF(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,
                                      const std::string& source_frame,
                                      const std::string& target_frame,
                                      tf2_ros::Buffer& tf_buffer)
{
    geometry_msgs::TransformStamped transformStamped;
    try
    {
        transformStamped = tf_buffer.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(3.0));
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("TF transform failed: %s", ex.what());
        return false;
    }

    // 转换为 Eigen 格式
    Eigen::Affine3d transform = tf2::transformToEigen(transformStamped.transform);
    // 输出旋转和平移
    Eigen::Matrix3d rotation = transform.rotation();
    Eigen::Vector3d translation = transform.translation();
    std::cout << "Rotation: \n" << rotation << std::endl;
    std::cout << "Translation: \n" << translation << std::endl;
    pcl::transformPointCloud(*cloud, *cloud, transform);
    return true;
}
