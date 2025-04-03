#include <ros/ros.h>   
#include <nav_msgs/OccupancyGrid.h>   
#include <nav_msgs/GetMap.h>   
#include <sensor_msgs/PointCloud2.h>   
#include <pcl/io/pcd_io.h>   
#include <pcl_conversions/pcl_conversions.h>   
#include <pcl/point_types.h>

/* DLIO生成的点云地图经过预处理之后，将这个点云地图转二维栅格地图 */
/* 这是一个对比文件，对比栅格地图的宽和高不同有没有什么影响 */
std::string pcd_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot0_dlio_map_without_ground.pcd"; 
   
std::string map_topic_name = "map_2";   
 
nav_msgs::OccupancyGrid map_topic_msg;   
double map_resolution = 0.05;
   
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_PassThrough(new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_Radius(new pcl::PointCloud<pcl::PointXYZ>);   
pcl::PointCloud<pcl::PointXYZ>::Ptr pcd_cloud(new pcl::PointCloud<pcl::PointXYZ>);
   
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg);
void TransformPointCloudToWorld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                double tx, double ty, double tz, 
                                double roll, double pitch, double yaw);


   
int main(int argc, char** argv)   
{
   ros::init(argc, argv, "pcl_filters_test");
   ros::NodeHandle nh;


   ros::Rate loop_rate(1.0);


   ros::Publisher map_topic_pub = nh.advertise<nav_msgs::OccupancyGrid>(map_topic_name, 1);

   if (pcl::io::loadPCDFile<pcl::PointXYZ> (pcd_file, *pcd_cloud) == -1)
   {
     PCL_ERROR ("Couldn't read file: %s \n", pcd_file.c_str());
     return (-1);
   }

   std::cout << "输入点云点数：" << pcd_cloud->points.size() << std::endl;
  
   TransformPointCloudToWorld(pcd_cloud, -42, -100, 0, 0.0, 0.0, 0.0);
   SetMapTopicMsg(pcd_cloud, map_topic_msg);

   while(ros::ok())
   {
     map_topic_pub.publish(map_topic_msg);

     loop_rate.sleep();

     ros::spinOnce();
   }

   return 0;   
}
   
void SetMapTopicMsg(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, nav_msgs::OccupancyGrid& msg)   
{
   msg.header.seq = 0;
   msg.header.stamp = ros::Time::now();  // 设置时间戳
   msg.header.frame_id = "map"; // 设置坐标系
  
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
   msg.info.width = int((x_max - x_min) / map_resolution); // 必须要+1，否则会漏掉一整行加一整列的点(或许可以改成round函数试一试！！！！)
   msg.info.height = int((y_max - y_min) / map_resolution);
   ROS_INFO("width: %d, height: %d", msg.info.width, msg.info.height);

   msg.data.resize(msg.info.width * msg.info.height);
   msg.data.assign(msg.info.width * msg.info.height, 0);  // 默认设置为不占据


   // 设置 z 阈值（例如：z_threshold）
   double z_threshold = 0.5;  // 可以根据实际需求设置该阈值

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

// 将点云转换到世界坐标系(输入参数为机器人相对于世界坐标系的位姿)
void TransformPointCloudToWorld(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
                                double dx, double dy, double dz, 
                                double roll, double pitch, double yaw)
{
    Eigen::Matrix3d rotation_matrix;
    rotation_matrix = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())
                    * Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY())
                    * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());


    Eigen::Vector3d translation_vector(dx, dy, dz);
    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        Eigen::Vector3d point(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z);
        Eigen::Vector3d transformed_point = rotation_matrix * point + translation_vector;
        cloud->points[i].x = transformed_point.x();
        cloud->points[i].y = transformed_point.y();
        cloud->points[i].z = transformed_point.z();
    }
}
