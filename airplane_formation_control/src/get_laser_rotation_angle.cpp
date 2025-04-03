#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <sensor_msgs/Imu.h>


//待修改！！！！！！！角度不需要转换到弧度
float angle = 0;//激光雷达旋转角度
float x,y,z;//这三个变量用于接收小车在世界坐标系下的位移
Eigen::Matrix4f transform_matrix1;//变换矩阵 激光雷达 ----> 小车
Eigen::Matrix4f transform_matrix2;//变换矩阵 小车 -----> 世界
Eigen::Matrix4f transform_matrix;//最终的变换矩阵
//函数声明
Eigen::Matrix4f generateTransformMatrix(float angle, const Eigen::Vector3f& axis, const Eigen::Vector3f& translation);

// 回调函数来处理接收到的关节状态消息
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "link_support_joint") {
      //转换到360度以内
      angle = int(msg->position[i]) % 360 + (msg->position[i] - int(msg->position[i]));
      // ROS_INFO("Rotating Joint Position: %f", angle);
    }
  }

  //激光雷达 ----> 小车
  //计算旋转矩阵
  float joint_rotation_angle = angle * (M_PI / 180);//转换为弧度
  // 定义旋转轴（y轴）
  Eigen::Vector3f axis(0, 1, 0);
  //定义位移向量
  Eigen::Vector3f translation(0, 0, 1+0.4-0.03585);
  // Eigen::Vector3f translation(1, 2, 3);
  // 调用函数生成变换矩阵
  transform_matrix1 = generateTransformMatrix(joint_rotation_angle, axis, translation);
  // transform_matrix1 = generateTransformMatrix(test_angle, axis, translation);
}

// 回调函数来处理接收到的里程计消息
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  
  x = msg->pose.pose.position.x;
  y = msg->pose.pose.position.y;
  z = msg->pose.pose.position.z;

  // ROS_INFO("位移x: %f", x);
  // ROS_INFO("位移y: %f", y);
  // ROS_INFO("位移z: %f", z);

  // 从四元数提取旋转矩阵
  Eigen::Quaternionf q(
        msg->pose.pose.orientation.w,
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z
  );

  Eigen::Matrix3f rotation_matrix = q.toRotationMatrix();

    //小车 -----> 世界
  transform_matrix2 = Eigen::Matrix4f::Identity();
  transform_matrix2.block<3, 3>(0, 0) = rotation_matrix;
  transform_matrix2(0, 3) = x;
  transform_matrix2(1, 3) = y;
  transform_matrix2(2, 3) = z;
  // transform_matrix2(0, 3) = test_x;
  // transform_matrix2(1, 3) = test_y;
  // transform_matrix2(2, 3) = test_z;
}

// 回调函数来处理接收到的点云消息
void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // 将ROS的PointCloud2消息转换为PCL点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*cloud_msg, *cloud);

    // 保存点云到PCD文件
    pcl::io::savePCDFileASCII("/home/shenlu/airports_ws/src/airport_robot_gazebo/config/lidar_scan.pcd", *cloud);
    // ROS_INFO("Point cloud saved to lidar_scan.pcd");


    // // 读取PCD文件
    // if (pcl::io::loadPCDFile<pcl::PointXYZ>("home/shenlu/airports_ws/src/airport_robot_gazebo/config/lidar_scan_test.pcd", *cloud) == -1) {
    //     PCL_ERROR("Couldn't read the PCD file \n");
    // }


      //最终的变换矩阵
    transform_matrix = transform_matrix2 * transform_matrix1;

    // 进行点云变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>());//变换后的pcl点云
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform_matrix);
    pcl::io::savePCDFileASCII("/home/shenlu/airports_ws/src/airport_robot_gazebo/config/lidar_scan1.pcd", *transformed_cloud);

    //调试
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix(0,0),transform_matrix(0,1),transform_matrix(0,2),transform_matrix(0,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix(1,0),transform_matrix(1,1),transform_matrix(1,2),transform_matrix(1,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix(2,0),transform_matrix(2,1),transform_matrix(2,2),transform_matrix(2,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix(3,0),transform_matrix(3,1),transform_matrix(3,2),transform_matrix(3,3));
    // ROS_INFO("--------------------------------");

    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix1(0,0),transform_matrix1(0,1),transform_matrix1(0,2),transform_matrix1(0,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix1(1,0),transform_matrix1(1,1),transform_matrix1(1,2),transform_matrix1(1,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix1(2,0),transform_matrix1(2,1),transform_matrix1(2,2),transform_matrix1(2,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix1(3,0),transform_matrix1(3,1),transform_matrix1(3,2),transform_matrix1(3,3));
    // ROS_INFO("--------------------------------");

    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix2(0,0),transform_matrix2(0,1),transform_matrix2(0,2),transform_matrix2(0,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix2(1,0),transform_matrix2(1,1),transform_matrix2(1,2),transform_matrix2(1,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix2(2,0),transform_matrix2(2,1),transform_matrix2(2,2),transform_matrix2(2,3));
    // ROS_INFO("%f, %f, %f, %f, ",transform_matrix2(3,0),transform_matrix2(3,1),transform_matrix2(3,2),transform_matrix2(3,3));


}

// 回调函数，用于处理接收到的IMU消息
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    // ROS_INFO("IMU Data:");
    // ROS_INFO("Orientation - x: [%f], y: [%f], z: [%f], w: [%f]",
    //          msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // ROS_INFO("Angular velocity - x: [%f], y: [%f], z: [%f]",
    //          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    ROS_INFO("Rotation_Linear acceleration - x: [%f], y: [%f], z: [%f]",
             msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
}


int main(int argc, char** argv) {
  

  setlocale(LC_ALL,"");
  ros::init(argc, argv, "point_cloud_process");
  ros::NodeHandle nh;

  // 订阅关节状态消息
  ros::Subscriber sub1 = nh.subscribe("/robot0/airport_robot/joint_states", 10, jointStateCallback);
  //订阅里程计消息
  ros::Subscriber sub2 = nh.subscribe("robot0/odom", 1000, odomCallback);
  //订阅激光雷达点云
  ros::Subscriber sub3 = nh.subscribe("/robot0/points_raw", 1, pointCloudCallback);
  // 订阅IMU消息
  ros::Subscriber sub4 = nh.subscribe("/robot0/rotation_imu", 1000, imuCallback);

  ros::spin();

  // float test_angle = 45 * (M_PI / 180);
  // float test_x = 1.0;
  // float test_y = 2.0;
  // float test_z = 3.0;


  return 0;
}


//生成变换矩阵
Eigen::Matrix4f generateTransformMatrix(float angle, const Eigen::Vector3f& axis, const Eigen::Vector3f& translation) {
    // 创建3x3的旋转矩阵
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = Eigen::AngleAxisf(angle, axis);//注意，单位是弧度！！！！

    // 创建4x4的变换矩阵，并初始化为单位矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotation_matrix;  // 设置旋转部分
    transform_matrix.block<3,1>(0,3) = translation;      // 设置位移部分

    return transform_matrix;
}