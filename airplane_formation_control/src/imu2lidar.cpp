#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>

/*****************************************************************************
 * 订阅激光雷达和IMU话题，将IMU数据变换成激光雷达的数据，并将变换后的数据发布，用于DLIO算法
 * 步骤：
 * 1.订阅话题
 * 2.读取激光雷达的旋转角速度和角度值
 * 3.将IMU数据变换为激光雷达数据
 * 4.将变换后的数据发布
*******************************************************************************/
float angle = 0;//激光雷达旋转角度
float x,y,z;//这三个变量用于接收小车在世界坐标系下的位移
Eigen::Matrix4f transform_matrix1;//变换矩阵 激光雷达 ----> 小车
ros::Publisher imu_pub;
sensor_msgs::Imu new_imu;//新的IMU数据
// 创建3x3的旋转矩阵
Eigen::Matrix3f rotation_matrix;
//激光雷达旋转角速度
double vel_angular_z = 0.0;

//生成变换矩阵
Eigen::Matrix4f generateTransformMatrix(float angle, const Eigen::Vector3f& axis, const Eigen::Vector3f& translation) {
    
    rotation_matrix = Eigen::AngleAxisf(angle, axis);//注意，单位是弧度！！！！

    // 创建4x4的变换矩阵，并初始化为单位矩阵
    Eigen::Matrix4f transform_matrix = Eigen::Matrix4f::Identity();
    transform_matrix.block<3,3>(0,0) = rotation_matrix;  // 设置旋转部分
    transform_matrix.block<3,1>(0,3) = translation;      // 设置位移部分

    return transform_matrix;
}

// 回调函数来处理接收到的关节状态消息
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
  for (int i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == "link_support_joint") {
      angle = msg->position[i];
    }
  }

  //激光雷达坐标系 ---> 机器人坐标系
  //计算旋转矩阵

  // 定义旋转轴（y轴）
  Eigen::Vector3f axis(0, 1, 0);
  rotation_matrix = Eigen::AngleAxisf(angle, axis);//注意，单位是弧度！！！！

}

// 回调函数，用于处理接收到的IMU消息
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  new_imu = *msg;
  // 提取原始四元数并转换为Eigen四元数
  Eigen::Quaternionf q_orig(msg->orientation.w,
                            msg->orientation.x,
                            msg->orientation.y,
                            msg->orientation.z);
   

  Eigen::Quaternionf q_rot(rotation_matrix.inverse());
  Eigen::Quaternionf q_new = q_orig * q_rot;
  q_new.normalize();

  // 更新IMU消息中的四元数
  new_imu.orientation.x = q_new.x();
  new_imu.orientation.y = q_new.y();
  new_imu.orientation.z = q_new.z();
  new_imu.orientation.w = q_new.w();


  //读取角速度
  Eigen::Vector3f ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);//小车的角速度
  Eigen::Vector3f rotated_ang_vel = rotation_matrix.inverse() * ang_vel;
  rotated_ang_vel.y() += vel_angular_z;
  new_imu.angular_velocity.x = rotated_ang_vel.x();
  new_imu.angular_velocity.y = rotated_ang_vel.y();
  new_imu.angular_velocity.z = rotated_ang_vel.z();

  // 读取线加速度
  Eigen::Vector3f lin_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);//小车的线加速度
  //定义IMU和激光雷达的位移向量
  Eigen::Vector3f IMU2Lidar_translation(0, -0.001, 0.2);//激光雷达在IMU坐标系下的位移

  Eigen::Vector3f rotated_lin_acc = rotation_matrix.inverse() * (lin_acc + ang_vel.cross(ang_vel.cross(IMU2Lidar_translation)));
  new_imu.linear_acceleration.x = rotated_lin_acc.x();
  new_imu.linear_acceleration.y = rotated_lin_acc.y();
  new_imu.linear_acceleration.z = rotated_lin_acc.z();

  
  // 发布新的IMU消息
  imu_pub.publish(new_imu);

    // ROS_INFO("IMU Data:");
    // ROS_INFO("原始IMU的四元数 - x: [%f], y: [%f], er z: [%f], w: [%f]",
    //          msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
    // ROS_INFO("Angular velocity - x: [%f], y: [%f], z: [%f]",
    //          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    // ROS_INFO("Linear acceleration - x: [%f], y: [%f], z: [%f]",
    //          msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // ROS_INFO("旋转角: %f", angle);
    // ROS_INFO("原始IMU的线加速度 - x: [%f], y: [%f], z: [%f]",
    //          msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    // ROS_INFO("原始IMU的角速度 - x: [%f], y: [%f], z: [%f]",
    //          msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
    // ROS_INFO("旋转矩阵:");
    // ROS_INFO("%f, %f, %f",rotation_matrix(0,0),rotation_matrix(0,1),rotation_matrix(0,2));
    // ROS_INFO("%f, %f, %f",rotation_matrix(1,0),rotation_matrix(1,1),rotation_matrix(1,2));
    // ROS_INFO("%f, %f, %f",rotation_matrix(2,0),rotation_matrix(2,1),rotation_matrix(2,2));
    // ROS_INFO("固定IMU的线加速度 - x: [%f], y: [%f], z: [%f]",
    //          new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z);
    // ROS_INFO("Processed IMU Data:");
    // ROS_INFO("Orientation - x: [%f], y: [%f], er z: [%f], w: [%f]",
    //          new_imu.orientation.x, new_imu.orientation.y, new_imu.orientation.z, new_imu.orientation.w);
    // ROS_INFO("固定IMU的角速度 - x: [%f], y: [%f], z: [%f]",
    //          new_imu.angular_velocity.x, new_imu.angular_velocity.y, new_imu.angular_velocity.z);
    // ROS_INFO("Linear acceleration - x: [%f], y: [%f], z: [%f]",
    //          new_imu.linear_acceleration.x, new_imu.linear_acceleration.y, new_imu.linear_acceleration.z);
    
}
void velCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    vel_angular_z = msg->angular.z;
}

int main(int argc, char** argv) {
  

  setlocale(LC_ALL,"");
  ros::init(argc, argv, "imu2lidar");
  ros::NodeHandle nh;

  // 订阅关节状态消息
  ros::Subscriber sub1 = nh.subscribe("/robot0/airport_robot/joint_states", 10, jointStateCallback);

  // 订阅IMU消息
  ros::Subscriber sub2 = nh.subscribe("/robot0/imu", 1000, imuCallback);
  
  //订阅激光雷达旋转角速度
  ros::Subscriber sub3 = nh.subscribe("/robot0/airport_robot/vel", 1000, velCallback);

  imu_pub = nh.advertise<sensor_msgs::Imu>("/robot0/processed_imu", 10);

  ros::spin();

  return 0;
}