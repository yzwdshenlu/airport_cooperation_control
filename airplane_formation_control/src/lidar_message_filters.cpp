#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/common/transforms.h>
#include <Eigen/Dense>
#include <sensor_msgs/Imu.h>


/*双激光雷达点云同步与合并*/

ros::Publisher fused_pub;
pcl::PointCloud<pcl::PointXYZ>::Ptr main_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr secondary_cloud(new pcl::PointCloud<pcl::PointXYZ>);
std_msgs::Header header;
ros::Publisher imu_pub;
sensor_msgs::Imu new_imu;//新的IMU数据
// 创建3x3的旋转矩阵
Eigen::Matrix3f rotation_matrix;



Eigen::Matrix4f createTransformationMatrix(const Eigen::Vector3f& translation, const Eigen::Vector3f& axis, double angle) {
    // Normalize the rotation axis
    Eigen::Vector3f norm_axis = axis.normalized();

    // Create the rotation matrix using the Rodrigues' rotation formula
    Eigen::Matrix3f rotation_matrix;
    rotation_matrix = Eigen::AngleAxisf(angle, axis);//注意，单位是弧度！！！！



    // Create the transformation matrix
    Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
    transformation_matrix.block<3,3>(0,0) = rotation_matrix;
    transformation_matrix.block<3,1>(0,3) = translation;

    return transformation_matrix;
}




//两个激光雷达时间戳近似相同时调用此回调函数
void callback(const sensor_msgs::PointCloud2::ConstPtr& scan1, const sensor_msgs::PointCloud2::ConstPtr& scan2)
{

    //获取两个激光雷达之间的变换矩阵
    //定义旋转角
    double angle = 1.57;
    // 定义旋转轴（y轴）
    Eigen::Vector3f axis(0, 1, 0);
    // 定义位移向量
    Eigen::Vector3f translation(0.2, 0, 0);
    // // 调用函数生成变换矩阵
    Eigen::Matrix4f transform_matrix = createTransformationMatrix(translation, axis, angle);



    // 在这里处理同步后的激光雷达数据
    pcl::fromROSMsg(*scan1, *main_cloud);
    pcl::fromROSMsg(*scan2, *secondary_cloud);

    header = scan1 ->header;

    pcl::PointCloud<pcl::PointXYZ>::Ptr secondary_cloud_transformed(new pcl::PointCloud<pcl::PointXYZ>);
    //将一个雷达点云变换到另一个点云的坐标系
    pcl::transformPointCloud(*secondary_cloud, *secondary_cloud_transformed, transform_matrix);
    // 直接叠加点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_merged(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_merged = *main_cloud + *secondary_cloud_transformed;
    // 发布合并后的点云
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_merged, output);


    output.header = header;  // 设置时间戳为其中一个雷达的时间戳
    fused_pub.publish(output);
}

// 回调函数，用于处理接收到的IMU消息
void imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  new_imu = *msg;
  

  //激光雷达不旋转的条件下，角速度不变,旋转矩阵为单位阵
  //读取角速度
  Eigen::Vector3f ang_vel(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);//小车的角速度
  new_imu.angular_velocity.x = ang_vel.x();
  new_imu.angular_velocity.y = ang_vel.y();
  new_imu.angular_velocity.z = ang_vel.z();

  // 读取线加速度
  Eigen::Vector3f lin_acc(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);//小车的线加速度
  //定义IMU和激光雷达的位移向量
  Eigen::Vector3f IMU2Lidar_translation(0.2, 0.2, 0);

  Eigen::Vector3f rotated_lin_acc = (lin_acc + ang_vel.cross(ang_vel.cross(IMU2Lidar_translation)));
  new_imu.linear_acceleration.x = rotated_lin_acc.x();
  new_imu.linear_acceleration.y = rotated_lin_acc.y();
  new_imu.linear_acceleration.z = rotated_lin_acc.z();

  
  // 发布新的IMU消息
  imu_pub.publish(new_imu);


}






int main(int argc, char** argv)
{
    ros::init(argc, argv, "lidar_sync_node");
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::PointCloud2> sub1(nh, "/robot0/points_raw", 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> sub2(nh, "/robot0/points_raw1", 1);
    fused_pub = nh.advertise<sensor_msgs::PointCloud2>("/robot0/fused_points", 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::PointCloud2, sensor_msgs::PointCloud2> MySyncPolicy;

     // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2);
    sync.registerCallback(boost::bind(&callback, _1, _2));

    // 订阅IMU消息
    ros::Subscriber sub3 = nh.subscribe("/robot0/imu", 1000, imuCallback);

    imu_pub = nh.advertise<sensor_msgs::Imu>("/robot0/processed_imu", 10);
    

    ros::spin();

    return 0;
}
