#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

class PointCloudHeightFilter {
public:
    PointCloudHeightFilter() : nh_("~"),ground_threshold_(-0.5), upper_threshold_(0.5) {
        
        // 订阅原始点云
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(
            "/robot0/points_raw", 1, &PointCloudHeightFilter::cloudCallback, this);
            
        // 发布过滤后的点云
        filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/robot0/filtered_cloud", 1);
    }

private:
    void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& input_msg) {
        // 转换ROS消息到PCL格式
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*input_msg, *cloud);

        // 创建滤波器对象
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");

        pass.setFilterLimits(ground_threshold_, upper_threshold_);
        pass.setNegative(false); // 保留范围内的点
        
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pass.filter(*filtered_cloud);

        // 转换回ROS消息并发布
        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_cloud, output_msg);
        output_msg.header = input_msg->header;
        filtered_pub_.publish(output_msg);
    }

    ros::NodeHandle nh_;
    ros::Subscriber cloud_sub_;
    ros::Publisher filtered_pub_;
    double ground_threshold_;  // 地面高度阈值（低于此值过滤）
    double upper_threshold_;   // 高度上限阈值（高于此值过滤）
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pointcloud_height_filter");
    PointCloudHeightFilter filter;
    ros::spin();
    return 0;
}