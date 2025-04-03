#include <iostream>   
#include <pcl/point_types.h>   
#include <pcl/io/pcd_io.h>   
#include <pcl/filters/passthrough.h>

/*这段代码使用直通滤波器滤除了地面的点云，对DLIO建立的点云地图进行预处理*/
   
int main() {
    // 指定输入和输出点云文件的路径
    // const std::string input_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot0_dlio_map_obscale.pcd";  // 输入文件路径
    // const std::string output_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot0_dlio_map_without_ground.pcd"; // 输出文件路径
    const std::string input_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_dlio_map_obscale.pcd";  // 输入文件路径
    const std::string output_file = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_dlio_map_without_ground.pcd"; // 输出文件路径

    // 创建点云指针
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

    // 读取点云文件
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(input_file, *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "Loaded point cloud with " << cloud->points.size() << " points." << std::endl;

    // 创建通过滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");

    // 设置滤波范围
    // pass.setFilterLimits(-0.5, 100.0);  // robot0
    pass.setFilterLimits(-0.5, 100.0);  // robot1
    pass.filter(*cloud_filtered);

    std::cout << "Filtered point cloud has " << cloud_filtered->points.size() << " points." << std::endl;

    // 保存过滤后的点云
    pcl::io::savePCDFileASCII(output_file, *cloud_filtered);
    std::cout << "Saved filtered point cloud to " << output_file << std::endl;

    return 0;   
}
