#include <pcl/registration/ia_ransac.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/search/kdtree.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <time.h>
#include <pcl/filters/passthrough.h>

/*实现了初始点云与地图点云的配准，输出一个变换矩阵，表示机器人初始位置与任务起点的位置和旋转关系*/

using pcl::NormalEstimation;
using pcl::search::KdTree;
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;
 
PointCloud::Ptr cloud_src_o (new PointCloud);//原点云，待配准
PointCloud::Ptr cloud_tgt_o (new PointCloud);//目标点云
/*滤除地面点云*/
void passThrough(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    // 创建通过滤波器对象
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");

    // 设置滤波范围
    pass.setFilterLimits(-0.5, 100.0);  // robot1
    // pass.setFilterLimits(-0.5, 100.0);  // robot0
    pass.filter(*cloud);
    
} 
/*预处理点云：降采样*/
void preprocessPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
    // 降采样
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.2f, 0.2f, 0.2f); // 根据实际场景调整
    voxel.filter(*cloud);

}
/*滤除半径之外的点云*/
pcl::PointCloud<pcl::PointXYZ>::Ptr filterPointsByRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, float radius_threshold) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>());
    for (const auto& point : cloud->points) {
        // 计算点到原点的距离
        float distance = std::sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
        // 如果距离小于或等于阈值，则保留该点
        if (distance <= radius_threshold) {
            filtered_cloud->points.push_back(point);
        }
    }
    filtered_cloud->width = static_cast<uint32_t>(filtered_cloud->points.size());
    filtered_cloud->height = 1;  // 表示这是无序点云（不是2D图像）
    filtered_cloud->is_dense = true;
    return filtered_cloud;   
}
/*计算法线和FPFH*/
void computeNormalsAndFPFH(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                           pcl::PointCloud<pcl::Normal>::Ptr& normals,
                           pcl::PointCloud<pcl::FPFHSignature33>::Ptr& fpfhs) {
    // 计算表面法线
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);
    ne.setKSearch(30);
    ne.compute(*normals);
    // 计算FPFH
    pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
    fpfh.setInputCloud(cloud);
    fpfh.setInputNormals(normals);
    pcl::search::KdTree<PointT>::Ptr tree_fpfh(new pcl::search::KdTree<PointT>);
    fpfh.setSearchMethod(tree_fpfh);
    fpfh.setKSearch(30);
    fpfh.compute(*fpfhs);   
}
int main (int argc, char** argv)
{
    const std::string map_cloud_path = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_dlio_map_without_ground.pcd";  // 地图点云文件路径
    const std::string cloud_path = "/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/robot1_curPC.pcd";  // 点云文件路径
   //加载点云文件

   pcl::io::loadPCDFile (cloud_path,*cloud_src_o);  
   pcl::io::loadPCDFile (map_cloud_path,*cloud_tgt_o);

    float radius_threshold = 20.0;
    cloud_src_o = filterPointsByRadius(cloud_src_o,radius_threshold);
    cloud_tgt_o = filterPointsByRadius(cloud_tgt_o,radius_threshold);

   passThrough(cloud_src_o);
 

   //去除NAN点
   std::vector<int> indices_src; //保存去除的点的索引
   pcl::removeNaNFromPointCloud(*cloud_src_o,*cloud_src_o, indices_src);

   //下采样滤波
   preprocessPointCloud(cloud_src_o);

   // 计算表面法线和FPFH
    pcl::PointCloud<pcl::Normal>::Ptr cloud_src_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_src(new pcl::PointCloud<pcl::FPFHSignature33>);
    computeNormalsAndFPFH(cloud_src_o, cloud_src_normals, fpfhs_src);
 
    //去除NAN点
   std::vector<int> indices_tgt;
   pcl::removeNaNFromPointCloud(*cloud_tgt_o,*cloud_tgt_o, indices_tgt);
 
    //下采样滤波
   preprocessPointCloud(cloud_tgt_o);


    // 计算表面法线和FPFH
   pcl::PointCloud<pcl::Normal>::Ptr cloud_tgt_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs_tgt(new pcl::PointCloud<pcl::FPFHSignature33>);
    computeNormalsAndFPFH(cloud_tgt_o, cloud_tgt_normals, fpfhs_tgt);
 
   //SAC配准
   pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> scia;
   scia.setInputSource(cloud_src_o);
   scia.setInputTarget(cloud_tgt_o);
   scia.setSourceFeatures(fpfhs_src);
   scia.setTargetFeatures(fpfhs_tgt);
   scia.setMinSampleDistance(0.18);
   scia.setNumberOfSamples(6);
   //scia.setCorrespondenceRandomness(20);
   PointCloud::Ptr sac_result (new PointCloud);
   scia.align(*sac_result);
   std::cout  <<"sac has converged:"<<scia.hasConverged()<<"  score: "<<scia.getFitnessScore()<<endl;
   Eigen::Matrix4f sac_trans;
   sac_trans=scia.getFinalTransformation();
   std::cout<<sac_trans<<endl;
 
   //icp配准
   PointCloud::Ptr icp_result (new PointCloud);
   pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
   icp.setInputSource(sac_result);
   icp.setInputTarget(cloud_tgt_o);
   //Set the max correspondence distance to 4cm (e.g., correspondences with higher distances will be ignored)
   icp.setMaxCorrespondenceDistance (0.20);
   // 最大迭代次数
   icp.setMaximumIterations (32);
   // 两次变化矩阵之间的差值
   icp.setTransformationEpsilon (1e-10);
   // 均方误差
   icp.setEuclideanFitnessEpsilon (0.01);
   icp.align(*icp_result,sac_trans);
 
   std::cout << "ICP has converged:" << icp.hasConverged()
       << " score: " << icp.getFitnessScore() << std::endl;
   Eigen::Matrix4f icp_trans;
   icp_trans=icp.getFinalTransformation();

   std::cout<<icp_trans<<endl;
   //使用创建的变换对未过滤的输入点云进行变换
   pcl::transformPointCloud(*cloud_src_o, *icp_result, icp_trans);

    // 合并点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr combined_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    *combined_cloud = *cloud_tgt_o + *icp_result; // 将目标点云和配准后的源点云合并
    // 保存合并后的点云
    pcl::io::savePCDFileASCII("/home/shenlu/airplane_ws/src/airplane_formation_control/pointcloud/combined_result.pcd", *combined_cloud); 
 
   return (0);
}