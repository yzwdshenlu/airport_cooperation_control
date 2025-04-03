#include <ros/ros.h>
#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>

class MultiModelPathPublisher {
public:
    MultiModelPathPublisher() : nh_("~") {
        // 初始化目标模型列表
        target_models_ = {"robot0", "robot1"};
        frame_id_ = "map";

        // 为每个模型初始化路径数据和发布器
        for (const auto& model : target_models_) {
            // 创建路径消息
            nav_msgs::Path path;
            path.header.frame_id = frame_id_;
            
            // 存储路径数据
            model_paths_[model] = path;
            
            // 创建话题发布器
            ros::Publisher pub = nh_.advertise<nav_msgs::Path>(model + "/path", 1);
            model_pubs_[model] = pub;
            
            ROS_INFO_STREAM("Tracking model: " << model);
        }

        // 订阅Gazebo模型状态
        model_sub_ = nh_.subscribe("/gazebo/model_states", 1, 
                                  &MultiModelPathPublisher::modelStatesCallback, this);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        // 遍历所有目标模型
        for (const auto& model : target_models_) {
            // 查找模型索引
            int model_index = -1;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                if (msg->name[i] == model) {
                    model_index = i;
                    break;
                }
            }
            
            if (model_index == -1) {
                ROS_WARN_ONCE("Model %s not found in model_states!", model.c_str());
                continue;
            }
            
            // 创建位姿信息
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = frame_id_;
            pose.pose = msg->pose[model_index];
            
            // 更新对应模型的路径
            model_paths_[model].poses.push_back(pose);
            model_paths_[model].header.stamp = ros::Time::now();
            
            // 发布对应模型的路径
            model_pubs_[model].publish(model_paths_[model]);
        }
    }

    ros::NodeHandle nh_;
    ros::Subscriber model_sub_;
    
    // 存储模型相关数据
    std::vector<std::string> target_models_;
    std::map<std::string, nav_msgs::Path> model_paths_;
    std::map<std::string, ros::Publisher> model_pubs_;
    std::string frame_id_;
};

int main(int argc,char** argv) {
    ros::init(argc, argv, "multi_model_path_publisher");
    MultiModelPathPublisher publisher;
    ros::spin();
    return 0;
}