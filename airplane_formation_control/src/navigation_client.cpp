// #include <ros/ros.h>
// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include <geometry_msgs/PoseStamped.h>
// #include <tf/transform_listener.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Matrix3x3.h>
// #include <cmath>

// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// // 使用SimpleActionClient实现机器人的单点导航

// class NavigationClient
// {
// public:
//     NavigationClient()
//         : ac_("/robot0/move_base", true)
//     {
//         ROS_INFO("Waiting for move_base action server...");
//         ac_.waitForServer();
//         ROS_INFO("Connected to move_base action server");
//     }

//     void beginNavigation()
//     {
//         geometry_msgs::PoseStamped target_pose;
//         target_pose.pose.position.x = -40.0;
//         target_pose.pose.position.y = -92.0;
//         target_pose.pose.position.z = 0;
//         target_pose.pose.orientation.w = 1.0;
//         target_pose.pose.orientation.x = 0.0;
//         target_pose.pose.orientation.y = 0.0;
//         target_pose.pose.orientation.z = 0.0;

        
//         goal_.target_pose.header.frame_id = "map";
//         goal_.target_pose.header.stamp = ros::Time::now();
//         goal_.target_pose.pose = target_pose.pose;

//         ac_.sendGoal(goal_);

//         ac_.waitForResult();

//         actionlib::SimpleClientGoalState state = ac_.getState();
//         if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
//         {
//             ROS_INFO("Goal reached successfully.");
//         }
//         else
//         {
//             ROS_WARN("Goal failed with state: %s", state.toString().c_str());
//         }
//     }

// private:
//     MoveBaseClient ac_;
//     move_base_msgs::MoveBaseGoal goal_;
// };

// int main(int argc, char** argv)
// {
//     ros::init(argc, argv, "navigation_client");
//     NavigationClient client;
//     client.beginNavigation();
//     return 0;
// }

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class NavigationClient
{
public:
    NavigationClient()
        : ac_("/robot0/move_base", true)
    {
        ROS_INFO("Waiting for move_base action server...");
        ac_.waitForServer();
        ROS_INFO("Connected to move_base action server");
    }

    void beginNavigation()
    {
        // 多个目标点列表
        std::vector<geometry_msgs::Pose> goals = {
            createPose(-40.0, -92.0, 0.0),
            createPose(-39.0, -92.0, 0.0),
            createPose(-39.0, -91.0, 0.0)
        };

        for (size_t i = 0; i < goals.size(); ++i)
        {
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose = goals[i];

            ROS_INFO("Sending goal %zu: (%.2f, %.2f)", i + 1, goals[i].position.x, goals[i].position.y);
            ac_.sendGoal(goal);
            ac_.waitForResult();

            actionlib::SimpleClientGoalState state = ac_.getState();
            if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ROS_INFO("Goal %zu reached successfully.", i + 1);
            }
            else
            {
                ROS_WARN("Goal %zu failed: %s", i + 1, state.toString().c_str());
            }
        }
    }

private:
    MoveBaseClient ac_;

    geometry_msgs::Pose createPose(double x, double y, double yaw_deg)
    {
        geometry_msgs::Pose pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = 0;

        double yaw_rad = yaw_deg * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw_rad);
        pose.orientation.x = q.x();
        pose.orientation.y = q.y();
        pose.orientation.z = q.z();
        pose.orientation.w = q.w();

        return pose;
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_client");
    NavigationClient client;
    client.beginNavigation();
    return 0;
}

