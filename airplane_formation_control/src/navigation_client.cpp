#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "navigation_client");
    MoveBaseClient ac("move_base", true);


    while (!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    // 设置目标点
    goal.target_pose.pose.position.x = -39.0;
    goal.target_pose.pose.position.y = -98.0;
    goal.target_pose.pose.orientation.w = 1.0;


    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("Mission success!!");
    }
    else
    {
        ROS_INFO("The base failed to move to the goal!!");
    }

    return 0;
}



// void NavResultCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_WARN("[NavResultCallback] %s", msg->data.c_str());
// }

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "my_navigation");
//     ros::NodeHandle nh;
//     ros::Publisher pub = nh.advertise<std_msgs::String>("navi_point", 10);
//     ros::Subscriber sub = nh.subscribe("navi_result",10,NavResultCallback);

//     sleep(1);

//     std_msgs::String nav_msg;
//     nav_msg.data = "1";
//     pub.publish(nav_msg);
//     ros::spin();
//     return 0;
// }
