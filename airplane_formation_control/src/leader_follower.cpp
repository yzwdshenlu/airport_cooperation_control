#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <gazebo_msgs/ModelStates.h>
#include <cmath>
const double PI = M_PI;
double w_leader = 0;
double v_leader = 0;
double delta_x = 0;
double delta_y = 0;
double delta_theta = 0;
double dis = 0;
double d = 0;
const double MAX_VEL = 3;
const double MAX_ANG_VEL = 3;

ros::Publisher follower_vel;
ros::Subscriber leader_vel_sub;
std::string leader_robot_name, follower_robot_name;

void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // find the index of the leader and the follower
    int leader_index = -1;
    int follower_index = -1;
    for (int i = 0; i < msg->name.size(); ++i)
    {
        if (msg->name[i] == leader_robot_name)
        {
            leader_index = i;
        }
        if (msg->name[i] == follower_robot_name)
        {
            follower_index = i;
        }
    }

    // check if the leader and the follower are found
    if (leader_index == -1)
    {
        ROS_WARN("Leader robot not found in ModelStates.");
        return;
    }
    if (follower_index == -1)
    {
        ROS_WARN("Follower robot not found in ModelStates.");
        return;
    }

    // get the leader's velocity
    w_leader = msg->twist[leader_index].angular.z; // get the angular velocity
    v_leader = msg->twist[leader_index].linear.x;  // get the linear velocity

    // Compute the relative position between the leader and the follower
    geometry_msgs::Pose leader_pose = msg->pose[leader_index];
    geometry_msgs::Pose follower_pose = msg->pose[follower_index];


    double leader_yaw = tf::getYaw(leader_pose.orientation);
    double follower_yaw = tf::getYaw(follower_pose.orientation);

    delta_x = (leader_pose.position.x - follower_pose.position.x - d * std::cos(follower_yaw)) * std::cos(leader_yaw) + 
                (leader_pose.position.y - follower_pose.position.y - d * std::sin(follower_yaw)) * std::sin(leader_yaw);
    delta_y = -1 * (leader_pose.position.x - follower_pose.position.x - d * std::cos(follower_yaw)) * std::sin(leader_yaw) + 
                (leader_pose.position.y - follower_pose.position.y - d * std::sin(follower_yaw)) * std::cos(leader_yaw);

    delta_theta = follower_yaw - leader_yaw;

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leader_follower");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh_global;
    // 延迟启动
    ros::Duration(5.0).sleep();  // 延迟5秒
    leader_robot_name = nh.param<std::string>("leader_robot_name", "default_leader");
    follower_robot_name = nh.param<std::string>("follower_robot_name", "default_follower");
    double L0 = nh.param<double>("expected_distance", 1.0);
    double theta0 = nh.param<double>("expected_theta", 3.14) / 3.14 * PI;//提高精度
    d = nh.param<double>("front_distance",0.1);
    double k_1 = 1.0;
    double k_2 = 1.0;

    leader_vel_sub = nh_global.subscribe("/gazebo/model_states", 10, modelStatesCallback);

    follower_vel = nh_global.advertise<geometry_msgs::Twist>(follower_robot_name + "/airport_robot/cmd_vel", 1);


    ros::Rate rate(10.0);

    while (ros::ok()) {
        // ROS_INFO("robot_name: %s,delta_x: %f, delta_y: %f, delta_theta: %f", follower_robot_name.c_str(),delta_x, delta_y, delta_theta);
        // ROS_INFO("name : %s",follower_robot_name.c_str());
        // Compute the error in the follower's position
        double err_x = L0 * std::cos(theta0) - delta_x;
        double err_y = L0 * std::sin(theta0) - delta_y;
        // ROS_INFO("robot_name: %s,delta_x: %f, delta_y: %f, delta_theta: %f", follower_robot_name.c_str(),delta_x, delta_y, delta_theta);
        // ROS_INFO("robot_name: %s,err_x: %f, err_y: %f", follower_robot_name.c_str(),err_x, err_y);
        // Compute the follower's velocity and angular velocity
        double v_follower = (-k_1 * err_x + delta_y * w_leader + v_leader) * std::cos(delta_theta)
                           + (-k_2 * err_y - delta_x * w_leader) * std::sin(delta_theta);
        double w_follower = ((-k_2 * err_y - delta_x * w_leader) * std::cos(delta_theta)
                           - (-k_1 * err_x + delta_y * w_leader + v_leader) * std::sin(delta_theta)) / d;

        // Limit the velocities and angular velocities
        if (v_follower > MAX_VEL)
        {
            v_follower = MAX_VEL;
        }
        if (v_follower < -MAX_VEL)
        {
            v_follower = -MAX_VEL;
        }
        if (w_follower > MAX_ANG_VEL)
        {
            w_follower = MAX_ANG_VEL;
        }
        if (w_follower < -MAX_ANG_VEL)
        {
            w_follower = -MAX_ANG_VEL;
        }
        
        // Publish the computed velocities
        geometry_msgs::Twist msg;
        msg.linear.x = v_follower;
        msg.angular.z = w_follower;
        // ROS_INFO("v_leader: %f, w_leader: %f", v_leader, w_leader);
        // ROS_INFO("robot_name: %s,v_follower: %f, w_follower: %f", follower_robot_name.c_str(),v_follower, w_follower);
        follower_vel.publish(msg);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
