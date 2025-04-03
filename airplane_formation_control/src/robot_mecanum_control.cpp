#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <string>
#include <cmath>

class RobotMecanumControl {
public:
    RobotMecanumControl() {
        ros::NodeHandle nh("~");
        ros::NodeHandle global_nh;
        // Get the robot name parameter
        std::string robot_name;
        nh.param<std::string>("topic_name", robot_name, "robot");

        // Subscriber for cmd_vel
        cmd_vel_x_sub = nh.subscribe(
            "/" + robot_name + "/cmd_vel_x",
            1,
            &RobotMecanumControl::cmdvel_x_callback,
            this
        );
        cmd_vel_y_sub = nh.subscribe(
            "/" + robot_name + "/cmd_vel_y",
            1,
            &RobotMecanumControl::cmdvel_y_callback,
            this
        );

        // Publishers for wheel controllers
        pub_steerL = global_nh.advertise<std_msgs::Float64>(
            "/" + robot_name + "/front_left_velocity_controller/command",
            1
        );
        pub_steerR = global_nh.advertise<std_msgs::Float64>(
            "/" + robot_name + "/front_right_velocity_controller/command",
            1
        );
        pub_rearL = global_nh.advertise<std_msgs::Float64>(
            "/" + robot_name + "/rear_left_velocity_controller/command",
            1
        );
        pub_rearR = global_nh.advertise<std_msgs::Float64>(
            "/" + robot_name + "/rear_right_velocity_controller/command",
            1
        );

        // Initialize variables
        v_x = 0.0;
        v_y = 0.0;
        omega = 0.0;
        timeout = ros::Duration(0.2);
        last_msg_time = ros::Time::now();

        // Main loop
        ros::Rate rate(10); // 10 Hz
        while (ros::ok()) {
            cmd_publish();
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber cmd_vel_x_sub,cmd_vel_y_sub;
    ros::Publisher pub_steerL, pub_steerR, pub_rearL, pub_rearR;

    double v_x; // Linear velocity x
    double v_y; // Linear velocity y
    double omega; // Angular velocity
    double base_length = 1.2; // Distance between front and rear wheels
    double base_width = 1.0; // Distance between left and right wheels
    ros::Duration timeout;
    ros::Time last_msg_time;

    void cmdvel_x_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        v_x = msg->linear.x;
        omega = msg->angular.z;
        last_msg_time = ros::Time::now();
    }
    void cmdvel_y_callback(const geometry_msgs::Twist::ConstPtr& msg) {
        v_y = msg->linear.x;
        // 不可以再设置角速度，防止冲突
        last_msg_time = ros::Time::now();
    }

    void cmd_publish() {
        ros::Duration delta_last_msg_time = ros::Time::now() - last_msg_time;
        if (delta_last_msg_time > timeout) {
            // Timeout: stop the robot
            v_x = 0.0;
            v_y = 0.0;
            omega = 0.0;
            publishToAllWheels(0.0, 0.0, 0.0, 0.0);
            return;
        }

        // Calculate wheel velocities based on linear and angular velocities
        if (std::abs(omega) > 1e-10 || std::abs(v_x) > 1e-10 || std::abs(v_y) > 1e-10) {
            // 解算轮子的速度
            double front_left_speed = v_x - v_y - omega * (base_length + base_width + 0.2);
            double front_right_speed = v_x + v_y + omega * (base_length + base_width + 0.2);
            double rear_left_speed = v_x + v_y - omega * (base_length + base_width + 0.2);
            double rear_right_speed = v_x - v_y + omega * (base_length + base_width + 0.2);
            publishToAllWheels(front_left_speed, front_right_speed, rear_left_speed, rear_right_speed);
        } else {
            // Stop the robot if no significant velocity commands
            publishToAllWheels(0.0, 0.0, 0.0, 0.0);
        }
    }

    void publishToAllWheels(double front_left_speed, double front_right_speed, double rear_left_speed, double rear_right_speed) {
        std_msgs::Float64 msg;

        // Publish to rear left wheel
        msg.data = rear_left_speed;
        pub_rearL.publish(msg);

        // Publish to rear right wheel
        msg.data = rear_right_speed;
        pub_rearR.publish(msg);

        // Publish to front left wheel
        msg.data = front_left_speed;
        pub_steerL.publish(msg);

        // Publish to front right wheel
        msg.data = front_right_speed;
        pub_steerR.publish(msg);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_mecanum_control");
    try {
        RobotMecanumControl node;
    } catch (const ros::Exception& e) {
        ROS_ERROR("Error: %s", e.what());
    }
    return 0;
}
