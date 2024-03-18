#include "ros/ros.h"
#include "my_rb1_ros/Rotate.h" 
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <tf/tf.h>

class RotateService {
private:
    ros::NodeHandle nh_;
    ros::ServiceServer service_;
    ros::Subscriber odom_sub_;
    ros::Publisher twist_pub_; 
    double current_yaw_;
    double desired_yaw_; 
    double tolerance_;
    bool is_rotating_;

public:
    RotateService() : is_rotating_(false), current_yaw_(0.0), desired_yaw_(0.0), tolerance_(0.05) {
        service_ = nh_.advertiseService("/rotate_robot", &RotateService::rotateCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
        ROS_INFO("Service Ready");
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_yaw_ = tf::getYaw(msg->pose.pose.orientation);
        if (is_rotating_) {
            adjustRobotOrientation();
        }
    }

    void adjustRobotOrientation() {
        double yaw_error = atan2(sin(desired_yaw_ - current_yaw_), cos(desired_yaw_ - current_yaw_));
        if (std::abs(yaw_error) > tolerance_) {
            double Kp = 2.0; // Increased Kp for more aggressive response
            double max_angular_velocity = 1.5; // Higher limit for faster rotation
            geometry_msgs::Twist twist_cmd;
            twist_cmd.angular.z = std::max(std::min(Kp * yaw_error, max_angular_velocity), -max_angular_velocity);
            twist_pub_.publish(twist_cmd);
        } else {
            geometry_msgs::Twist stop_twist;
            stop_twist.angular.z = 0;
            twist_pub_.publish(stop_twist);
            is_rotating_ = false;
        }
    }

    bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
        ROS_INFO("Service Requested: Received request to rotate by %d degrees.", req.degrees);
        desired_yaw_ = fmod(current_yaw_ + req.degrees * M_PI / 180.0, 2 * M_PI);
        if (desired_yaw_ < 0) desired_yaw_ += 2 * M_PI; // Ensure desired_yaw_ is always positive

        is_rotating_ = true;

        ros::Rate rate(50);
        while (is_rotating_ && ros::ok()) {
            ros::spinOnce(); 
            rate.sleep();
        }

        res.result = "Robot rotated successfully.";
        ROS_INFO("Service Completed: Robot Rotated Successfully");
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotate_service");
    RotateService rotate_service;
    ros::spin();
    return 0;
}