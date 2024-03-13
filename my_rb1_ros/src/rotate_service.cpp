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
    my_rb1_ros::Rotate::Response rotate_res_; 

public:
    RotateService() : is_rotating_(false), current_yaw_(0.0), desired_yaw_(0.0), tolerance_(0.05) {
        service_ = nh_.advertiseService("/rotate_robot", &RotateService::rotateCallback, this);
        odom_sub_ = nh_.subscribe("/odom", 1, &RotateService::odomCallback, this);
        twist_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        current_yaw_ = tf::getYaw(msg->pose.pose.orientation);
        if (is_rotating_) {
            double yaw_error = desired_yaw_ - current_yaw_;
            yaw_error = atan2(sin(yaw_error), cos(yaw_error)); 

            if (std::abs(yaw_error) > tolerance_) {
                double Kp = 1.0; 
                double angular_velocity = std::max(std::min(Kp * yaw_error, 0.5), -0.5); 
                geometry_msgs::Twist twist_cmd;
                twist_cmd.angular.z = angular_velocity;
                twist_pub_.publish(twist_cmd);
            } else {
                geometry_msgs::Twist stop_twist;
                stop_twist.angular.z = 0;
                twist_pub_.publish(stop_twist);
                is_rotating_ = false;
                
                rotate_res_.result = "Robot rotated successfully.";
                
            }
        }
    }

    bool rotateCallback(my_rb1_ros::Rotate::Request &req, my_rb1_ros::Rotate::Response &res) {
        ROS_INFO("Received request to rotate by %d degrees.", req.degrees);
        desired_yaw_ = current_yaw_ + req.degrees * M_PI / 180.0;
        desired_yaw_ = atan2(sin(desired_yaw_), cos(desired_yaw_)); 

        is_rotating_ = true;
        
        
        
        ros::Rate rate(10);
        while (is_rotating_) {
            ros::spinOnce(); 
            rate.sleep();
        }

        
        res.result = rotate_res_.result; 
        return true;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "rotate_service");
    RotateService rotate_service;
    ros::spin();
    return 0;
}